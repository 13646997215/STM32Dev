#include "test.h"
#include "runtime.h"
#include "board.h"
#include <string.h>
static uint8_t rx[256],tx[256],imu_rx[1024],last_query[3];
static size_t rn,ri,tn,imu_rn,imu_ri;
static unsigned apply,stops,queries,toggles,nonzero;
static float applied_duty[2];
static int key,respond=1,accept_query=1,corrupt_response;static uint32_t errors;
static int16_t sensor_pitch;
void Board_Init(void){} uint32_t Board_Micros(void){return 1000;}
int Board_Read(unsigned port,uint8_t*b){
 if(port==0){if(imu_ri==imu_rn)return 0;*b=imu_rx[imu_ri++];return 1;}
 if(port!=1||ri==rn)return 0;*b=rx[ri++];return 1;
}
int Board_Write(unsigned port,const uint8_t*p,size_t n){
 if(port==0){
  uint8_t frame[]={0x5a,0x5a,0x45,6,0,0,0,0,0,0,0};unsigned i;
  if(!accept_query)return 0;
  if(n!=3)return 0;
  memcpy(last_query,p,3);queries++;
  if(p[1]==0x45||p[1]==0x15||p[1]==0x35)toggles++;
  if(respond&&p[0]==0xa5&&p[1]==0x95&&p[2]==0x3a){
   frame[6]=(uint8_t)((uint16_t)sensor_pitch>>8);frame[7]=(uint8_t)sensor_pitch;
   frame[8]=(uint8_t)((queries%18000U)>>8);frame[9]=(uint8_t)(queries%18000U);
   for(i=0;i<10;i++)frame[10]+=frame[i];
   if(corrupt_response)frame[10]^=1;
   if(imu_ri==imu_rn)imu_ri=imu_rn=0;
   if(imu_rn+sizeof(frame)<=sizeof(imu_rx)){memcpy(imu_rx+imu_rn,frame,sizeof(frame));imu_rn+=sizeof(frame);}
  }
  return 1;
 }
 if(port!=1||n>sizeof(tx))return 0;memcpy(tx,p,n);tn=n;return 1;
}
void Board_Snapshot(FgCapture*f){memset(f,0,2*sizeof(*f));}
void Board_Apply(const float*d,const int8_t*s,const RobotConfig*c){(void)s;(void)c;apply++;applied_duty[0]=d[0];applied_duty[1]=d[1];if(d[0]!=0||d[1]!=0)nonzero++;}
void Board_Stop(void){stops++;}void Board_Led(int on){(void)on;}int Board_Key(void){return key;}
uint32_t Board_Errors(void){return errors;}void Board_Fatal(void){stops++;}
static void reset_fixture(void){
 rn=ri=tn=imu_rn=imu_ri=0;apply=stops=queries=toggles=nonzero=0;
 key=corrupt_response=0;sensor_pitch=0;errors=0;respond=accept_query=1;memset(last_query,0,3);Runtime_Init();
}
static void request(uint8_t selector,uint16_t seq,uint32_t now){
 WireFrame f;memset(&f,0,sizeof(f));f.type=CMD_IMU_QUERY;f.sequence=seq;f.length=1;f.payload[0]=selector;
 ri=0;rn=Wire_Encode(&f,rx,sizeof(rx));Runtime_Tick(now);
}
int main(void){WireFrame f;unsigned i;
 Runtime_Init();CHECK(!g_course.armed&&stops);
 /* The symmetry probe must exist and start at zero, not at some stale value. */
 CHECK(g_rpm_l==0&&g_rpm_r==0&&g_rpm_diff==0);
 memset(&f,0,sizeof(f));f.type=CMD_PING;f.sequence=10;rn=Wire_Encode(&f,rx,sizeof(rx));Runtime_Tick(1);
 CHECK(g_course.frames==1&&tn>0&&apply==1);CHECK(tx[3]==MSG_ACK&&tx[6]==10);
 /* A short hold must no longer stop the round: interrupting needs 2 s (test_key
    covers the 2 s boundary; here we only check the integration wiring). */
 g_course.armed=1;key=1;for(i=10;i<60;i+=10)Runtime_Tick(i);CHECK(g_course.armed);key=0;
 errors=1;g_course.armed=1;Runtime_Tick(70);CHECK(!g_course.armed&&g_course.fault==FAULT_UART);
 /* Repeated diagnostic reads must be single-shot commands, never stream toggles. */
 reset_fixture();request(2,100,1);CHECK(queries==1);CHECK(last_query[1]==0x95&&last_query[2]==0x3a);
 request(2,100,2);CHECK(queries==2&&toggles==0);CHECK(g_course.imu.euler_seq==1);
 request(0,101,3);CHECK(last_query[1]==0xc5&&last_query[2]==0x6a);
 request(1,102,4);CHECK(last_query[1]==0xe5&&last_query[2]==0x8a);
 request(3,103,5);CHECK(queries==4&&tx[3]==MSG_ERROR);
 /* The real Runtime must acquire independently of any PC command for 30 minutes. */
 reset_fixture();for(i=0;i<=1800000U;i+=10U){
  Runtime_Tick(i);
  if(i&&i%1000U==0){CHECK(Imu_Fresh(&g_course.imu,i,200));CHECK(!g_course.armed&&nonzero==0);}
 }
 CHECK(queries==90000U);CHECK(toggles==0);CHECK(g_course.imu.euler_seq==89999U);
 CHECK(Imu_Fresh(&g_course.imu,1800000U,200U));CHECK(g_course.frames==0);
 CHECK(!g_course.armed&&g_course.fault==FAULT_NONE&&nonzero==0);
 CHECK(g_imu_query_sent==90000U&&g_imu_query_failed==0&&g_imu_last_command==0x95);
 CHECK(g_imu_last_query_ms==1800000U);
 /* Missing responses stay stale; later real responses restore freshness without a click. */
 reset_fixture();for(i=0;i<=100;i+=10)Runtime_Tick(i);CHECK(g_course.imu.have_euler);
 respond=0;for(i=110;i<=1000;i+=10)Runtime_Tick(i);
 CHECK(!Imu_Fresh(&g_course.imu,1000,200));respond=1;
 Runtime_Tick(1020);Runtime_Tick(1030);CHECK(Imu_Fresh(&g_course.imu,1030,200));
 CHECK(toggles==0&&nonzero==0);
 /* A full TX queue does not block control; next scheduled request can succeed. */
 reset_fixture();accept_query=0;Runtime_Tick(20);CHECK(queries==0&&apply==1);
 CHECK(g_imu_query_sent==0&&g_imu_query_failed==1&&g_imu_last_query_ms==0);
 accept_query=1;Runtime_Tick(40);Runtime_Tick(50);CHECK(queries==1&&g_course.imu.have_euler);
 CHECK(g_imu_query_sent==1&&g_imu_query_failed==1&&g_imu_last_query_ms==40);
 /* ACK OK alone never makes stale/missing data valid; bad checksums remain rejected. */
 reset_fixture();accept_query=0;request(2,200,1);CHECK(tx[3]==MSG_ERROR&&tx[9]==RESULT_NOT_READY);
 CHECK(g_imu_query_failed==1&&!g_course.imu.have_euler);
 reset_fixture();corrupt_response=1;for(i=0;i<=1000;i+=10)Runtime_Tick(i);
 CHECK(g_imu_query_sent==50&&g_course.imu.bad>0&&!Imu_Fresh(&g_course.imu,1000,200));
 corrupt_response=0;Runtime_Tick(1020);Runtime_Tick(1030);CHECK(Imu_Fresh(&g_course.imu,1030,200));
 /* Unsigned schedule arithmetic across the tick wrap; no catch-up burst on a delay. */
 reset_fixture();Runtime_Tick(0xfffffff0U);CHECK(queries==1);
 Runtime_Tick(0xfffffffaU);CHECK(queries==1);Runtime_Tick(4);CHECK(queries==2);
 Runtime_Tick(14);CHECK(queries==2);Runtime_Tick(24);CHECK(queries==3);
 Runtime_Tick(1000000);CHECK(queries==4);
 /* New bench commands pass through the actual wire/runtime dispatch to bounded outputs. */
 reset_fixture();Runtime_Tick(20);Runtime_Tick(30);
 memset(&f,0,sizeof(f));f.type=CMD_TASK1_BENCH_START;f.sequence=900;f.length=4;memcpy(f.payload,"T1LV",4);
 ri=0;rn=Wire_Encode(&f,rx,sizeof(rx));Runtime_Tick(40);
 CHECK(tx[3]==MSG_ACK&&tx[8]==CMD_TASK1_BENCH_START&&tx[9]==RESULT_OK);
 CHECK(g_course.mode==MODE_BENCH_TASK1&&g_course.config.cal_flags==0);
 NEAR(applied_duty[0],0,.0001);NEAR(applied_duty[1],0,.0001);
 sensor_pitch=500;Runtime_Tick(50);Runtime_Tick(60);Runtime_Tick(70);
 CHECK(applied_duty[0]>=.4f&&applied_duty[0]<=.45f);NEAR(applied_duty[0],applied_duty[1],.0001);
 f.type=CMD_STOP;f.length=0;f.sequence=901;ri=0;rn=Wire_Encode(&f,rx,sizeof(rx));Runtime_Tick(71);
 CHECK(!g_course.armed&&applied_duty[0]==0&&applied_duty[1]==0);
 DONE();}
