#include "test.h"
#define BENCH_START 11
#define BENCH_KEEP 12
#define BENCH_MODE 4

static void sample(CourseApp *a,uint32_t now,float pitch,float yaw)
{a->imu.have_euler=1;a->imu.euler_ms=now;a->imu.euler_seq++;a->imu.euler_deg[1]=pitch;a->imu.euler_deg[2]=yaw;}
static void setup(CourseApp *a,FgCapture fg[2])
{App_Init(a);memset(fg,0,2*sizeof(*fg));a->config.yaw_sign=-1;sample(a,0,0,40);}
static uint8_t start(CourseApp *a,uint16_t seq,uint32_t now)
{WireFrame f={0};f.type=BENCH_START;f.sequence=seq;f.length=4;memcpy(f.payload,"T1LV",4);return App_Command(a,&f,now);}
static uint8_t keep(CourseApp *a,uint16_t token,uint32_t now)
{WireFrame f={0};f.type=BENCH_KEEP;f.length=2;f.payload[0]=(uint8_t)token;f.payload[1]=(uint8_t)(token>>8);return App_Command(a,&f,now);}
/* flags: actual test sample, left FG, right FG, matching PC keepalive. */
static void tick(CourseApp *a,FgCapture fg[2],uint32_t now,float pitch,float yaw,unsigned flags)
{
 unsigned i;if(flags&1)sample(a,now,pitch,yaw);
 for(i=0;i<2;i++)if(flags&(2U<<i)){fg[i].have_edge=fg[i].have_period=1;fg[i].last_us=now*1000U;fg[i].period_us=2000;fg[i].total+=5;}
 if((flags&8)&&now%100U==0)keep(a,31,now);
 App_Step(a,fg,now,now*1000U);
}
int main(void)
{
 CourseApp a;FgCapture fg[2];WireFrame f={0};uint32_t ms;unsigned i;
 setup(&a,fg);f.type=CMD_ARM;f.length=2;f.payload[0]=0xde;f.payload[1]=0xc0;
 CHECK(App_Command(&a,&f,0)==RESULT_NOT_READY);
 CHECK(start(&a,31,0)==RESULT_OK);CHECK(a.armed&&a.mode==BENCH_MODE&&a.config.cal_flags==0);
 tick(&a,fg,10,0,40,9);CHECK(a.mission.state==MS_HOLD&&!a.pose.valid);
 NEAR(a.pwm[0],0,.0001);NEAR(a.pwm[1],0,.0001);
 CHECK(a.direction[0]==1&&a.direction[1]==1);
 tick(&a,fg,20,5,30,9);CHECK(a.pwm[0]>=.40f&&a.pwm[0]<=.45001f);
 NEAR(a.pwm[0],a.pwm[1],.00001);
 tick(&a,fg,30,5,150,9);NEAR(a.pwm[0],a.pwm[1],.00001);CHECK(a.pwm[0]>=.40f);
 CHECK(!a.direction_known[0]&&!a.direction_known[1]);
 /* Normal controls and config cannot take over a running explicit bench. */
 memset(&f,0,sizeof(f));f.type=CMD_START;f.length=1;f.payload[0]=PROFILE_A;
 CHECK(App_Command(&a,&f,31)==RESULT_LOCKED);
 f.type=CMD_DUTY;f.length=8;Wire_PutFloat(f.payload,.4f);Wire_PutFloat(f.payload+4,.4f);
 CHECK(App_Command(&a,&f,32)==RESULT_LOCKED);
 f.type=CMD_CONFIG;f.length=6;f.payload[0]=10;f.payload[1]=0;Wire_PutFloat(f.payload+2,.9f);
 CHECK(App_Command(&a,&f,33)==RESULT_LOCKED);
 CHECK(start(&a,31,34)==RESULT_DUPLICATE);CHECK(start(&a,32,35)==RESULT_LOCKED);
 f.type=CMD_STOP;f.length=0;CHECK(App_Command(&a,&f,36)==RESULT_OK);
 CHECK(!a.armed&&a.pwm[0]==0&&a.pwm[1]==0);
 CHECK(start(&a,31,37)==RESULT_DUPLICATE&&!a.armed);

 /* Tilting 3+ degrees is a control input, never successful completion. */
 setup(&a,fg);start(&a,31,0);
 for(ms=10;ms<=900;ms+=10)tick(&a,fg,ms,6,40,15);
 CHECK(a.armed&&a.mission.state==MS_BALANCE);
 for(ms=910;ms<=1090;ms+=10)tick(&a,fg,ms,0,40,15);
 for(ms=1100;ms<=1290;ms+=10)tick(&a,fg,ms,6,40,15);
 CHECK(a.armed&&a.mission.state==MS_BALANCE);
 tick(&a,fg,1300,6,40,15);CHECK(a.armed&&a.fault==0&&a.pwm[0]>=.4f);
 CHECK(a.config.cal_flags==0);CHECK(keep(&a,31,1310)==RESULT_OK);
 /* Zero stops immediately; hysteresis prevents jitter, and later tilt resumes. */
 setup(&a,fg);start(&a,31,0);
 for(ms=10;ms<=3000;ms+=10)tick(&a,fg,ms,0,170,9);
 CHECK(a.armed&&a.fault==0&&a.pwm[0]==0&&a.pwm[1]==0);
 tick(&a,fg,3010,.7f,-170,9);CHECK(a.pwm[0]==0);
 tick(&a,fg,3020,5,-170,9);CHECK(a.pwm[0]>=.4f&&a.direction[0]==1);
 tick(&a,fg,3030,.7f,0,9);CHECK(a.pwm[0]>=.4f);
 tick(&a,fg,3040,.4f,0,9);CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 /* Reverse only after both outputs have been zero and FG has gone quiet. */
 tick(&a,fg,3050,-5,0,15);CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 for(ms=3060;ms<=3240;ms+=10){tick(&a,fg,ms,-5,90,9);CHECK(a.pwm[0]==0&&a.pwm[1]==0);}
 for(ms=3250;ms<=3400;ms+=10)tick(&a,fg,ms,-5,-90,9);
 CHECK(a.armed&&a.direction[0]==-1&&a.direction[1]==-1);
 CHECK(a.pwm[0]>=.4f&&a.pwm[0]<=.45f);NEAR(a.pwm[0],a.pwm[1],.00001);
 tick(&a,fg,3410,0,0,9);CHECK(a.armed&&a.pwm[0]==0&&a.pwm[1]==0);
 /* One wheel coasting cannot allow the other wheel to power in reverse. */
 setup(&a,fg);start(&a,31,0);
 for(ms=10;ms<=500;ms+=10)tick(&a,fg,ms,5,0,15);
 for(ms=510;ms<=1500;ms+=10){tick(&a,fg,ms,-5,120,11);CHECK(a.pwm[0]==0&&a.pwm[1]==0&&a.armed);}
 for(ms=1510;ms<=1800;ms+=10)tick(&a,fg,ms,-5,-120,9);
 CHECK(a.armed&&a.pwm[0]>=.4f&&a.direction[0]==-1&&a.direction[1]==-1);
 /* A wheel which never goes quiet must fault, never hot-switch direction. */
 setup(&a,fg);start(&a,31,0);tick(&a,fg,10,5,0,15);
 for(ms=20;ms<=2530;ms+=10)tick(&a,fg,ms,-5,0,15);
 CHECK(!a.armed&&a.fault==FAULT_MOTOR&&a.pwm[0]==0&&a.pwm[1]==0);
 /* Startup reference is configured zero, not the current tilted sample. */
 setup(&a,fg);sample(&a,0,5,100);start(&a,31,0);tick(&a,fg,10,5,100,9);
 CHECK(a.pwm[0]>=.4f&&a.direction[0]==1);
 setup(&a,fg);a.config.pitch_zero_deg=5;sample(&a,0,5,100);start(&a,31,0);
 tick(&a,fg,10,5,-100,9);CHECK(a.pwm[0]==0&&a.pwm[1]==0);

 /* PC loss, stale IMU, missing FG and long run all stop the physical outputs. */
 setup(&a,fg);start(&a,31,0);for(ms=10;ms<=500;ms+=10)tick(&a,fg,ms,0,40,7);
 CHECK(a.armed);tick(&a,fg,510,0,40,7);CHECK(!a.armed&&a.fault==FAULT_TIMEOUT);
 CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 setup(&a,fg);start(&a,31,0);for(ms=10;ms<=210;ms+=10)tick(&a,fg,ms,0,40,14);
 CHECK(!a.armed&&a.fault==FAULT_IMU&&a.pwm[0]==0&&a.pwm[1]==0);
 setup(&a,fg);start(&a,31,0);for(ms=10;ms<=1520;ms+=10)tick(&a,fg,ms,5,40,11);
 CHECK(!a.armed&&a.fault==FAULT_FEEDBACK);
 setup(&a,fg);start(&a,31,0);
 for(ms=10;ms<60000;ms+=10){tick(&a,fg,ms,0,40,9);CHECK(a.pwm[0]==0&&a.pwm[1]==0);}
 CHECK(a.armed);tick(&a,fg,60000,0,40,9);CHECK(!a.armed&&a.fault==FAULT_TIMEOUT);
 setup(&a,fg);start(&a,31,0);tick(&a,fg,10,20,40,15);CHECK(!a.armed&&a.fault==FAULT_TILT);
 CHECK(start(&a,32,10)==RESULT_FAULT);sample(&a,20,0,40);CHECK(start(&a,33,20)==RESULT_FAULT);
 f.type=CMD_CLEAR;f.length=0;CHECK(App_Command(&a,&f,20)==RESULT_OK);
 for(ms=20;ms<=300;ms+=10)tick(&a,fg,ms,0,40,1);
 CHECK(start(&a,34,300)==RESULT_OK);
 setup(&a,fg);start(&a,31,0);tick(&a,fg,10,NAN,40,15);CHECK(!a.armed&&a.pwm[0]==0&&a.pwm[1]==0);

 /* Reject invalid start/keepalive, motion already present, and existing faults. */
 setup(&a,fg);a.imu.have_euler=0;CHECK(start(&a,31,0)==RESULT_NOT_READY);
 setup(&a,fg);CHECK(start(&a,31,201)==RESULT_NOT_READY);
 setup(&a,fg);a.speed[0].hz=1;CHECK(start(&a,31,0)==RESULT_NOT_READY);
 setup(&a,fg);a.config.max_duty=.44f;CHECK(start(&a,31,0)==RESULT_NOT_READY);
 setup(&a,fg);a.fault=FAULT_UART;CHECK(start(&a,31,0)==RESULT_FAULT);
 setup(&a,fg);f.type=BENCH_START;f.length=4;memcpy(f.payload,"oops",4);
 CHECK(App_Command(&a,&f,0)==RESULT_FORMAT&&!a.armed);
 memcpy(f.payload,"T1BN",4);CHECK(App_Command(&a,&f,0)==RESULT_FORMAT&&!a.armed);
 CHECK(keep(&a,31,0)==RESULT_NOT_READY);
 start(&a,31,0);CHECK(keep(&a,30,200)==RESULT_LOCKED);
 for(ms=10;ms<=510;ms+=10)tick(&a,fg,ms,0,40,7);
 CHECK(!a.armed);CHECK(keep(&a,31,520)!=RESULT_OK&&!a.armed);
 /* Timer wrap does not terminate an otherwise fresh bench. */
 setup(&a,fg);sample(&a,0xfffffff0U,0,40);CHECK(start(&a,31,0xfffffff0U)==RESULT_OK);
 for(i=1;i<=5;i++)tick(&a,fg,0xfffffff0U+10U*i,0,40,7);
 CHECK(a.armed&&a.mode==BENCH_MODE);
 DONE();
}
