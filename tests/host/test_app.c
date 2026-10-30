#include "test.h"
#include <string.h>
static WireFrame frame(uint8_t type,uint16_t n){WireFrame f;memset(&f,0,sizeof(f));f.type=type;f.length=n;f.sequence=12;return f;}
static void imu(CourseApp *a,uint32_t now){a->imu.have_euler=1;a->imu.euler_ms=now;a->imu.euler_seq++;}
int main(void){CourseApp a;FgCapture fg[2]={{0},{0}};WireFrame f;uint8_t out[96];unsigned i;
 App_Init(&a);CHECK(!a.armed&&a.mode==MODE_DIAGNOSTIC&&a.config.cal_flags==0);
 App_Step(&a,fg,0,0);CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 f=frame(CMD_ARM,2);f.payload[0]=0xDE;f.payload[1]=0xC0;CHECK(App_Command(&a,&f,0)==RESULT_NOT_READY);
 a.config.cal_flags=CAL_BOARD;CHECK(App_Command(&a,&f,1)==RESULT_OK&&a.armed);
 f=frame(CMD_CONFIG,6);f.payload[0]=1;Wire_PutFloat(f.payload+2,.068f);CHECK(App_Command(&a,&f,2)==RESULT_LOCKED);
 f=frame(CMD_VELOCITY,8);Wire_PutFloat(f.payload,15);Wire_PutFloat(f.payload+4,15);CHECK(App_Command(&a,&f,10)==RESULT_OK);
 App_Step(&a,fg,10,10000);CHECK(a.pwm[0]>0&&a.pwm[1]>0);
 for(i=20;i<=520;i+=10)App_Step(&a,fg,i,i*1000);CHECK(!a.armed&&a.pwm[0]==0&&a.pwm[1]==0);
 App_Init(&a);a.config.cal_flags=CAL_BOARD;imu(&a,0);f=frame(CMD_ARM,2);f.payload[0]=0xDE;f.payload[1]=0xC0;CHECK(App_Command(&a,&f,0)==RESULT_OK);
 f=frame(CMD_START,1);f.payload[0]=PROFILE_RECTANGLE;CHECK(App_Command(&a,&f,0)==RESULT_OK&&a.mode==MODE_AUTO);
 CHECK(App_Command(&a,&f,1)==RESULT_DUPLICATE);
 f=frame(CMD_VELOCITY,8);CHECK(App_Command(&a,&f,2)==RESULT_LOCKED);
 for(i=0;i<=600;i+=10){imu(&a,i);fg[0].have_edge=fg[1].have_edge=1;fg[0].last_us=fg[1].last_us=i*1000;App_Step(&a,fg,i,i*1000);}
 CHECK(a.armed&&a.mode==MODE_AUTO); /* PC disconnect does not cancel autonomous mission. */
 App_Step(&a,fg,610,610000);App_Step(&a,fg,900,900000);CHECK(a.fault==0&&a.armed&&a.pwm[0]==0&&(a.warnings&12U));
 f=frame(CMD_STOP,0);CHECK(App_Command(&a,&f,900)==RESULT_OK&&!a.armed);
 f=frame(CMD_CLEAR,0);CHECK(App_Command(&a,&f,901)==RESULT_OK&&!a.fault);
 f=frame(CMD_STOP,0);CHECK(App_Command(&a,&f,902)==RESULT_OK&&!a.armed);
 CHECK(App_Telemetry(&a,1000,out,sizeof(out))==88);CHECK(out[0]==1&&out[1]==MODE_DIAGNOSTIC);
 CHECK(App_Telemetry(&a,1000,out,20)==0);
 a.direction_known[0]=1;a.motor[0].phase=MOTOR_READY;a.pose.valid=1;
 App_Telemetry(&a,1000,out,sizeof(out));CHECK(Wire_GetU32(out+84)==11U);
 App_Init(&a);a.config.cal_flags=CAL_BOARD;f=frame(CMD_ARM,2);f.payload[0]=0xDE;f.payload[1]=0xC0;App_Command(&a,&f,0);
 f=frame(CMD_DUTY,8);Wire_PutFloat(f.payload,NAN);CHECK(App_Command(&a,&f,1)==RESULT_RANGE);
 Wire_PutFloat(f.payload,.45f);Wire_PutFloat(f.payload+4,-.45f);CHECK(App_Command(&a,&f,1)==RESULT_OK);
 App_Step(&a,fg,1,1000);CHECK(a.pwm[0]>.4f&&a.pwm[1]==0); /* Initial reverse must wait. */
 a.motor[0].direction=-1;a.direction[0]=-1;
 f=frame(CMD_STOP,0);App_Command(&a,&f,1);CHECK(a.pwm[0]==0&&a.pwm[1]==0&&!a.armed);
 CHECK(a.motor[0].direction==-1&&a.direction[0]==-1); /* STOP never requests a direction change. */
 /* A mounting-sign edit must rebase yaw even between two Euler frames. */
 App_Init(&a);imu(&a,10);a.imu.euler_deg[2]=45;App_Step(&a,fg,10,10000);
 NEAR(a.yaw,COURSE_PI/4,.0001);
 f=frame(CMD_CONFIG,6);f.payload[0]=32;Wire_PutFloat(f.payload+2,-1);
 CHECK(App_Command(&a,&f,11)==RESULT_OK);App_Step(&a,fg,20,20000);
 NEAR(a.yaw,-COURSE_PI/4,.0001);CHECK(!a.armed&&a.config.cal_flags==0);
 imu(&a,30);a.imu.euler_deg[2]=35;App_Step(&a,fg,30,30000);
 NEAR(a.yaw,-35*COURSE_PI/180,.0001);
 /* Reversing the sign again on a new sample is also coherent. */
 Wire_PutFloat(f.payload+2,1);CHECK(App_Command(&a,&f,31)==RESULT_OK);
 imu(&a,40);a.imu.euler_deg[2]=30;App_Step(&a,fg,40,40000);
 NEAR(a.yaw,30*COURSE_PI/180,.0001);
 Wire_PutFloat(f.payload+2,0);CHECK(App_Command(&a,&f,41)==RESULT_RANGE);
 App_Step(&a,fg,50,50000);NEAR(a.yaw,30*COURSE_PI/180,.0001);
 /* A same-value edit must preserve an already accumulated full turn. */
 a.continuous_yaw+=2*COURSE_PI;Wire_PutFloat(f.payload+2,1);
 CHECK(App_Command(&a,&f,51)==RESULT_OK);App_Step(&a,fg,60,60000);
 NEAR(a.continuous_yaw,30*COURSE_PI/180+2*COURSE_PI,.0001);
 /* Implausible FG must stop autonomous motion before contaminating odometry. */
 App_Init(&a);memset(fg,0,sizeof(fg));imu(&a,100);
 a.armed=1;a.mode=MODE_AUTO;a.mission.state=MS_CLIMB;a.mission.profile=PROFILE_BALANCE_RETURN;
 a.pose.initialized=a.pose.valid=1;a.direction_known[0]=a.direction_known[1]=1;
 fg[1].total=50;fg[1].have_edge=fg[1].have_period=1;fg[1].last_us=100000;fg[1].period_us=100;
 App_Step(&a,fg,100,100000);
 CHECK(a.fault==0&&a.armed&&(a.warnings&1U));
 NEAR(a.pose.x,0,.000001);
 /* A burst with an ordinary final pulse period must be rejected immediately,
    before Speed_Update's 20 ms count window has elapsed. */
 App_Init(&a);memset(fg,0,sizeof(fg));imu(&a,110);
 a.armed=1;a.mode=MODE_AUTO;a.mission.state=MS_CLIMB;a.mission.profile=PROFILE_BALANCE_RETURN;
 a.pose.initialized=a.pose.valid=1;a.direction_known[0]=a.direction_known[1]=1;
 a.have_control=1;a.last_control_ms=100;a.speed[1].initialized=1;a.speed[1].previous_us=100000;
 fg[1].total=50;fg[1].have_edge=fg[1].have_period=1;fg[1].last_us=105000;fg[1].period_us=5000;
 App_Step(&a,fg,110,110000);
 CHECK(a.fault==0&&a.armed&&(a.warnings&1U));NEAR(a.pose.x,0,.000001);
 /* Clock and count wrapping are unsigned; 3 pulses in 10ms are plausible. */
 App_Init(&a);memset(fg,0,sizeof(fg));imu(&a,2);
 a.armed=1;a.mode=MODE_AUTO;a.mission.state=MS_CLIMB;a.mission.profile=PROFILE_BALANCE_RETURN;
 a.pose.initialized=a.pose.valid=1;a.pose.last_count[1]=0xfffffffeU;
 a.direction_known[0]=a.direction_known[1]=1;a.have_control=1;a.last_control_ms=0xfffffff8U;
 fg[1].total=1;fg[1].have_edge=fg[1].have_period=1;fg[1].last_us=2000;fg[1].period_us=5000;
 App_Step(&a,fg,2,2000);CHECK(a.fault==FAULT_NONE&&a.armed);
 DONE();}
