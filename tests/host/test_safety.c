#include "test.h"
#include <string.h>
int main(void){CourseApp a;RobotConfig c;WireFrame f={0};FgCapture fg[2]={{0},{0}};Pose previous;MotionOutput motion;int8_t dir[2]={1,1};uint8_t trusted[2]={1,1};
 Config_Default(&c);c.cal_flags=255;CHECK(Config_Set(&c,1,.068f));CHECK(c.cal_flags==0);
 App_Init(&a);a.config.cal_flags=255;a.armed=1;a.mode=MODE_MANUAL_SPEED;a.manual[0]=10;
 a.imu.have_euler=1;a.imu.euler_ms=0;a.imu.euler_seq=1;f.type=CMD_START;f.length=1;f.payload[0]=PROFILE_A;
 CHECK(App_Command(&a,&f,0)==RESULT_LOCKED&&a.mode==MODE_MANUAL_SPEED);
 App_Init(&a);a.armed=1;a.pose.x=.6f;a.pose.y=.02f;previous=a.pose;f.payload[0]=99;
 CHECK(App_Command(&a,&f,0)==RESULT_RANGE);CHECK(a.pose.x==previous.x&&a.pose.y==previous.y);
 App_Init(&a);a.armed=1;a.config.cal_flags=CAL_BOARD;a.motor[0].direction=-1;a.direction[0]=-1;
 App_Stop(&a,FAULT_IMU);CHECK(a.motor[0].direction==-1&&!a.armed&&a.fault==FAULT_IMU);
 CHECK(a.mission.state==MS_FAULT&&a.mission.fault==FAULT_IMU);
 App_Step(&a,fg,0,0);CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 f.type=CMD_ARM;f.length=2;f.payload[0]=0xDE;f.payload[1]=0xC0;CHECK(App_Command(&a,&f,0)==RESULT_FAULT);
 Config_Default(&c);Pose_Reset(&previous,0,0,NAN,fg);motion=Motion_Goto(&previous,1,0,0,0,&c);CHECK(motion.v==0&&motion.w==0&&!motion.reached);
 Pose_Reset(&previous,0,0,0,fg);Pose_Update(&previous,fg,dir,trusted,NAN,&c);CHECK(!previous.valid);
 App_Init(&a);a.mission.state=MS_HOLD;a.mission.balance.stable=1;App_Stop(&a,0);CHECK(a.mission.state==MS_IDLE&&!a.mission.balance.stable);
 DONE();}
