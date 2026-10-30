#include "test.h"
#include <string.h>
int main(void){
 CourseApp a; FgCapture fg[2]={{0},{0}}; Mission m={0}; MissionInput in={0}; RobotConfig c; WireFrame f={0};
 Config_Default(&c);c.cal_flags=CAL_PATH;in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 in.now_ms=60001; Mission_Update(&m,&in,&c);CHECK(m.state==MS_ALIGN&&m.fault==0);
 App_Init(&a);a.config.cal_flags=CAL_PATH;a.imu.have_euler=1;a.imu.euler_ms=110;a.imu.euler_seq=1;
 a.armed=1;a.mode=MODE_AUTO;a.mission.state=MS_CLIMB;a.mission.profile=PROFILE_BALANCE_RETURN;
 a.pose.initialized=a.pose.valid=1;a.direction_known[0]=a.direction_known[1]=1;
 a.have_control=1;a.last_control_ms=100;a.speed[1].initialized=1;a.speed[1].previous_us=100000;
 fg[1].total=50;fg[1].have_edge=fg[1].have_period=1;fg[1].last_us=105000;fg[1].period_us=5000;
 App_Step(&a,fg,110,110000);CHECK(a.armed&&a.fault==0&&(a.warnings&1U));NEAR(a.pose.x,0,.000001);
 {uint8_t out[88];CHECK(App_Telemetry(&a,110,out,sizeof(out))==88);CHECK((Wire_GetU32(out+84)&0x101U)==0x100U);}
 a.imu.euler_ms=110;App_Step(&a,fg,400,400000);CHECK(a.armed&&a.fault==0&&a.pwm[0]==0&&a.pwm[1]==0);
 a.imu.euler_ms=410;a.imu.euler_seq++;fg[1].have_edge=0;App_Step(&a,fg,410,410000);
 CHECK(a.armed&&a.fault==0&&a.mode==MODE_AUTO&&(a.warnings&4U));
 CHECK(a.pwm[0]>0||a.pwm[1]>0);
 f.type=CMD_STOP;CHECK(App_Command(&a,&f,401)==RESULT_OK&&!a.armed);
 DONE();
}
