#include "test.h"
int main(void){RobotConfig c;Mission m={0};MissionInput in={0};unsigned i;
 Config_Default(&c);Config_ApplyMeasured(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 m.midpoint.initialized=1;m.midpoint.phase=1;m.midpoint.target_x=.9f;
 in.pose.x=.7f;in.pitch_deg=.5f;in.yaw_rad=.3f;in.imu_seq=1;in.now_ms=20;Mission_Update(&m,&in,&c);
 CHECK(m.v==0&&m.w==0);CHECK(m.balance.stable_tracking);
 memset(&m,0,sizeof(m));memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_CLIMB;in.pose.x=.1f;
 in.pitch_deg=4.5f;in.imu_seq=1;in.now_ms=20;Mission_Update(&m,&in,&c);
 in.pitch_deg=3.9f;in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);
 CHECK(m.state==MS_CLIMB&&m.v>0);
 in.pitch_deg=4.5f;for(i=0;i<30;i++){in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);}
 in.pitch_deg=3.9f;for(i=0;i<12;i++){in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);}
 CHECK(m.state==MS_BALANCE&&m.v==0&&m.w==0);
 {CourseApp a;FgCapture fg[2]={{0},{0}};
  App_Init(&a);Config_ApplyMeasured(&a.config);a.armed=1;a.mode=MODE_AUTO;
  a.mission.state=MS_BALANCE;a.mission.profile=PROFILE_BALANCE_RETURN;
  a.mission.midpoint.initialized=1;a.mission.midpoint.phase=1;a.mission.midpoint.target_x=.9f;
  a.pose.x=.7f;a.pose.valid=a.pose.initialized=1;a.imu.have_euler=1;a.imu.euler_seq=1;
  a.imu.euler_ms=100;a.imu.euler_deg[1]=.5f;a.imu.euler_deg[2]=0;
  a.have_control=1;a.last_control_ms=90;App_Step(&a,fg,100,100000);
  CHECK(a.pwm[0]==0&&a.pwm[1]==0);CHECK(a.mission.v==0&&a.mission.w==0);
 }
 DONE();}
