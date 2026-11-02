#include "test.h"
int main(void){Mission m={0};MissionInput in={0};RobotConfig c;unsigned i;
 Config_Default(&c);Config_ApplyMeasured(&c);c.balance_kd=0;in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;m.balance_ms=0;
 /* Three successive commanded directions create two reversal coordinates. */
 in.pose.x=.60f;in.pitch_deg=2;in.imu_seq=1;in.now_ms=20;Mission_Update(&m,&in,&c);CHECK(m.v>0);
 in.pose.x=.80f;in.pitch_deg=-2;in.imu_seq=2;in.now_ms=40;Mission_Update(&m,&in,&c);CHECK(m.v<0);
 in.pose.x=.60f;in.pitch_deg=2;in.imu_seq=3;in.now_ms=60;Mission_Update(&m,&in,&c);
 /* The midpoint .70 is ahead: bounded crawl to it, independent of PID speed. */
 CHECK(m.v>0);CHECK(fabsf(m.v)<=.01501f);
 in.pose.x=.70f;in.pitch_deg=2;in.imu_seq=4;in.now_ms=80;Mission_Update(&m,&in,&c);CHECK(fabsf(m.v)<=.01501f);
 CHECK(m.midpoint.phase==2);
 /* A rejected FG interval invalidates the midpoint, without a fault stop. */
 in.position_epoch++;in.pitch_deg=2;in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);
 CHECK(m.midpoint.phase==0&&!m.midpoint.first_valid&&m.fault==0);
 /* An implausibly short span cannot become a midpoint. */
 memset(&m,0,sizeof(m));in.position_epoch=0;in.now_ms=0;in.imu_seq=0;in.pose.x=.6f;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 in.pitch_deg=2;in.now_ms=20;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.pose.x=.8f;in.pitch_deg=-2;in.now_ms=40;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.pose.x=.795f;in.pitch_deg=2;in.now_ms=60;in.imu_seq++;Mission_Update(&m,&in,&c);
 CHECK(m.midpoint.phase==0);
 for(i=0;i<5;i++){in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);CHECK(m.fault==0);}
 /* End-to-end: an FG burst in App_Step must reset an active target. */
 {CourseApp a;FgCapture fg[2]={{0},{0}};
  App_Init(&a);Config_ApplyMeasured(&a.config);a.armed=1;a.mode=MODE_AUTO;
  a.mission.state=MS_BALANCE;a.mission.profile=PROFILE_BALANCE_RETURN;
  a.mission.midpoint.initialized=1;a.mission.midpoint.phase=1;a.mission.midpoint.target_x=.3f;
  a.imu.have_euler=1;a.imu.euler_seq=1;a.imu.euler_ms=110;a.imu.euler_deg[1]=4;
  a.pose.initialized=a.pose.valid=1;a.direction_known[0]=a.direction_known[1]=1;
  a.have_control=1;a.last_control_ms=100;fg[1].total=100;
  App_Step(&a,fg,110,110000);
  CHECK(a.position_epoch>0);CHECK(a.mission.midpoint.position_epoch==a.position_epoch);
  CHECK(a.mission.midpoint.phase==0);CHECK(a.fault==0);
 }
 DONE();}
