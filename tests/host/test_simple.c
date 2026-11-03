#include "test.h"
int main(void){RobotConfig c;Mission m={0};MissionInput in={0};unsigned i;float v;
 Config_Default(&c);Config_ApplyMeasured(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(c.max_v>.15f);CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 in.yaw_rad=.01f;in.now_ms=20;in.imu_seq=1;Mission_Update(&m,&in,&c);
 in.now_ms=1100;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB);
 in.pose.x=.1f;in.pitch_deg=4;in.now_ms=1120;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v>.1f);
 in.pose.x=.5f;in.now_ms=1140;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB);
 in.now_ms=1160;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB);
 in.pitch_deg=1.8f;in.now_ms=1180;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.now_ms=1220;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_BALANCE);
 in.pitch_deg=4;in.now_ms=1240;in.imu_seq++;Mission_Update(&m,&in,&c);v=m.v;CHECK(v>0);
 m.midpoint.initialized=1;m.midpoint.phase=1;m.midpoint.target_x=-1;
 in.now_ms=1260;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v>0);
 in.pitch_deg=0;for(i=0;i<=540;i++){in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);}
 CHECK(m.state==MS_RETURN);CHECK(m.balance_achieved);

 /* Task 1 stops after a sustained tilt; Task 2 adds alignment and approach. */
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_A,&in,&c));CHECK(m.state==MS_SEEK);
 in.pitch_deg=4;in.now_ms=20;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.now_ms=240;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_DONE);
 in.pitch_deg=0;in.now_ms=0;CHECK(Mission_Start(&m,PROFILE_ENTRY,&in,&c));CHECK(m.state==MS_ALIGN);
 in.now_ms=20;in.imu_seq++;Mission_Update(&m,&in,&c);in.now_ms=1100;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB);
 in.pose.x=.5f;in.now_ms=1120;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_SEEK);
 in.pitch_deg=4;in.now_ms=1140;in.imu_seq++;Mission_Update(&m,&in,&c);in.now_ms=1360;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_DONE);
 /* Task 6 waits stationary after first balance, then rebalances on disturbance. */
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;CHECK(Mission_Start(&m,PROFILE_REWEIGHT,&in,&c));m.state=MS_BALANCE;
 for(i=0;i<=510;i++){in.now_ms=20*(i+1);in.imu_seq++;Mission_Update(&m,&in,&c);}CHECK(m.state==MS_HOLD&&m.v==0&&m.w==0);
 in.pitch_deg=3;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_BALANCE&&m.weight_events==1);
 in.pitch_deg=0;for(i=0;i<=530;i++){in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);}CHECK(m.state==MS_RETURN);
 /* GUI launch and PA0 both use distance relative to the start position. */
 {CourseApp a;WireFrame f={0};App_Init(&a);Config_ApplyMeasured(&a.config);a.armed=1;
  a.imu.have_euler=1;a.imu.euler_seq=1;a.imu.euler_ms=0;a.imu.euler_deg[1]=2;a.scene_pitch_deg=2;
  f.type=CMD_START;f.length=1;f.payload[0]=PROFILE_BALANCE_RETURN;f.sequence=77;
  CHECK(App_Command(&a,&f,0)==RESULT_NOT_READY);a.scene_valid=1;
  CHECK(App_Command(&a,&f,0)==RESULT_OK);NEAR(a.pose.x,0,1e-6);NEAR(a.pose.y,0,1e-6);NEAR(a.config.pitch_zero_deg,2,1e-6);
 }
 /* STOP remains available in automatic mode. */
 {CourseApp a;WireFrame f={0};App_Init(&a);a.armed=1;a.mode=MODE_AUTO;a.pwm[0]=a.pwm[1]=.6f;f.type=CMD_STOP;CHECK(App_Command(&a,&f,0)==RESULT_OK);CHECK(!a.armed&&a.pwm[0]==0&&a.pwm[1]==0);}
 DONE();}
