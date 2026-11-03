#include "test.h"
int main(void){
 RobotConfig c; Mission m={0}; MissionInput in={0};
 Config_Default(&c);Config_ApplyMeasured(&c);
 CHECK(c.yaw_tol_rad < .01746f && c.yaw_tol_rad > .01744f);
 in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 in.yaw_rad=.02f;in.now_ms=10;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.now_ms=1500;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_ALIGN);
 in.yaw_rad=.01f;in.now_ms=1520;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.yaw_rad=.02f;in.now_ms=2000;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.yaw_rad=.01f;in.now_ms=2020;in.imu_seq++;Mission_Update(&m,&in,&c);
 in.now_ms=3019;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_ALIGN);
 in.now_ms=3020;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB);
 in.yaw_rad=.02f;in.now_ms=3040;in.imu_seq++;Mission_Update(&m,&in,&c);
 NEAR(m.w,-.06f,.0001f);
 DONE();
}
