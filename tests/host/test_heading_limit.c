#include "test.h"
#include <math.h>
int main(void)
{
 RobotConfig c; Mission m = {0}; MissionInput in = {0}; float wheels[2];
 Config_Default(&c); Config_ApplyMeasured(&c);
 in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 m.state=MS_CLIMB; in.yaw_rad=1.5f;in.pitch_deg=4;in.imu_seq=1;in.now_ms=10;
 Mission_Update(&m,&in,&c);
 Course_Mix(m.v,m.w,c.track_m,c.max_v,wheels);
 CHECK(m.v>0);
 NEAR(m.w,-c.max_w,.0001f);
 in.yaw_rad=.02f;in.now_ms=20;in.imu_seq++;
 Mission_Update(&m,&in,&c);
 NEAR(m.w,-c.heading_kp*.02f,.0001f);
 in.yaw_rad=-.02f;in.now_ms=25;in.imu_seq++;
 Mission_Update(&m,&in,&c);
 NEAR(m.w,c.heading_kp*.02f,.0001f);
 in.yaw_rad=1.5f;
 m.state=MS_BALANCE;Balance_Reset(&m.balance);
 in.yaw_rad=1.5f;in.pitch_deg=3;in.now_ms=30;in.imu_seq++;
 Mission_Update(&m,&in,&c);
 NEAR(m.w,-c.max_w,.0001f);
 m.state=MS_RETURN; in.now_ms=50;in.imu_seq++;
 Mission_Update(&m,&in,&c);
 Course_Mix(m.v,m.w,c.track_m,c.max_v,wheels);
 CHECK(m.v<0);
 NEAR(m.w,-c.max_w,.0001f);
 DONE();
}
