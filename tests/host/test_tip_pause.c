#include "test.h"
int main(void){Mission m={0};MissionInput in={0};RobotConfig c;unsigned i;
 Config_Default(&c);Config_ApplyMeasured(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_CLIMB;in.pose.x=.1f;
 for(i=0;i<30;i++){in.now_ms=20*(i+1);in.imu_seq++;in.pitch_deg=i<10?4.5f:3.8f;Mission_Update(&m,&in,&c);CHECK(m.state==MS_CLIMB&&m.v>0);}
 in.pose.x=.5f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_BALANCE);
 in.pitch_deg=4.0f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v>0);
 in.pitch_deg=3.5f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v<0);
 CHECK(m.fault==0);DONE();}
