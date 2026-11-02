#include "test.h"
int main(void){Mission m={0};MissionInput in={0};RobotConfig c;unsigned i;
 Config_Default(&c);Config_ApplyMeasured(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 in.pose.x=.7f;in.yaw_rad=.08f;in.pitch_deg=4.5f;in.imu_seq=1;in.now_ms=20;Mission_Update(&m,&in,&c);CHECK(m.v>0);
 in.pitch_deg=3.9f;in.imu_seq++;in.now_ms=40;Mission_Update(&m,&in,&c);CHECK(m.v==0&&m.w==0);
 for(i=0;i<10;i++){in.pitch_deg-=.10f;in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);CHECK(m.v==0&&m.w==0);}
 for(i=0;i<40;i++){in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);}
 CHECK(m.v!=0);CHECK(m.fault==0);
 memset(&m,0,sizeof(m));in.now_ms=0;in.imu_seq=0;in.pose.x=.1f;in.pitch_deg=0;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_CLIMB;
 in.pitch_deg=4.5f;in.now_ms=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v>0);
 for(i=0;i<20;i++){in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);}
 in.pitch_deg=3.9f;for(i=0;i<10;i++){in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);}
 CHECK(m.state==MS_BALANCE&&m.v==0&&m.w==0);
 /* Either end tipping must trigger the same hold. */
 memset(&m,0,sizeof(m));in.now_ms=0;in.imu_seq=0;in.pitch_deg=0;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 in.pitch_deg=-4.5f;in.now_ms=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v<0);
 in.pitch_deg=-3.9f;in.now_ms=40;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.v==0&&m.w==0);
 DONE();}
