#include "test.h"
int main(void){RobotConfig c;Mission m={0};MissionInput in={0};BalanceController b;unsigned i;float v;
 Config_Default(&c);Config_ApplyMeasured(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(c.balance_ki>.0003f&&c.balance_kd>.012f);
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_CLIMB;in.pose.x=.1f;
 in.pitch_deg=4.5f;in.now_ms=20;in.imu_seq=1;Mission_Update(&m,&in,&c);
 for(i=0;i<30;i++){in.now_ms+=20;in.imu_seq++;in.pitch_deg=3.8f;Mission_Update(&m,&in,&c);}
 CHECK(m.state==MS_CLIMB&&m.v>0);
 in.pose.x=.5f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.state==MS_BALANCE);
 Balance_Reset(&b);Balance_Update(&b,4.0f,1,20,1,&c);v=Balance_Update(&b,3.5f,2,40,1,&c);CHECK(v<0);
 memset(&m,0,sizeof(m));in.now_ms=0;in.imu_seq=0;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 in.pose.x=.6f;in.pitch_deg=4;in.imu_seq=1;in.now_ms=20;Mission_Update(&m,&in,&c);
 m.midpoint.initialized=1;m.midpoint.phase=1;m.midpoint.target_x=.8f;
 in.pitch_deg=3.5f;in.imu_seq=2;in.now_ms=40;Mission_Update(&m,&in,&c);CHECK(m.v<0);
 {CourseApp a;uint8_t out[88];App_Init(&a);a.mission.midpoint.phase=2;
 CHECK(App_Telemetry(&a,0,out,sizeof(out))==88);CHECK(((Wire_GetU32(out+84)>>17)&3U)==2U);}
 DONE();}
