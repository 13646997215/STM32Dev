#include "test.h"
#include <math.h>
int main(void){
 RobotConfig c;Mission m={0};MissionInput in={0};float forward;
 Config_Default(&c);Config_ApplyMeasured(&c);
 in.imu_valid=in.pose_valid=in.pose.valid=in.center_feedback_valid=1;in.position_epoch=7;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 m.state=MS_CLIMB;in.pitch_deg=4;in.pose.x=.2f;in.imu_seq=1;in.now_ms=20;
 in.pose.y=.03f;Mission_Update(&m,&in,&c);forward=m.w;CHECK(forward<-.001f);
 in.pose.y=-.03f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.w>.001f);
 in.pose.y=.03f;m.state=MS_RETURN;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.w>.001f);
 in.pose.y=.03f;in.position_epoch=8;m.state=MS_CLIMB;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);NEAR(m.w,0,.0001f);
 in.position_epoch=7;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);NEAR(m.w,0,.0001f);
 in.pose.y=0;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_CLIMB;
 in.pose.y=.002f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);NEAR(m.w,0,.0001f);
 in.pose.y=.30f;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);NEAR(m.w,-CENTER_MAX_W,.0001f);
 in.center_feedback_valid=0;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);CHECK(m.center_invalid);NEAR(m.w,0,.0001f);
 in.center_feedback_valid=1;in.now_ms+=20;in.imu_seq++;Mission_Update(&m,&in,&c);NEAR(m.w,0,.0001f);
 {CourseApp a;uint8_t out[88];App_Init(&a);a.mission.center_started=1;
  CHECK(App_Telemetry(&a,0,out,sizeof(out))==88);CHECK((out[86]&8)!=0);
  a.mission.center_invalid=1;App_Telemetry(&a,0,out,sizeof(out));CHECK((out[86]&16)!=0);}
 {Pose p={0};FgCapture fg[2]={{0}};int8_t direction[2]={1,1};uint8_t known[2]={1,1};
  Pose_Reset(&p,0,0,.2f,fg);fg[0].total=fg[1].total=738;
  Pose_Update(&p,fg,direction,known,.2f,&c);CHECK(p.valid&&p.y>0);
  in.pose.y=0;in.position_epoch=9;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
  m.state=MS_CLIMB;in.pose=p;in.yaw_rad=0;in.imu_seq++;in.now_ms+=20;Mission_Update(&m,&in,&c);
  CHECK(m.w<0);}
 DONE();
}
