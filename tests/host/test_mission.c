#include "test.h"
static void tick(Mission*m,MissionInput*i,RobotConfig*c,uint32_t ms,float x,float pitch){i->now_ms=ms;i->pose.x=x;i->pitch_deg=pitch;i->imu_seq++;Mission_Update(m,i,c);}
int main(void){RobotConfig c;Mission m={0};MissionInput in={0};unsigned i;
 Config_Default(&c);in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(!Mission_Start(&m,PROFILE_A,&in,&c));c.cal_flags=255;CHECK(Mission_Start(&m,PROFILE_A,&in,&c));CHECK(m.state==MS_BALANCE);
 tick(&m,&in,&c,20,.4f,5);CHECK(m.v>0&&m.w==0&&m.state==MS_BALANCE);
 in.yaw_rad=in.pose.yaw=2;tick(&m,&in,&c,40,.4f,5);CHECK(m.v>0&&m.w==0&&m.fault==0);
 tick(&m,&in,&c,60,.4f,-5);CHECK(m.v<0&&m.w==0);
 for(i=0;i<=60;i++)tick(&m,&in,&c,80+20*i,.4f,0);
 CHECK(m.state==MS_DONE&&m.v==0&&m.w==0);
 /* Task 1 is plain Pitch leveling: no longitudinal corridor is enforced in
    either direction. The course 60 s session limit is the only time bound. */
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_A,&in,&c));tick(&m,&in,&c,20,-.60f,-5);
 CHECK(m.fault==0&&m.v<0&&m.state==MS_BALANCE);
 tick(&m,&in,&c,40,-1.50f,-5);CHECK(m.fault==0&&m.v<0);
 in.now_ms=0;in.pose.x=0;CHECK(Mission_Start(&m,PROFILE_A,&in,&c));
 tick(&m,&in,&c,20,c.seek_max_m+.50f,5);CHECK(m.fault==0&&m.v>0);
 tick(&m,&in,&c,40,3.00f,5);CHECK(m.fault==0&&m.v>0);
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;CHECK(Mission_Start(&m,PROFILE_A,&in,&c));tick(&m,&in,&c,60001,.1f,0);CHECK(m.state==MS_FAULT&&m.fault==FAULT_TIMEOUT);
 /* Unified flow: turn onto the board heading (yaw == 0), hold the band 500 ms,
    then drive the configured climb distance and hand over to the balancer. */
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;
 {RobotConfig d=c;d.cal_flags=CAL_PATH;d.yaw_tol_rad=.035f;d.climb_end_m=.5f;d.max_v=.06f;
  CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&d));CHECK(m.state==MS_ALIGN);
  in.yaw_rad=.5f;tick(&m,&in,&d,20,0,0);CHECK(m.state==MS_ALIGN&&m.v==0&&m.w<0);   /* turn right */
  in.yaw_rad=-.5f;tick(&m,&in,&d,40,0,0);CHECK(m.state==MS_ALIGN&&m.w>0);           /* turn left */
  in.yaw_rad=.01f;tick(&m,&in,&d,60,0,0);CHECK(m.state==MS_ALIGN);
  tick(&m,&in,&d,700,0,0);CHECK(m.state==MS_CLIMB);                                  /* 640 ms in band */
  for(i=0;i<200;i++){in.now_ms=710+i*20;in.pose.x=.0025f*(i+1);tick(&m,&in,&d,in.now_ms,in.pose.x,0);
    if(m.state!=MS_CLIMB)break;}
  CHECK(m.state==MS_BALANCE);
  CHECK(in.pose.x>=d.climb_end_m-.06f);}
 /* A wrong yaw sign must never send the car the long way round: the 400 ms probe
    notices that the error grew and inverts the drive for the rest of the turn. */
 {RobotConfig e=c;e.cal_flags=CAL_PATH;memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;in.yaw_rad=1.2f;
  CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&e));CHECK(m.state==MS_ALIGN&&m.align_sign==0);
  tick(&m,&in,&e,20,0,0);CHECK(m.w<0);
  in.yaw_rad=1.9f;tick(&m,&in,&e,450,0,0);CHECK(m.align_sign<0);   /* error grew by >0.5 rad */
  tick(&m,&in,&e,460,0,0);CHECK(m.w>0);
  CHECK(m.align_dir<0);                                            /* committed past 90 deg */
  in.yaw_rad=1.2f;tick(&m,&in,&e,1000,0,0);CHECK(m.align_dir==0);  /* back under 90 deg: free */
  CHECK(m.ramp_seen==0);                                           /* the retreat latch is untouched */}
 /* Crossing the +/- pi representation is a tiny yaw change, not a full turn. */
 {RobotConfig e=c;e.cal_flags=CAL_PATH;e.heading_kp=.1f;e.max_w=4.0f;
  memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;in.yaw_rad=3.14f;
  CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&e));tick(&m,&in,&e,20,0,0);
  in.yaw_rad=-3.14f;tick(&m,&in,&e,40,0,0);CHECK(fabsf(m.w)<1.0f);}
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_BALANCE;
 /* 8 s flat is not enough: the 10 s balance timer must still be running. */
 for(i=0;i<=400;i++)tick(&m,&in,&c,20*i,.7f,0);
 CHECK(m.state==MS_BALANCE&&!m.balance.stable);
 /* A single disturbance restarts the 10 s count instead of finishing the task. */
 tick(&m,&in,&c,8100,.7f,5);
 CHECK(m.state==MS_BALANCE&&!m.balance.stable&&!m.balance_achieved);
 for(i=0;i<=520;i++)tick(&m,&in,&c,8200+20*i,.7f,0);
 CHECK(m.state==MS_RETURN&&m.balance_achieved);
 /* Retreat: latch the tilted deck first, then finish once the pitch is flat. */
 tick(&m,&in,&c,19000,.7f,0.5f);CHECK(m.state==MS_RETURN&&m.v<0&&!m.ramp_seen);
 for(i=0;i<=18;i++)tick(&m,&in,&c,19020+20*i,.7f,5);
 CHECK(m.state==MS_RETURN&&m.ramp_seen);
 for(i=0;i<=60;i++)tick(&m,&in,&c,20000+20*i,.7f,0.5f);
 CHECK(m.state==MS_ARRIVED&&m.v==0);
 /* Distance fallback stops the retreat but cannot prove that the car arrived. */
 memset(&in,0,sizeof(in));in.imu_valid=in.pose_valid=in.pose.valid=1;
 CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_RETURN;m.return_ms=0;
 tick(&m,&in,&c,20,1.30f,0);CHECK(m.state==MS_RETURN&&m.fault==0&&(m.warnings&16U));
 in.now_ms=0;in.pose.x=0;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));m.state=MS_RETURN;m.return_ms=0;
 tick(&m,&in,&c,45001,.1f,0);CHECK(m.state==MS_RETURN&&m.fault==0);
 tick(&m,&in,&c,60001,.1f,0);CHECK(m.state==MS_RETURN&&m.fault==0&&(m.warnings&2U));
 /* Balancing is allocated its own 90 seconds after arrival on the board. */
 in.now_ms=0;in.pose.x=0;CHECK(Mission_Start(&m,PROFILE_BALANCE_RETURN,&in,&c));
 m.state=MS_BALANCE;m.state_ms=50000;m.balance_ms=50000;
 tick(&m,&in,&c,90001,.5f,5);CHECK(m.state==MS_BALANCE&&m.fault==0);
 tick(&m,&in,&c,140001,.5f,5);CHECK(m.state==MS_BALANCE&&m.fault==0&&(m.warnings&2U));
 in.now_ms=0;in.pose.x=0;CHECK(Mission_Start(&m,PROFILE_REWEIGHT,&in,&c));m.state=MS_BALANCE;
 for(i=0;i<=501;i++)tick(&m,&in,&c,20*i,.7f,0);CHECK(m.state==MS_HOLD);
 tick(&m,&in,&c,10060,.7f,3);CHECK(m.state==MS_BALANCE&&m.weight_events==1);
 for(i=0;i<=520;i++)tick(&m,&in,&c,10080+20*i,.7f,0);
 CHECK(m.state==MS_RETURN&&m.weight_events==1&&m.fault==0);
 in.now_ms=0;in.pitch_deg=0;in.pose.x=0;CHECK(Mission_Start(&m,PROFILE_PRELIM,&in,&c));m.state=MS_RETURN;m.return_ms=100;
 tick(&m,&in,&c,5101,.7f,0);CHECK(m.state==MS_FAULT&&m.fault==FAULT_TIMEOUT);
 in.now_ms=0;in.pose.x=in.pose.y=0;CHECK(Mission_Start(&m,PROFILE_RECTANGLE,&in,&c));CHECK(m.state==MS_PATH);
 {const float x[]={.5f,.5f,0,0,0,.5f,.5f,0};const float y[]={0,-.3f,-.3f,0,-.3f,-.3f,0,0};const float a[]={0,-COURSE_PI/2,-COURSE_PI,COURSE_PI/2,-COURSE_PI/2,0,COURSE_PI/2,-COURSE_PI};
  for(i=0;i<8;i++){in.pose.x=x[i];in.pose.y=y[i];in.pose.yaw=a[i];in.yaw_rad=a[i];tick(&m,&in,&c,100+i*40,x[i],0);CHECK(m.path_index==i+1);
   if(i==3){CHECK(m.state==MS_DOCK);in.pose.yaw=in.yaw_rad=0;tick(&m,&in,&c,240,0,0);CHECK(m.state==MS_PATH);}}}
 CHECK(m.state==MS_DOCK);in.pose.yaw=0;in.yaw_rad=0;tick(&m,&in,&c,500,0,0);CHECK(m.state==MS_DONE);
 in.now_ms=0;in.pose.x=in.pose.y=0;CHECK(Mission_Start(&m,PROFILE_A,&in,&c));in.imu_valid=0;tick(&m,&in,&c,10,.1f,0);CHECK(m.state==MS_FAULT&&m.fault==FAULT_IMU);
 DONE();}
