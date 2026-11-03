#include "course.h"
#include <math.h>
#include <string.h>
static void set_state(Mission *m,uint8_t s,uint32_t now){m->previous_state=m->state;m->state=s;m->state_ms=now;m->v=m->w=0;}
int Mission_Start(Mission *m,uint8_t profile,const MissionInput *in,const RobotConfig *c)
{
 if(profile<PROFILE_A||profile>PROFILE_REWEIGHT||!Config_Valid(c)||!in->imu_valid||!in->pose_valid||(c->cal_flags&CAL_PATH)!=CAL_PATH)return 0;
 memset(m,0,sizeof(*m));m->profile=profile;m->started_ms=in->now_ms;m->initial_pitch=in->pitch_deg;
 m->origin_x=in->pose.x;m->align_ref=fabsf(in->yaw_rad);
 set_state(m,profile==PROFILE_A?MS_SEEK:MS_ALIGN,in->now_ms);return 1;
}
int Mission_TipDetected(Mission *m,float pitch,int ready,uint32_t now,const RobotConfig *c)
{
 if(ready&&Course_Finite(pitch)&&fabsf(pitch-m->initial_pitch)>=c->tip_delta_deg){
  if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}
  return (uint32_t)(now-m->trigger_ms)>=200U;
 }
 m->tip_tracking=0;return 0;
}
/* The robot is mounted so its front-up Pitch is positive.  Only examine new
   samples after the configured minimum approach distance; transient single
   samples cannot on their own confirm the beginning of a flip. */
static int first_flip(Mission *m,const MissionInput *in,const RobotConfig *c)
{
 unsigned i;int falling=0;
 if(in->pose.x<c->climb_end_m){m->flip_count=0;m->flip_next=0;m->flip_candidate=0;return 0;}
 if(m->flip_has_seq&&in->imu_seq==m->flip_seq)return 0;
 m->flip_has_seq=1;m->flip_seq=in->imu_seq;
 for(i=0;i<m->flip_count;i++){
  uint32_t age=(uint32_t)(in->now_ms-m->flip_time[i]);
  if(age<=FLIP_WINDOW_MS&&m->flip_pitch[i]>=FLIP_MIN_PITCH_DEG&&
     m->flip_pitch[i]-in->pitch_deg>=FLIP_DROP_DEG){falling=1;break;}
 }
 m->flip_pitch[m->flip_next]=in->pitch_deg;m->flip_time[m->flip_next]=in->now_ms;
 m->flip_next=(uint8_t)((m->flip_next+1U)%FLIP_HISTORY_SIZE);
 if(m->flip_count<FLIP_HISTORY_SIZE)m->flip_count++;
 if(!falling){m->flip_candidate=0;return 0;}
 if(!m->flip_candidate){m->flip_candidate=1;m->flip_candidate_ms=in->now_ms;return 0;}
 return (uint32_t)(in->now_ms-m->flip_candidate_ms)>=FLIP_CONFIRM_MS;
}
void Mission_Update(Mission *m,const MissionInput *in,const RobotConfig *c)
{
 float e,rate=0,u;uint32_t now=in->now_ms;
 m->v=m->w=0;
 if(m->state==MS_IDLE||m->state==MS_FAULT||m->state==MS_DONE||m->state==MS_ARRIVED)return;
 if(!in->imu_valid||!Course_Finite(in->pitch_deg)||!Course_Finite(in->yaw_rad)){m->fault=FAULT_IMU;set_state(m,MS_FAULT,now);return;}
 if(!in->pose_valid||!in->pose.valid||!Course_Finite(in->pose.x)){m->fault=FAULT_POSE;set_state(m,MS_FAULT,now);return;}
 switch(m->state){
 case MS_ALIGN:
  e=in->yaw_rad;
  if(m->align_prev_ms&&(uint32_t)(now-m->align_prev_ms)<250U&&now>m->align_prev_ms)
   rate=Course_Wrap(e-m->align_prev)/((float)(now-m->align_prev_ms)*.001f);
  m->align_prev=e;m->align_prev_ms=now;
  if(fabsf(e)>1.57f){if(!m->align_dir)m->align_dir=e>0?-1:1;}else m->align_dir=0;
  if(!m->align_sign&&(uint32_t)(now-m->state_ms)>=400U)m->align_sign=(m->align_ref>.4f&&fabsf(e)>m->align_ref+.5f)?-1:1;
  u=Course_Clamp(-(c->heading_kp*e+ALIGN_DAMP*rate),-c->max_w,c->max_w);
  if(m->align_dir)u=(float)m->align_dir*fabsf(u);
  if(m->align_sign<0)u=-u;
  m->w=u;
  if(fabsf(e)<=c->yaw_tol_rad){if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}else if((uint32_t)(now-m->trigger_ms)>=ALIGN_HOLD_MS){m->tip_tracking=0;set_state(m,MS_CLIMB,now);}}
  else m->tip_tracking=0;
  break;
 case MS_SEEK:
  m->v=c->max_v;
  if(Mission_TipDetected(m,in->pitch_deg,1,now,c))set_state(m,MS_DONE,now);
  break;
 case MS_CLIMB:
  m->v=c->max_v;m->w=Course_Clamp(-CLIMB_HEADING_GAIN_MULTIPLIER*c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
  if(m->profile==PROFILE_ENTRY&&in->pose.x>=c->climb_end_m){set_state(m,MS_SEEK,now);}
  else if(m->profile!=PROFILE_ENTRY&&first_flip(m,in,c)){
   Balance_Reset(&m->balance);m->balance_ms=now;set_state(m,MS_BALANCE,now);
   m->v=Balance_Update(&m->balance,in->pitch_deg,in->imu_seq,now,1,c);
   if(fabsf(in->pitch_deg)>c->stable_deg)m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
  }
  break;
 case MS_BALANCE:
  m->v=Balance_Update(&m->balance,in->pitch_deg,in->imu_seq,now,1,c);
  if(fabsf(in->pitch_deg)<=c->stable_deg)m->v=m->w=0;
  else m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
  if(m->balance.stable){m->balance_achieved=1;if(m->profile==PROFILE_REWEIGHT&&!m->weight_events)set_state(m,MS_HOLD,now);else {m->return_ms=now;set_state(m,MS_RETURN,now);}}
  break;
 case MS_HOLD:
  if(fabsf(in->pitch_deg)>=c->disturb_deg){m->weight_events++;Balance_Reset(&m->balance);set_state(m,MS_BALANCE,now);}
  break;
 case MS_RETURN:
  m->v=-c->max_v;m->w=Course_Clamp(-RETURN_HEADING_GAIN_MULTIPLIER*c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
  if(!m->ramp_seen){if(fabsf(in->pitch_deg)>3.0f){if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}else if((uint32_t)(now-m->trigger_ms)>=300U){m->ramp_seen=1;m->tip_tracking=0;}}else m->tip_tracking=0;}
  else if(fabsf(in->pitch_deg)<1.5f){if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}else if((uint32_t)(now-m->trigger_ms)>=1000U)set_state(m,MS_ARRIVED,now);}else m->tip_tracking=0;
  break;
 default:m->fault=FAULT_CONFIG;set_state(m,MS_FAULT,now);break;
 }
}
