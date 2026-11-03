#include "course.h"
#include <math.h>
#include <string.h>
void Balance_Reset(BalanceController *b){memset(b,0,sizeof(*b));}
static float update(BalanceController *b,float pitch,uint32_t seq,uint32_t now,int valid,const RobotConfig *c,uint32_t hold_ms,int bench)
{
 float dt=.02f,rate=0,next,raw,limit,angle,fraction;
 if(!valid||!Course_Finite(pitch)||fabsf(pitch)>c->max_tilt_deg){Balance_Reset(b);return 0;}
 if(b->initialized&&seq==b->last_seq)return b->v;
 if(b->initialized){
  dt=(float)(uint32_t)(now-b->last_ms)*.001f;
  if(dt<.005f||dt>.25f){Balance_Reset(b);return 0;}
  rate=.35f*(pitch-b->last_pitch)/dt+.65f*b->rate;
  if(pitch*b->last_pitch<0)b->integral=0;
 }
 b->initialized=1;b->last_seq=seq;b->last_ms=now;b->last_pitch=pitch;b->rate=rate;
 limit=c->max_v;
 if(!bench){
  angle=fabsf(pitch);
  /* Reduce the cap earlier only when the measured slope is toward level. */
  if(pitch*rate<0)angle=fmaxf(0,angle-fabsf(rate)*BALANCE_LOOKAHEAD_S);
  fraction=Course_Clamp((angle-c->stable_deg)/(BALANCE_SLOW_ANGLE_DEG-c->stable_deg),0,1);
  limit=fminf(c->max_v,BALANCE_SPEED_NEAR+(BALANCE_SPEED_FAR-BALANCE_SPEED_NEAR)*fraction);
 }
 b->speed_limit=limit;
 if(fabsf(pitch)<=c->stable_deg){
  b->v=b->integral=0;
  if(fabsf(rate)<=c->stable_rate_dps){
   if(!b->stable_tracking){b->stable_tracking=1;b->stable_since_ms=now;}
   b->stable=(uint32_t)(now-b->stable_since_ms)>=hold_ms;
  }else b->stable=b->stable_tracking=0;
  return 0;
 }
 b->stable=b->stable_tracking=0;
 next=Course_Clamp(b->integral+c->balance_ki*pitch*dt,-limit,limit);
 raw=c->balance_kp*pitch+c->balance_kd*rate+next;
 if(fabsf(raw)<=limit||raw*pitch<0)b->integral=next;
 b->integral=Course_Clamp(b->integral,-limit,limit);
 raw=Course_Clamp(c->balance_kp*pitch+c->balance_kd*rate+b->integral,-limit,limit);
 if(bench&&raw*pitch<=0)raw=0;
 b->v=c->balance_sign*raw;return b->v;
}
float Balance_Update(BalanceController *b,float p,uint32_t seq,uint32_t now,int valid,const RobotConfig *c){return update(b,p,seq,now,valid,c,10000,0);}
float Task1_Update(BalanceController *b,float p,uint32_t seq,uint32_t now,int valid,const RobotConfig *c){return update(b,p,seq,now,valid,c,1000,1);}
