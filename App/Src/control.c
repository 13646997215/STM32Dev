#include "course.h"
#include <math.h>
#include <string.h>

void Fg_Edge(FgCapture *f,uint32_t now)
{
    uint32_t delta=now-f->last_us;
    if(f->have_edge&&delta<100U){f->glitches++;return;}
    if(f->have_edge){f->period_us=delta;f->have_period=1;}
    f->last_us=now;f->have_edge=1;f->total++;
}
void Speed_Update(SpeedEstimate *s,const FgCapture *f,uint32_t now,uint32_t timeout)
{
    uint32_t elapsed,count;
    if(!s->initialized){s->previous_total=f->total;s->previous_us=now;s->initialized=1;}
    if(!f->have_edge||(uint32_t)(now-f->last_us)>timeout){s->hz=0;s->fresh=0;s->previous_total=f->total;s->previous_us=now;return;}
    s->fresh=1;elapsed=now-s->previous_us;count=f->total-s->previous_total;
    if(elapsed>=20000U&&count>=4U)s->hz=(float)count*1000000.0f/(float)elapsed;
    else if(f->have_period&&f->period_us>=100U)s->hz=1000000.0f/(float)f->period_us;
    else s->hz=0;
    if(elapsed>=20000U){s->previous_total=f->total;s->previous_us=now;}
}
float Pi_Update(PiController *p,float error,float dt,float kp,float ki,float ff,float limit)
{
    float integral,raw;
    if(!Course_Finite(error)||!Course_Finite(dt)||!Course_Finite(kp)||!Course_Finite(ki)||
       !Course_Finite(ff)||!Course_Finite(limit)||dt<.001f||dt>.05f||kp<0||ki<0||limit<=0){p->integral=0;return 0;}
    integral=Course_Clamp(p->integral+ki*error*dt,-limit,limit);raw=ff+kp*error+integral;
    if((raw>limit&&error>0)||(raw<0&&error<0))raw=ff+kp*error+p->integral;
    else p->integral=integral;
    return Course_Clamp(raw,0,limit);
}
void Motor_Init(MotorController *m){memset(m,0,sizeof(*m));m->direction=1;}
static void motor_fault(MotorController *m){m->phase=MOTOR_FAULT;m->duty=0;m->direction_assumed=0;m->pi.integral=0;}
void Motor_Update(MotorController *m,float target,float measured,const FgCapture *fg,
                  uint32_t now,uint32_t us,float dt,int enabled,const RobotConfig *c,unsigned i)
{
    int8_t requested;uint8_t quiet;
    if(!enabled){m->phase=MOTOR_DISABLED;m->ramp_rpm=m->duty=m->pi.integral=0;m->direction_assumed=0;return;}
    if(m->phase==MOTOR_FAULT)return;
    if(i>1||!Course_Finite(target)||!Course_Finite(measured)||measured<0||!Course_Finite(dt)||dt<.001f||dt>.05f){motor_fault(m);return;}
    target=Course_Clamp(target,-c->max_rpm,c->max_rpm);
    if(fabsf(target)<.001f){m->duty=m->ramp_rpm=m->pi.integral=0;m->direction_assumed=0;m->phase=MOTOR_READY;return;}
    requested=target>0?1:-1;
    if(requested!=m->direction&&m->phase!=MOTOR_WAIT){m->phase=MOTOR_WAIT;m->phase_ms=now;m->duty=m->ramp_rpm=m->pi.integral=0;m->direction_assumed=0;}
    if(m->phase==MOTOR_WAIT){
        /* Blank the output for reversal_quiet_ms, then allow the new direction
           once the wheel is effectively stopped. "Effectively stopped" has to
           accept a low measured speed, not only an old edge timestamp: during an
           in-place turn the other wheel's vibration keeps producing stray edges
           on this wheel's FG line, so the timestamp-only test never went true and
           one wheel sat at zero duty for the whole alignment (then raised a false
           MOTOR fault). A genuinely spinning wheel still blocks the reversal. */
        m->duty=0;quiet=!fg->have_edge||(uint32_t)(us-fg->last_us)>=c->reversal_quiet_ms*1000U||measured<2.0f;
        if((uint32_t)(now-m->phase_ms)>=c->reversal_quiet_ms&&quiet){m->direction=requested;m->phase=MOTOR_READY;return;}
        if((uint32_t)(now-m->phase_ms)>2500U)motor_fault(m);
        return;
    }
    if(m->phase!=MOTOR_RUN){m->phase=MOTOR_RUN;m->run_ms=now;m->ramp_rpm=0;m->pi.integral=0;}
    if(c->fg_fault_en&&(uint32_t)(now-m->run_ms)>1500U&&(!fg->have_edge||(uint32_t)(us-fg->last_us)>c->fg_timeout_ms*1000U)){motor_fault(m);return;}
    m->ramp_rpm+=Course_Clamp(fabsf(target)-m->ramp_rpm,-c->rpm_slew*dt,c->rpm_slew*dt);
    m->duty=Pi_Update(&m->pi,m->ramp_rpm-measured,dt,c->kp[i],c->ki[i],c->deadband[i],c->max_duty);
    /* A loaded wheel simply will not break away below its stiction duty, so any
       running wheel is floored here. Zero still means stop: this only lifts a
       non-zero command that the PI would otherwise have throttled into the dead
       band, where the motor buzzes instead of turning. */
    if(m->duty>0&&m->duty<c->min_duty)m->duty=c->min_duty;
    /* Assumed commanded direction, NOT an independently measured FG direction. */
    m->direction_assumed=1;
}
