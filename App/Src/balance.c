#include "course.h"
#include <math.h>
#include <string.h>

void Balance_Reset(BalanceController *b){memset(b,0,sizeof(*b));}
float Task1_Update(BalanceController *b,float pitch,uint32_t seq,uint32_t now,int valid,const RobotConfig *c)
{
    /* The command ceiling is the configured max_v. A hard .04 m/s cap here used
       to pin the whole response flat above ~1.6 deg, so a bigger tilt produced
       no bigger command; the operator now sets the ceiling with key 20. */
    float dt=.02f,rate=0,limit=c->max_v,magnitude=fabsf(pitch),minmag,raw,integral,pterm,dterm;
    if(!valid||!Course_Finite(pitch)||magnitude>c->max_tilt_deg){Balance_Reset(b);return 0;}
    if(b->initialized&&seq==b->last_seq)return b->v;
    if(b->initialized){
        dt=(float)(uint32_t)(now-b->last_ms)*.001f;
        if(dt>=.005f&&dt<=.25f)rate=(pitch-b->last_pitch)/dt;
        else {Balance_Reset(b);}
    }
    b->initialized=1;b->last_seq=seq;b->last_ms=now;b->last_pitch=pitch;b->rate=rate;
    minmag=fminf(c->min_v,limit);
    /* Stop immediately inside tolerance; require twice that error to restart. */
    if(magnitude<=c->stable_deg||(b->v==0&&magnitude<=2*c->stable_deg)){
        b->v=0;b->integral=0;
    }else{
        /* PID on the pitch error, same structure as the course material:
           KP*e + KI*integral + KD*rate, integral clamped against wind-up and the
           output clamped to max_v. KI=KD=0 reproduces the previous pure-P map.
           The D term is limited to half of the P term: on this plant any usable
           KD becomes huge during a fast hand movement (one 20 ms frame can be
           -400 deg/s), and without that limit D would wipe out the command
           exactly when the car is returning to level. */
        pterm=c->balance_kp*pitch;
        dterm=Course_Clamp(c->balance_kd*rate,-.5f*fabsf(pterm),.5f*fabsf(pterm));
        integral=Course_Clamp(b->integral+c->balance_ki*pitch*dt,-limit,limit);
        raw=pterm+dterm+integral;
        if(fabsf(raw)<=limit||raw*pitch<0)b->integral=integral;
        raw=pterm+dterm+b->integral;
        raw=Course_Clamp(raw,-limit,limit);
        /* Damping may cancel the command, but the command never reverses sign:
           front-up must not command the wheels backwards. */
        if(raw*pitch<=0)raw=0;
        else if(fabsf(raw)<minmag)raw=(pitch>0?1:-1)*minmag;
        b->v=raw;
    }
    if(magnitude<=c->stable_deg&&fabsf(rate)<=c->stable_rate_dps){
        if(!b->stable_tracking){b->stable_tracking=1;b->stable_since_ms=now;}
        b->stable=(uint32_t)(now-b->stable_since_ms)>=1000U;
    }else b->stable=b->stable_tracking=0;
    return b->v;
}
float Balance_Update(BalanceController *b,float pitch,uint32_t seq,uint32_t now,int valid,const RobotConfig *c)
{
    float dt=.02f,rate=0,raw,integral;
    if(!valid||!Course_Finite(pitch)||fabsf(pitch)>c->max_tilt_deg){Balance_Reset(b);return 0;}
    if(b->initialized&&seq==b->last_seq)return b->v;
    if(b->initialized){
        dt=(float)(uint32_t)(now-b->last_ms)*.001f;
        if(dt<.005f||dt>.25f){Balance_Reset(b);return 0;}
        rate=(pitch-b->last_pitch)/dt;
        /* A short first-order rate filter, rather than the course's long FIR. */
        rate=.35f*rate+.65f*b->rate;
        /* A bias accumulated on the other side must not keep driving the board
           away from level after a crossing between two sampled Euler angles. */
        if(pitch*b->last_pitch<0)b->integral=0;
    }
    b->initialized=1;b->last_seq=seq;b->last_ms=now;b->last_pitch=pitch;b->rate=rate;
    /* Stop purely on the tilt angle, and time the hold on the tilt angle alone.
       The rate is a noisy two-sample difference: gating either the wheels or the
       10 s timer on it kept the car twitching and kept resetting the hold while
       the board was in fact level. A board that is really tipping cannot stay
       inside a 1 deg band for 10 s anyway, so the angle gate is sufficient. */
    if(fabsf(pitch)<=c->stable_deg){
        b->integral=0;b->v=0;
        /* The plank keeps swinging after the car stops, so a dead output inside
           the band lets it ring for several cycles and the 10 s hold never starts.
           While the board is clearly still moving, apply the D term alone as a
           D-term action using the same sign as outside the band; the hold timer only
           runs when the board is really quiet. The physical sign needs on-board tuning. */
        if(fabsf(rate)>8.0f){
            b->v=c->balance_sign*Course_Clamp(c->balance_kd*rate,-c->max_v,c->max_v);
            b->stable=b->stable_tracking=0;
            return b->v;
        }
        if(!b->stable_tracking){b->stable_tracking=1;b->stable_since_ms=now;}
        b->stable=(uint32_t)(now-b->stable_since_ms)>=10000U;
        return 0;
    }
    b->stable=b->stable_tracking=0;
    integral=Course_Clamp(b->integral+c->balance_ki*pitch*dt,-c->max_v,c->max_v);
    raw=c->balance_kp*pitch+c->balance_kd*rate+integral;
    if(fabsf(raw)<=c->max_v||raw*pitch<0)b->integral=integral;
    raw=c->balance_kp*pitch+c->balance_kd*rate+b->integral;
    b->v=c->balance_sign*Course_Clamp(raw,-c->max_v,c->max_v);return b->v;
}
