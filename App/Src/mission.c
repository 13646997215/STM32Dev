#include "course.h"
#include <math.h>
#include <string.h>

static void state(Mission *m,uint8_t s,uint32_t now)
{m->previous_state=m->state;m->state=s;m->state_ms=now;m->v=m->w=0;}
static void fail(Mission *m,uint8_t f,uint32_t now){m->fault=f;state(m,MS_FAULT,now);}
int Mission_Start(Mission *m,uint8_t profile,const MissionInput *in,const RobotConfig *c)
{
    uint32_t required=CAL_PATH;
    if(profile<PROFILE_A||profile>PROFILE_RECTANGLE||!Config_Valid(c)||!in->imu_valid||!in->pose_valid)return 0;
    /* Unified task 1-6 flow is the only flow: no tip detection and no free start
       position, so ENTRY/BALANCE claims are not part of the gate. balance_sign
       correctness is established by the first on-board run instead. */
    if((c->cal_flags&required)!=required)return 0;
    memset(m,0,sizeof(*m));m->profile=profile;m->started_ms=in->now_ms;m->heading=0;
    m->origin_x=in->pose.x;m->origin_y=in->pose.y;m->initial_pitch=in->pitch_deg;
    if(profile==PROFILE_A){m->seek_start_x=in->pose.x;state(m,MS_BALANCE,in->now_ms);}
    else if(profile==PROFILE_RECTANGLE)state(m,MS_PATH,in->now_ms);
    else {m->align_ref=fabsf(in->yaw_rad);state(m,MS_ALIGN,in->now_ms);}
    return 1;
}
static void use_motion(Mission *m,MotionOutput o){m->v=o.v;m->w=o.w;}
int Mission_TipDetected(Mission *m,float pitch,int ready,uint32_t now,const RobotConfig *c)
{
    if(ready&&Course_Finite(pitch)&&fabsf(pitch-m->initial_pitch)>=c->tip_delta_deg){
        if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}
        return (uint32_t)(now-m->trigger_ms)>=200U;
    }
    m->tip_tracking=0;return 0;
}
void Mission_Update(Mission *m,const MissionInput *in,const RobotConfig *c)
{
    uint32_t now=in->now_ms,elapsed=now-m->started_ms;MotionOutput o;
    float error,dx,dy,wheelx,wheely;unsigned i;
    static const int8_t px[]={1,1,0,0,0,1,1,0};
    static const int8_t py[]={0,-1,-1,0,-1,-1,0,0};
    static const float pa[]={0,-COURSE_PI/2,-COURSE_PI,COURSE_PI/2,-COURSE_PI/2,0,COURSE_PI/2,-COURSE_PI};
    m->v=m->w=0;
    if(m->state==MS_IDLE||m->state==MS_FAULT||m->state==MS_DONE||m->state==MS_ARRIVED)return;
    if(!in->imu_valid){fail(m,FAULT_IMU,now);return;}
    if(!in->pose_valid||!in->pose.valid||!Course_Finite(in->pose.x)||!Course_Finite(in->pose.y)||
       !Course_Finite(in->pose.yaw)||!Course_Finite(in->yaw_rad)){fail(m,FAULT_POSE,now);return;}
    if(!Course_Finite(in->pitch_deg)||fabsf(in->pitch_deg)>c->max_tilt_deg){fail(m,FAULT_TILT,now);return;}
    /* Lateral/longitudinal envelope guard only applies to the profiles that use a
       free start position. The unified task 1-6 flow has no position measurement
       (single-channel FG), so an odometry envelope only false-trips. */
    if((m->profile==PROFILE_ENTRY||m->profile==PROFILE_PRELIM)&&m->state>=MS_CLIMB&&
       !Course_Inside(in->pose.x,in->pose.y,in->pose.yaw,c)){fail(m,FAULT_BOUNDARY,now);return;}
    if(m->profile==PROFILE_A){
        if(elapsed>60000U){fail(m,FAULT_TIMEOUT,now);return;}
        /* Task 1 is plain Pitch leveling: no longitudinal or lateral corridor is
           enforced. Wheel-count odometry is not a position measurement, so a
           distance guard here fired on slip and on hand-held runs instead of on
           anything real. Only the course 60 s bound and genuine faults remain. */
        m->v=Task1_Update(&m->balance,in->pitch_deg,in->imu_seq,now,1,c);m->w=0;
        if(m->balance.stable){m->balance_achieved=1;state(m,MS_DONE,now);}
        return;
    }
    if((m->state==MS_ALIGN||m->state==MS_APPROACH||m->state==MS_CLIMB||m->state==MS_SEEK)&&elapsed>60000U){if(m->profile==PROFILE_BALANCE_RETURN||m->profile==PROFILE_WEIGHT_RETURN||m->profile==PROFILE_REWEIGHT)m->warnings|=2U;else{fail(m,FAULT_TIMEOUT,now);return;}}
    if((m->state==MS_BALANCE||m->state==MS_HOLD)&&!m->balance_achieved&&
       (uint32_t)(now-m->balance_ms)>90000U){if(m->profile==PROFILE_BALANCE_RETURN||m->profile==PROFILE_WEIGHT_RETURN||m->profile==PROFILE_REWEIGHT)m->warnings|=2U;else{fail(m,FAULT_TIMEOUT,now);return;}}
    if(m->profile==PROFILE_RECTANGLE&&elapsed>180000U){fail(m,FAULT_TIMEOUT,now);return;}
    switch(m->state){
    case MS_ALIGN:
        /* Turn in place onto the configured board heading. a->yaw is already the
           heading error against board_heading_rad, so the target is yaw == 0 and
           the wrapped error already points the shorter way round. One 400 ms probe
           then checks the sign convention itself: if turning made the error
           bigger, the drive is flipped, so a wrong yaw sign can never send the car
           the long way round. Requiring the band for 500 ms averages out the
           magnetometer jitter instead of trusting a single sample. */
        error=in->yaw_rad;
        /* Near 180 deg both ways are equally short, so the wrapped sign can flip
           back and forth while the car is still moving. Commit to one direction
           while the error is that large; below 90 deg the sign is unambiguous. */
        if(fabsf(error)>1.57f){if(m->align_dir==0)m->align_dir=(error>0)?-1:1;}
        else m->align_dir=0;
        /* Probe the sign convention once, with a wide margin so jitter cannot trip
           it: only a genuine 180 deg mistake grows the error by half a radian. */
        if(m->align_sign==0&&(uint32_t)(now-m->state_ms)>=400U)
            m->align_sign=(m->align_ref>.4f&&fabsf(error)>m->align_ref+.5f)?-1:1;
        {
            /* PD on the heading error: plain P overshot the 2 deg band and needed a
               few swings to settle, because every sign flip also costs the 200 ms
               reversal blanking. Damps with the measured closing rate. */
            float mag,u,rate_h=0;
            if(m->align_prev_ms&&(uint32_t)(now-m->align_prev_ms)<250U&&now>m->align_prev_ms)
                rate_h=Course_Wrap(error-m->align_prev)/((float)(uint32_t)(now-m->align_prev_ms)*.001f);
            m->align_prev=error;m->align_prev_ms=now;
            u=-(c->heading_kp*error+ALIGN_DAMP*rate_h);
            mag=fminf(fabsf(u),c->max_w);
            {
                int8_t s=(u>0)?1:-1;
                if(m->align_dir!=0)s=m->align_dir;
                if(m->align_sign<0)s=(int8_t)-s;
                m->w=(float)s*mag;
            }
        }
        if(fabsf(error)<=c->yaw_tol_rad){
            if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}
            else if((uint32_t)(now-m->trigger_ms)>=500U)state(m,MS_CLIMB,now);
        }else m->tip_tracking=0;
        break;
    case MS_APPROACH:
        o=Motion_Goto(&in->pose,-.05f,0,0,0,c);use_motion(m,o);
        if(o.reached)state(m,MS_CLIMB,now);
        break;
    case MS_CLIMB:
        /* Straight run onto the board; the heading is held parallel to the board
           axis and the distance, not a pose target, decides when to stop. */
        m->v=fminf(c->max_v,.10f);
        m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
        if(in->pose.x>=c->climb_end_m){Balance_Reset(&m->balance);m->balance_ms=now;state(m,MS_BALANCE,now);}
        break;
    case MS_SEEK:
        m->v=fminf(c->max_v,.04f);m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
        if(in->pose.x>c->seek_max_m){fail(m,FAULT_BOUNDARY,now);break;}
        if(Mission_TipDetected(m,in->pitch_deg,in->pose.x-m->seek_start_x>=c->seek_min_m,now,c)){
                if(m->profile==PROFILE_ENTRY)state(m,MS_DONE,now);
                else if(m->profile==PROFILE_PRELIM){m->return_ms=now;state(m,MS_RETURN,now);}
                else {Balance_Reset(&m->balance);m->balance_ms=now;state(m,MS_BALANCE,now);}
        }
        break;
    case MS_BALANCE:case MS_HOLD:
        m->v=Balance_Update(&m->balance,in->pitch_deg,in->imu_seq,now,1,c);
        m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
        if(m->state==MS_HOLD&&fabsf(in->pitch_deg)>=c->disturb_deg){
            m->weight_events++;m->balance_ms=now;state(m,MS_BALANCE,now);
        }else if(m->state==MS_BALANCE&&m->balance.stable){
            m->balance_achieved=1;
            if(m->profile==PROFILE_REWEIGHT&&m->weight_events==0)state(m,MS_HOLD,now);
            else {m->return_ms=now;state(m,MS_RETURN,now);}
        }
        if(m->balance_achieved&&m->state==MS_BALANCE&&(uint32_t)(now-m->balance_ms)>90000U){if(m->profile==PROFILE_REWEIGHT)m->warnings|=2U;else fail(m,FAULT_TIMEOUT,now);}
        break;
    case MS_RETURN:{
        /* Unified flow: reverse off the deck. Wheel odometry cannot tell a slip
           from real travel, so the board slope is the signal. Stage 1 latches once
           the deck is clearly tilted, stage 2 ends the retreat when the relative
           pitch is back to zero and stays there; distance and time are last-resort
           stops and cannot establish that the car actually reached the table. */
        uint8_t unified=(m->profile==PROFILE_BALANCE_RETURN||m->profile==PROFILE_WEIGHT_RETURN||m->profile==PROFILE_REWEIGHT);
        uint32_t limit=(m->profile==PROFILE_PRELIM)?5000U:60000U;
        if((uint32_t)(now-m->return_ms)>limit){
            if(unified)m->warnings|=2U;else{fail(m,FAULT_TIMEOUT,now);break;}
        }
        if(unified){
            m->v=-fminf(c->max_v,.08f);
            m->w=Course_Clamp(-c->heading_kp*in->yaw_rad,-c->max_w,c->max_w);
            if(fabsf(in->pose.x)>1.20f)m->warnings|=16U;
            if(!m->ramp_seen){
                if(fabsf(in->pitch_deg)>3.0f){
                    if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}
                    else if((uint32_t)(now-m->trigger_ms)>=300U){m->ramp_seen=1;m->tip_tracking=0;}
                }else m->tip_tracking=0;
            }else if(fabsf(in->pitch_deg)<1.5f){
                if(!m->tip_tracking){m->tip_tracking=1;m->trigger_ms=now;}
                else if((uint32_t)(now-m->trigger_ms)>=1000U){m->v=m->w=0;state(m,MS_ARRIVED,now);}
            }else m->tip_tracking=0;
            break;
        }
        o=Motion_Goto(&in->pose,c->return_x_m,0,0,1,c);use_motion(m,o);
        /* Large turns on deck are disallowed; return is longitudinal reverse. */
        if(fabsf(in->yaw_rad)>.45f){fail(m,FAULT_BOUNDARY,now);break;}
        if(o.reached)state(m,MS_ARRIVED,now);
        break;}
    case MS_PATH:
        i=m->path_index;if(i>=8){state(m,MS_DOCK,now);break;}
        o=Motion_Goto(&in->pose,m->origin_x+px[i]*c->path_a_m,m->origin_y+py[i]*c->path_b_m,pa[i],0,c);use_motion(m,o);
        if(o.reached){m->path_index++;m->v=m->w=0;if(m->path_index==4||m->path_index==8)state(m,MS_DOCK,now);}
        break;
    case MS_DOCK:
        o=Motion_Goto(&in->pose,m->origin_x,m->origin_y,0,0,c);use_motion(m,o);
        Motion_WheelPoint(&in->pose,c->dock_x_m,c->dock_y_m,&wheelx,&wheely);
        dx=wheelx-(m->origin_x+c->dock_x_m);dy=wheely-(m->origin_y+c->dock_y_m);
        if(o.reached&&sqrtf(dx*dx+dy*dy)<=c->pos_tol_m)state(m,m->path_index==4?MS_PATH:MS_DONE,now);
        break;
    default:fail(m,FAULT_CONFIG,now);break;
    }
}
