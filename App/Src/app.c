#include "course.h"
#include <math.h>
#include <string.h>

void App_Init(CourseApp *a)
{
    memset(a,0,sizeof(*a));Config_Default(&a->config);Imu_Init(&a->imu);
    Motor_Init(&a->motor[0]);Motor_Init(&a->motor[1]);
    a->direction[0]=a->direction[1]=1;a->pose.valid=1;
}
void App_Stop(CourseApp *a,uint8_t fault)
{
    unsigned i;a->armed=0;a->mode=MODE_DIAGNOSTIC;
    if(fault)a->fault=fault;
    for(i=0;i<2;i++){
        int8_t direction=a->motor[i].direction;
        a->manual[i]=a->pwm[i]=0;Motor_Init(&a->motor[i]);
        a->motor[i].direction=direction;a->direction_known[i]=0;
    }
    a->mission.v=a->mission.w=0;
    if(a->fault){
        a->mission.state=MS_FAULT;a->mission.fault=a->fault;
        Balance_Reset(&a->mission.balance);a->mission.balance_achieved=0;
    }else if(a->mission.state!=MS_DONE&&a->mission.state!=MS_ARRIVED){
        a->mission.state=MS_IDLE;Balance_Reset(&a->mission.balance);a->mission.balance_achieved=0;
    }
}
static void orientation(CourseApp *a)
{
    float raw=Course_Wrap(a->imu.euler_deg[2]*COURSE_PI/180*a->config.yaw_sign);
    if(!a->have_yaw){a->continuous_yaw=raw;a->have_yaw=1;}
    else if(a->last_yaw_seq!=a->imu.euler_seq)a->continuous_yaw+=Course_Wrap(raw-a->last_raw_yaw);
    a->last_raw_yaw=raw;a->last_yaw_seq=a->imu.euler_seq;
    a->yaw=Course_Wrap(a->continuous_yaw-a->config.board_heading_rad);
    a->pitch=a->config.pitch_sign*(a->imu.euler_deg[a->config.pitch_axis]-a->config.pitch_zero_deg);
}
static MissionInput input(const CourseApp *a,uint32_t now)
{
    MissionInput in;memset(&in,0,sizeof(in));in.now_ms=now;in.imu_seq=a->imu.euler_seq;
    in.pose=a->pose;in.pitch_deg=a->pitch;in.yaw_rad=a->yaw;
    in.imu_valid=(uint8_t)Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms);in.pose_valid=a->pose.valid;return in;
}
uint8_t App_Command(CourseApp *a,const WireFrame *f,uint32_t now)
{
    float x,y;uint16_t key;MissionInput in;uint8_t profile;Pose candidate;
    a->frames++;
    if(f->type==CMD_STOP){if(f->length)return RESULT_FORMAT;App_Stop(a,0);return RESULT_OK;}
    if(f->type==CMD_PING||f->type==CMD_STATUS)return f->length?RESULT_FORMAT:RESULT_OK;
    if(f->type==CMD_CLEAR){
        if(f->length)return RESULT_FORMAT;
        if(a->armed)return RESULT_LOCKED;
        App_Stop(a,0);a->fault=0;a->warnings=0;memset(&a->mission,0,sizeof(a->mission));return RESULT_OK;
    }
    if(f->type==CMD_CONFIG){
        if(f->length!=6)return RESULT_FORMAT;
        if(a->armed)return RESULT_LOCKED;
        key=(uint16_t)(f->payload[0]|((uint16_t)f->payload[1]<<8));
        x=a->config.yaw_sign;
        if(!Config_Set(&a->config,key,Wire_GetFloat(f->payload+2)))return RESULT_RANGE;
        /* The previous raw yaw belongs to the old sign convention. */
        if(a->config.yaw_sign!=x)a->have_yaw=0;
        return RESULT_OK;
    }
    if(a->fault)return RESULT_FAULT;
    if(f->type==CMD_TASK1_BENCH_START){
        if(f->length!=4||memcmp(f->payload,"T1LV",4))return RESULT_FORMAT;
        if(a->have_bench_sequence&&f->sequence==a->bench_sequence)return RESULT_DUPLICATE;
        if(a->armed||a->mode!=MODE_DIAGNOSTIC)return RESULT_LOCKED;
        if(!Config_Valid(&a->config)||a->config.max_duty<.45f||
           !Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms)||
           !Course_Finite(a->imu.euler_deg[2])||a->speed[0].hz>.5f||a->speed[1].hz>.5f)return RESULT_NOT_READY;
        orientation(a);
        if(!Course_Finite(a->pitch)||fabsf(a->pitch)>a->config.max_tilt_deg)return RESULT_NOT_READY;
        memset(&a->mission,0,sizeof(a->mission));
        a->mission.profile=PROFILE_A;a->mission.state=MS_BALANCE;
        a->mission.started_ms=a->mission.state_ms=now;
        a->mission.initial_pitch=a->pitch;a->mission.heading=a->yaw;
        a->bench_sequence=f->sequence;a->have_bench_sequence=1;
        a->last_cmd_ms=now;a->have_control=0;a->mode=MODE_BENCH_TASK1;a->armed=1;
        a->pose.x=a->pose.y=0;a->pose.valid=0;a->pose.initialized=0;
        return RESULT_OK;
    }
    if(f->type==CMD_BENCH_KEEPALIVE){
        if(f->length!=2)return RESULT_FORMAT;
        if(!a->armed||a->mode!=MODE_BENCH_TASK1)return RESULT_NOT_READY;
        key=(uint16_t)(f->payload[0]|((uint16_t)f->payload[1]<<8));
        if(key!=a->bench_sequence)return RESULT_LOCKED;
        a->last_cmd_ms=now;return RESULT_OK;
    }
    if(f->type==CMD_ARM){
        if(f->length!=2||f->payload[0]!=0xDE||f->payload[1]!=0xC0)return RESULT_FORMAT;
        if(a->armed)return RESULT_LOCKED;
        if(!Config_Valid(&a->config)||(a->config.cal_flags&(CAL_POWER|CAL_SIGNS))!=(CAL_POWER|CAL_SIGNS))return RESULT_NOT_READY;
        memset(&a->mission,0,sizeof(a->mission));
        a->armed=1;a->last_cmd_ms=now;a->have_control=0;return RESULT_OK;
    }
    if(f->type==CMD_START){
        if(f->length!=1)return RESULT_FORMAT;
        if(a->have_start_seq&&f->sequence==a->last_start_seq)return RESULT_DUPLICATE;
        if(!a->armed)return RESULT_NOT_READY;
        if(a->mode!=MODE_DIAGNOSTIC)return RESULT_LOCKED;
        profile=f->payload[0];
        if(profile<PROFILE_A||profile>PROFILE_RECTANGLE)return RESULT_RANGE;
        orientation(a);candidate=a->pose;
        /* A coordinate reference is a declared setup, not a position measurement. */
        candidate.x=(profile==PROFILE_A||profile==PROFILE_RECTANGLE)?0:a->config.start_x_m;
        candidate.y=(profile==PROFILE_RECTANGLE||profile==PROFILE_A)?0:a->config.start_y_m;
        candidate.yaw=a->yaw;candidate.valid=1;candidate.initialized=0;
        in=input(a,now);in.pose=candidate;in.pose_valid=1;
        if(!Mission_Start(&a->mission,profile,&in,&a->config))return RESULT_NOT_READY;
        a->pose=candidate;a->warnings=0;
        a->mode=MODE_AUTO;a->last_start_seq=f->sequence;a->have_start_seq=1;return RESULT_OK;
    }
    if(f->type==CMD_VELOCITY||f->type==CMD_DUTY){
        if(f->length!=8)return RESULT_FORMAT;
        if(!a->armed)return RESULT_NOT_READY;
        if(a->mode==MODE_AUTO||a->mode==MODE_BENCH_TASK1)return RESULT_LOCKED;
        x=Wire_GetFloat(f->payload);y=Wire_GetFloat(f->payload+4);
        if(!Course_Finite(x)||!Course_Finite(y))return RESULT_RANGE;
        if(f->type==CMD_VELOCITY){
            if((a->config.cal_flags&CAL_MANUAL)!=CAL_MANUAL)return RESULT_NOT_READY;
            if(fabsf(x)>a->config.max_rpm||fabsf(y)>a->config.max_rpm)return RESULT_RANGE;
            a->mode=MODE_MANUAL_SPEED;
        }else {if(fabsf(x)>a->config.max_duty||fabsf(y)>a->config.max_duty)return RESULT_RANGE;a->mode=MODE_MANUAL_DUTY;}
        a->manual[0]=x;a->manual[1]=y;a->last_cmd_ms=now;return RESULT_OK;
    }
    /* Allow-listed GY953 single reads; repeated requests do not toggle stream state. */
    if(f->type==CMD_IMU_QUERY)return f->length==1&&f->payload[0]<=2?RESULT_OK:RESULT_FORMAT;
    return RESULT_FORMAT;
}
static void bench_step(CourseApp *a,const FgCapture fg[2],uint32_t now,uint32_t us,float dt)
{
    unsigned i;float target,duty;uint32_t elapsed=now-a->mission.started_ms;
    a->pose.valid=0;a->pose.last_count[0]=fg[0].total;a->pose.last_count[1]=fg[1].total;
    if(!a->armed){App_Stop(a,0);return;}
    if((uint32_t)(now-a->last_cmd_ms)>500U||elapsed>=60000U){App_Stop(a,FAULT_TIMEOUT);return;}
    if(!Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms)||!Course_Finite(a->pitch)||!Course_Finite(a->yaw)){
        App_Stop(a,FAULT_IMU);return;
    }
    if(fabsf(a->pitch)>a->config.max_tilt_deg){App_Stop(a,FAULT_TILT);return;}
    a->mission.v=Task1_Update(&a->mission.balance,a->pitch,a->imu.euler_seq,now,1,&a->config);
    a->mission.w=0;a->mission.state=a->mission.v==0?MS_HOLD:MS_BALANCE;
    target=a->mission.v==0?0:(a->mission.v>0?1:-1)*a->config.max_rpm;
    duty=Course_Clamp(.40f+.05f*Course_Clamp(fabsf(a->mission.v)/.04f,0,1),.40f,.45f);
    /* Reuse the motor's stop-before-reverse interlock, not its uncalibrated PI output. */
    for(i=0;i<2;i++){
        if(target!=0&&a->motor[i].phase==MOTOR_RUN&&(uint32_t)(now-a->motor[i].run_ms)>1500U&&
           (!fg[i].have_edge||(uint32_t)(us-fg[i].last_us)>a->config.fg_timeout_ms*1000U)){
            App_Stop(a,FAULT_FEEDBACK);return;
        }
        Motor_Update(&a->motor[i],target,a->speed[i].hz*60/a->config.ppr[i],&fg[i],now,us,dt,1,&a->config,i);
        if(a->motor[i].phase==MOTOR_FAULT){App_Stop(a,FAULT_MOTOR);return;}
        a->direction[i]=a->motor[i].direction;a->direction_known[i]=0;
        a->pwm[i]=a->motor[i].phase==MOTOR_RUN?duty:0;
    }
    /* No wheel is powered while its partner is still waiting for a safe reversal. */
    if(a->motor[0].phase!=MOTOR_RUN||a->motor[1].phase!=MOTOR_RUN){
        a->pwm[0]=a->pwm[1]=0;
        for(i=0;i<2;i++)if(a->motor[i].phase==MOTOR_RUN)a->motor[i].run_ms=now;
    }
    for(i=0;i<2;i++)a->motor[i].duty=a->pwm[i];
}
void App_Step(CourseApp *a,const FgCapture fg[2],uint32_t now,uint32_t us)
{
    float dt=.01f,rpm[2]={0,0},wheels[2],meas[2];uint8_t bad[2];unsigned i;MissionInput in;FgCapture filtered[2];
    if(a->have_control){uint32_t d=now-a->last_control_ms;if(!d)return;
        if(d>50U&&a->armed){if(a->mode!=MODE_AUTO){App_Stop(a,FAULT_TIMING);a->last_control_ms=now;return;}a->warnings|=8U;d=50U;}dt=(float)d*.001f;}
    a->last_control_ms=now;a->have_control=1;
    if(a->imu.have_euler)orientation(a);
    for(i=0;i<2;i++)Speed_Update(&a->speed[i],&fg[i],us,a->config.fg_timeout_ms*1000U);
    if(a->mode==MODE_BENCH_TASK1){bench_step(a,fg,now,us,dt);return;}
    /* Never integrate an implausible FG stream into autonomous odometry. Partner
       speed is not a position sensor and cannot establish the missing counts. */
    if(a->armed&&a->mode==MODE_AUTO){
        memcpy(filtered,fg,sizeof(filtered));
        for(i=0;i<2;i++){
            uint32_t count=fg[i].total-a->pose.last_count[i];
            /* In short intervals the final period may look valid after a burst.
               Use the count since the last pose update as well, allowing one
               whole extra pulse for quantization and unsigned counter wrap. */
            float expected=a->config.max_rpm*a->config.ppr[i]*1.5f*dt/60.0f;
            uint32_t max_count=(uint32_t)expected;
            if((float)max_count<expected)max_count++;
            max_count++;
            if((a->pose.initialized&&count>max_count)||
               (a->speed[i].fresh&&a->speed[i].hz*60.0f>a->config.max_rpm*a->config.ppr[i]*1.5f)){
                a->warnings|=1U;
                /* Discard this interval, rebase to the observed counter, and
                   report uncertain position without inventing partner pulses. */
                a->pose.last_count[i]=fg[i].total;
            }
        }
        fg=filtered;
        if(!Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms)||!Course_Finite(a->pitch)||!Course_Finite(a->yaw)||fabsf(a->pitch)>a->config.max_tilt_deg){
            a->warnings|=(!Course_Finite(a->pitch)||!Course_Finite(a->yaw)||!Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms))?4U:32U;
            for(i=0;i<2;i++){a->pwm[i]=0;a->motor[i].duty=0;a->motor[i].ramp_rpm=0;a->pose.last_count[i]=fg[i].total;}
            Balance_Reset(&a->mission.balance);return;
        }
    }
    /* Task1 is constrained to the declared deck axis; magnetic Yaw is observation only. */
    wheels[0]=(a->mode==MODE_AUTO&&a->mission.profile==PROFILE_A)?0:a->yaw;
    if(!a->pose.initialized)Pose_Reset(&a->pose,a->pose.x,a->pose.y,wheels[0],fg);
    else if(a->armed)Pose_Update(&a->pose,fg,a->direction,a->direction_known,wheels[0],&a->config);
    else {a->pose.last_count[0]=fg[0].total;a->pose.last_count[1]=fg[1].total;}
    if(a->armed&&a->mode==MODE_AUTO&&!a->pose.valid){
        a->warnings|=128U;
        Pose_Reset(&a->pose,a->pose.x,a->pose.y,wheels[0],fg);
    }
    if(a->armed&&a->mode!=MODE_AUTO&&(uint32_t)(now-a->last_cmd_ms)>500U){App_Stop(a,0);return;}
    if(a->armed&&a->mode==MODE_AUTO){
        in=input(a,now);Mission_Update(&a->mission,&in,&a->config);
        if(a->mission.fault){App_Stop(a,a->mission.fault);return;}
        if(a->mission.state==MS_DONE||a->mission.state==MS_ARRIVED){App_Stop(a,0);return;}
        Course_Mix(a->mission.v,a->mission.w,a->config.track_m,a->config.max_v,wheels);
        for(i=0;i<2;i++)rpm[i]=wheels[i]*60/(COURSE_PI*a->config.wheel_diameter_m*a->config.wheel_scale[i]);
    }else if(a->mode==MODE_MANUAL_SPEED){rpm[0]=a->manual[0];rpm[1]=a->manual[1];}
    else if(a->mode==MODE_MANUAL_DUTY){for(i=0;i<2;i++)rpm[i]=fabsf(a->manual[i])<.001f?0:(a->manual[i]>0?1:-1)*a->config.max_rpm;}
    /* Feedback plausibility: a FG line that picks up the 2 kHz motor PWM reads
       far above max_rpm. Feeding that to the PI throttles the wheel to zero, so an
       implausible reading is discarded. While driving straight or turning in place
       both wheels turn at the same rate, so the partner's reading is a valid
       substitute; if both are implausible the wheel runs without feedback. */
    for(i=0;i<2;i++){meas[i]=a->speed[i].hz*60/a->config.ppr[i];bad[i]=(meas[i]>a->config.max_rpm*1.5f)?1:0;}
    if(bad[0]&&!bad[1])meas[0]=meas[1];
    else if(bad[1]&&!bad[0])meas[1]=meas[0];
    else if(bad[0]&&bad[1])meas[0]=meas[1]=0;
    for(i=0;i<2;i++){
        Motor_Update(&a->motor[i],rpm[i],meas[i],&fg[i],now,us,dt,a->armed,&a->config,i);
        if(a->motor[i].phase==MOTOR_FAULT){
            if(a->mode!=MODE_AUTO){App_Stop(a,FAULT_MOTOR);return;}
            /* Keep the reversal interlock: retry from rest instead of declaring
               the round completed or changing direction while still moving. */
            {int8_t dir=a->motor[i].direction;Motor_Init(&a->motor[i]);a->motor[i].direction=dir;}
            a->warnings|=256U;a->pwm[0]=a->pwm[1]=0;return;
        }
        a->direction[i]=a->motor[i].direction;
        if(a->motor[i].direction_assumed)a->direction_known[i]=1;
        a->pwm[i]=a->motor[i].duty;
        if(a->mode==MODE_MANUAL_DUTY&&a->motor[i].phase==MOTOR_RUN)a->pwm[i]=fabsf(a->manual[i]);
    }
}
/* ---- unified round: KEY start, reference capture, task 1-6 flow ---- */
uint8_t Key_Event(uint32_t held_ms,int released,uint8_t state,int armed)
{
    /* A 2 s hold interrupts a running round, a latched fault, or a manual session.
       A short press (released before 500 ms) starts the next round, but only when
       nothing is armed and the previous round is idle or finished. */
    if(held_ms>=2000U)return (state==MS_IDLE&&!armed)?KEY_NONE:KEY_STOP;
    if(!released||held_ms==0||held_ms>=500U)return KEY_NONE;
    if(armed||(state!=MS_IDLE&&state!=MS_ARRIVED))return KEY_NONE;
    return KEY_START;
}
void App_StartUnified(CourseApp *a,uint32_t now)
{
    MissionInput in;
    /* Measured calibration is installed here, not in Config_Default, so the
       firmware still boots with "no calibration claimed" until a round starts.
       A config already loaded over the link (cal_flags != 0) wins: that keeps the
       PC able to override values for tuning without them being overwritten. */
    if(!a->config.cal_flags)Config_ApplyMeasured(&a->config);
    /* The reference has to come from a real attitude sample, otherwise the zero
       would be whatever the parser happens to hold. */
    if(!a->imu.have_euler||!Course_Finite(a->imu.euler_deg[a->config.pitch_axis]))return;
    a->fault=0;a->warnings=0;a->manual[0]=a->manual[1]=0;a->pwm[0]=a->pwm[1]=0;
    /* Take the current attitude as the reference: everything afterwards is
       relative to it. Yaw stays absolute so the board heading stays meaningful. */
    a->config.pitch_zero_deg=a->imu.euler_deg[a->config.pitch_axis];
    /* yaw_sign was just changed by the measured configuration, so the accumulated
       continuous-yaw history belongs to the old sign; restart it from this frame
       or the displayed heading jumps by roughly twice the current yaw. */
    a->have_yaw=0;
    orientation(a);
    a->pose.x=a->pose.y=0;a->pose.yaw=a->yaw;a->pose.valid=1;a->pose.initialized=0;
    in.now_ms=now;in.imu_seq=a->imu.euler_seq;in.pose=a->pose;
    in.pitch_deg=a->pitch;in.yaw_rad=a->yaw;
    in.imu_valid=(uint8_t)Imu_Fresh(&a->imu,now,a->config.imu_timeout_ms);in.pose_valid=1;
    if(!Mission_Start(&a->mission,PROFILE_BALANCE_RETURN,&in,&a->config))return;
    a->armed=1;a->mode=MODE_AUTO;a->last_cmd_ms=now;a->last_start_seq=0;a->have_start_seq=0;
    a->have_control=0;a->have_bench_sequence=0;a->bench_sequence=0;
}
size_t App_Telemetry(const CourseApp *a,uint32_t now,uint8_t *p,size_t cap)
{
    uint32_t quality=(uint32_t)(a->pose.valid&&!(a->warnings&(1U|128U)))|((a->warnings|a->mission.warnings)<<8);unsigned i;
    if(cap<88)return 0;
    memset(p,0,88);
    p[0]=1;p[1]=a->mode;p[2]=a->armed;p[3]=a->fault;p[4]=a->mission.state;p[5]=a->mission.profile;
    p[6]=a->motor[0].phase;p[7]=a->motor[1].phase;
    Wire_PutU32(p+8,now);Wire_PutU32(p+12,a->config.cal_flags);
    Wire_PutU32(p+16,a->imu.have_euler?now-a->imu.euler_ms:0xFFFFFFFFU);Wire_PutU32(p+20,a->imu.good);
    Wire_PutU32(p+24,a->pose.last_count[0]);Wire_PutU32(p+28,a->pose.last_count[1]);
    Wire_PutFloat(p+32,a->speed[0].hz);Wire_PutFloat(p+36,a->speed[1].hz);
    Wire_PutFloat(p+40,a->pwm[0]);Wire_PutFloat(p+44,a->pwm[1]);
    Wire_PutFloat(p+48,a->pitch);Wire_PutFloat(p+52,a->yaw);Wire_PutFloat(p+56,a->pose.x);Wire_PutFloat(p+60,a->pose.y);
    Wire_PutFloat(p+64,a->manual[0]);Wire_PutFloat(p+68,a->manual[1]);
    Wire_PutU32(p+72,a->imu.bad);Wire_PutU32(p+76,a->commands_rejected);
    for(i=0;i<2;i++)if(a->direction_known[i]){
        quality|=1U<<(1U+i);
        if(a->motor[i].phase!=MOTOR_RUN)quality|=1U<<(3U+i);
    }
    Wire_PutU32(p+80,a->mission.weight_events);Wire_PutU32(p+84,quality);return 88;
}
