#include "course.h"
#include <math.h>
#include <float.h>
#include <string.h>

typedef struct {uint16_t key;size_t offset;float lo,hi;} Field;
#define F(k,n,l,h) {k,offsetof(RobotConfig,n),l,h}
static const Field fields[]={
 F(1,wheel_diameter_m,.01f,.3f),F(2,track_m,.03f,.8f),F(3,length_m,.03f,1),F(4,width_m,.03f,.29f),
 F(5,ppr[0],1,100000),F(6,ppr[1],1,100000),F(7,wheel_scale[0],.5f,1.5f),F(8,wheel_scale[1],.5f,1.5f),
 F(10,max_duty,.05f,1),F(11,deadband[0],0,.8f),F(12,deadband[1],0,.8f),
 F(13,kp[0],0,1),F(14,ki[0],0,10),F(15,kp[1],0,1),F(16,ki[1],0,10),F(17,max_rpm,1,300),F(18,rpm_slew,1,1000),
 F(20,max_v,.01f,.5f),F(21,max_w,.05f,4),F(22,min_v,.001f,.2f),F(23,heading_kp,.1f,10),
 F(30,pitch_zero_deg,-90,90),F(31,board_heading_rad,-COURSE_PI,COURSE_PI),F(32,yaw_sign,-1,1),F(33,pitch_sign,-1,1),
 F(34,balance_kp,0,1),F(35,balance_ki,0,1),F(36,balance_kd,0,1),F(37,balance_sign,-1,1),
 F(38,stable_deg,.05f,2),F(39,stable_rate_dps,.05f,10),F(40,disturb_deg,.1f,10),F(41,max_tilt_deg,5,40),
 F(42,tip_delta_deg,.5f,15),F(43,seek_min_m,.02f,1),F(44,seek_max_m,.1f,1.55f),F(45,climb_end_m,.05f,1.5f),
 F(46,start_x_m,-1,0),F(47,start_y_m,-.5f,.5f),F(48,return_x_m,-.15f,.2f),
 F(50,path_a_m,.1f,3),F(51,path_b_m,.1f,3),F(52,dock_x_m,-.4f,.4f),F(53,dock_y_m,-.4f,.4f),
 F(54,pos_tol_m,.001f,.05f),F(55,yaw_tol_rad,.005f,.2f),F(56,boundary_margin_m,0,.05f),
 F(58,min_duty,0,1)
};
int Course_Finite(float x){return x==x&&x<=FLT_MAX&&x>=-FLT_MAX;}
float Course_Clamp(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
float Course_Wrap(float x){
    float y;if(!Course_Finite(x))return 0;
    y=fmodf(x+COURSE_PI,2.0f*COURSE_PI);if(y<0)y+=2.0f*COURSE_PI;
    y-=COURSE_PI;return y>=COURSE_PI?-COURSE_PI:y;
}
void Config_Default(RobotConfig *c)
{
    memset(c,0,sizeof(*c));c->wheel_diameter_m=.067f;c->track_m=.140f;c->length_m=.200f;c->width_m=.190f;
    c->ppr[0]=c->ppr[1]=738;c->wheel_scale[0]=c->wheel_scale[1]=1;
    c->max_duty=.55f;c->deadband[0]=c->deadband[1]=.38f;
    c->kp[0]=c->kp[1]=.008f;c->ki[0]=c->ki[1]=.01f;c->max_rpm=40;c->rpm_slew=30;
    c->max_v=.10f;c->max_w=.7f;c->min_v=.015f;c->heading_kp=1.5f;
    c->yaw_sign=c->pitch_sign=c->balance_sign=1;c->pitch_axis=1;
    c->balance_kp=.025f;c->balance_ki=.001f;c->balance_kd=.012f;
    c->stable_deg=.5f;c->stable_rate_dps=1;c->disturb_deg=1.5f;c->max_tilt_deg=18;
    c->tip_delta_deg=3;c->seek_min_m=.2f;c->seek_max_m=1.25f;c->climb_end_m=.2f;
    c->start_x_m=-.3f;c->return_x_m=.08f;c->path_a_m=.5f;c->path_b_m=.3f;
    c->pos_tol_m=.008f;c->yaw_tol_rad=.035f;c->boundary_margin_m=.015f;c->min_duty=0;
    c->imu_timeout_ms=200;c->fg_timeout_ms=250;c->reversal_quiet_ms=200;
    c->forward_release[0]=0;c->forward_release[1]=1;c->start_requires_ramp=1;c->fg_fault_en=1;
}
void Config_ApplyMeasured(RobotConfig *c)
{
    /* Measured on this machine, installed when a KEY press starts a round so the
       whole task 1-6 flow runs with no PC attached. Sources: wheel diameter and
       track from the user, ppr 738 = teacher demo 82*9 (corroborated by the duty
       sweep), duty window and speed-loop gains from the measured duty->RPM curve
       (migration/results/fg-sweep-2026-10-20.md), board heading 90 deg magnetic.
       max_v and balance_kp are the conservative round-1 values; the faster set is
       in docs/unified-task-flow.md for round 2. */
    c->wheel_diameter_m=.067f;c->track_m=.195f;
    c->ppr[0]=c->ppr[1]=738;c->wheel_scale[0]=c->wheel_scale[1]=1;
    c->max_duty=1.0f;c->deadband[0]=c->deadband[1]=.35f;
    c->min_duty=.35f;   /* User reports both wheels respond at 35%; verify under load. */
    c->kp[0]=c->kp[1]=.012f;c->ki[0]=c->ki[1]=.02f;c->max_rpm=50;c->rpm_slew=60;
    c->max_v=.06f;c->min_v=.015f;
    c->yaw_sign=-1;c->pitch_sign=1;c->pitch_axis=1;
    /* Initial field-tuning trial; not a claim of closed-loop stability. */
    c->balance_kp=.006f;c->balance_ki=.0003f;c->balance_kd=.012f;
    c->max_tilt_deg=40;c->climb_end_m=.5f;c->yaw_tol_rad=.035f;
    /* The course balance criterion is |dA-dB| < 30 mm over 1600 mm = 1.07 deg, so
       a 1.0 deg band still satisfies it while giving the on/off duty enough room
       to settle. The rate gate is loosened because the derived rate is noisy. */
    c->stable_deg=1.0f;c->stable_rate_dps=3.0f;
    c->board_heading_rad=-1.5708f;
    c->pitch_zero_deg=0;   /* replaced by the start-up reference capture */
    c->forward_release[0]=1;c->forward_release[1]=0;
    c->fg_fault_en=0;   /* an unreliable FG line must not stop the whole round */
    c->cal_flags=CAL_PATH; /* this flow claims nothing beyond CAL_PATH */
}
int Config_Valid(const RobotConfig *c)
{
    unsigned i;float x;
    for(i=0;i<sizeof(fields)/sizeof(fields[0]);i++){
        memcpy(&x,(const uint8_t*)c+fields[i].offset,sizeof(x));
        if(!Course_Finite(x)||x<fields[i].lo||x>fields[i].hi)return 0;
    }
    return (c->yaw_sign==1||c->yaw_sign==-1)&&(c->pitch_sign==1||c->pitch_sign==-1)&&
        (c->balance_sign==1||c->balance_sign==-1)&&c->cal_flags<=255&&c->pitch_axis<=1&&
        c->forward_release[0]<=1&&c->forward_release[1]<=1&&c->start_requires_ramp<=1&&c->fg_fault_en<=1&&
        c->imu_timeout_ms>=50&&c->imu_timeout_ms<=1000&&c->fg_timeout_ms>=50&&c->fg_timeout_ms<=2000&&
        c->reversal_quiet_ms>=100&&c->reversal_quiet_ms<=1000&&
        c->deadband[0]<c->max_duty&&c->deadband[1]<c->max_duty&&c->min_v<=c->max_v&&
        c->stable_deg<c->disturb_deg&&c->disturb_deg<c->max_tilt_deg&&c->seek_min_m<c->seek_max_m;
}
int Config_Set(RobotConfig *c,uint16_t key,float value)
{
    RobotConfig next=*c;unsigned i;int found=0;
    if(!Course_Finite(value))return 0;
    for(i=0;i<sizeof(fields)/sizeof(fields[0]);i++)if(fields[i].key==key){memcpy((uint8_t*)&next+fields[i].offset,&value,sizeof(value));found=1;break;}
    if(!found){
        if(value<0||value>65535||value!=(float)(uint32_t)value)return 0;
        switch(key){
        case 100:if(value>255)return 0;next.cal_flags=(uint32_t)value;break;
        case 101:if(value>1)return 0;next.pitch_axis=(uint8_t)value;break;
        case 102:case 103:if(value>1)return 0;next.forward_release[key-102]=(uint8_t)value;break;
        case 104:if(value>1)return 0;next.start_requires_ramp=(uint8_t)value;break;
        case 59:if(value>1)return 0;next.fg_fault_en=(uint8_t)value;break;
        case 105:next.imu_timeout_ms=(uint32_t)value;break;
        case 106:next.fg_timeout_ms=(uint32_t)value;break;
        case 107:next.reversal_quiet_ms=(uint32_t)value;break;
        default:return 0;}
    }
    if(!Config_Valid(&next))return 0;
    /* Any parameter edit invalidates the previous physical calibration claim. */
    if(key!=100)next.cal_flags=0;
    *c=next;return 1;
}
void Course_Mix(float v,float w,float track,float limit,float out[2])
{
    float peak;
    out[0]=out[1]=0;
    if(!Course_Finite(v)||!Course_Finite(w)||!Course_Finite(track)||!Course_Finite(limit)||limit<=0||track<=0)return;
    out[0]=v-w*track*.5f;out[1]=v+w*track*.5f;
    peak=fmaxf(fabsf(out[0]),fabsf(out[1]));
    if(peak>limit){out[0]*=limit/peak;out[1]*=limit/peak;}
}
int Course_Inside(float x,float y,float yaw,const RobotConfig *c)
{
    float hx,hy;
    if(!Course_Finite(x)||!Course_Finite(y)||!Course_Finite(yaw))return 0;
    hx=.5f*(c->length_m*fabsf(cosf(yaw))+c->width_m*fabsf(sinf(yaw)));
    hy=.5f*(c->length_m*fabsf(sinf(yaw))+c->width_m*fabsf(cosf(yaw)));
    /* The A ramp corridor is handled by mission until the complete body is on deck. */
    return fabsf(y)+hy+c->boundary_margin_m<.15f&&x+hx+c->boundary_margin_m<1.6f;
}
