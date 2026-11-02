#include "course.h"
#include <string.h>
#include <math.h>

void Pose_Reset(Pose *p,float x,float y,float yaw,const FgCapture fg[2])
{
    memset(p,0,sizeof(*p));p->x=x;p->y=y;p->yaw=yaw;p->last_count[0]=fg[0].total;p->last_count[1]=fg[1].total;p->initialized=p->valid=1;
}
void Pose_Update(Pose *p,const FgCapture fg[2],const int8_t dir[2],const uint8_t trusted[2],float yaw,const RobotConfig *c)
{
    uint32_t dn[2];float dl,dr,ds,mid;unsigned i;
    if(!p->initialized){Pose_Reset(p,0,0,yaw,fg);return;}
    for(i=0;i<2;i++){
        dn[i]=fg[i].total-p->last_count[i];p->last_count[i]=fg[i].total;
        if((dn[i]&&(!trusted[i]||(dir[i]!=1&&dir[i]!=-1)))||dn[i]>10000U)p->valid=0;
    }
    if(!Course_Finite(yaw))p->valid=0;
    if(!p->valid)return;
    dl=(float)dn[0]*c->wheel_diameter_m*COURSE_PI*c->wheel_scale[0]/c->ppr[0]*dir[0];
    dr=(float)dn[1]*c->wheel_diameter_m*COURSE_PI*c->wheel_scale[1]/c->ppr[1]*dir[1];
    ds=(dl+dr)*.5f;mid=p->yaw+Course_Wrap(yaw-p->yaw)*.5f;
    p->x+=ds*cosf(mid);p->y+=ds*sinf(mid);p->distance+=ds;p->yaw=yaw;
}
MotionOutput Motion_Goto(const Pose *p,float x,float y,float final_yaw,int reverse,const RobotConfig *c)
{
    MotionOutput o={0};float dx=x-p->x,dy=y-p->y,dist=sqrtf(dx*dx+dy*dy),desired,error;
    if(!p->valid||!Course_Finite(p->yaw)||!Course_Finite(dist)||!Course_Finite(final_yaw))return o;
    if(dist<=c->pos_tol_m){
        error=Course_Wrap(final_yaw-p->yaw);
        if(fabsf(error)<=c->yaw_tol_rad)o.reached=1;
        else o.w=Course_Clamp(c->heading_kp*error,-c->max_w,c->max_w);
        return o;
    }
    desired=atan2f(dy,dx);if(reverse)desired=Course_Wrap(desired+COURSE_PI);
    error=Course_Wrap(desired-p->yaw);o.w=Course_Clamp(c->heading_kp*error,-c->max_w,c->max_w);
    if(fabsf(error)<.3f){o.v=Course_Clamp(dist*.8f,c->min_v,c->max_v);if(reverse)o.v=-o.v;}
    return o;
}
void Motion_WheelPoint(const Pose *p,float rx,float ry,float *x,float *y)
{*x=p->x+cosf(p->yaw)*rx-sinf(p->yaw)*ry;*y=p->y+sinf(p->yaw)*rx+cosf(p->yaw)*ry;}
