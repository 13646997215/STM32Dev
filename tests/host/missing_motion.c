#include "course.h"
#include <string.h>
void Config_Default(RobotConfig*c){memset(c,0,sizeof(*c));}
void Pose_Reset(Pose*p,float x,float y,float a,const FgCapture*f){(void)x;(void)y;(void)a;(void)f;memset(p,0,sizeof(*p));}
void Pose_Update(Pose*p,const FgCapture*f,const int8_t*d,const uint8_t*t,float a,const RobotConfig*c){(void)p;(void)f;(void)d;(void)t;(void)a;(void)c;}
MotionOutput Motion_Goto(const Pose*p,float x,float y,float a,int r,const RobotConfig*c){MotionOutput o={0};(void)p;(void)x;(void)y;(void)a;(void)r;(void)c;return o;}
void Motion_WheelPoint(const Pose*p,float rx,float ry,float*x,float*y){(void)p;(void)rx;(void)ry;*x=*y=0;}
