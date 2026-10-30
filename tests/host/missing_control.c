#include "course.h"
#include <string.h>
void Config_Default(RobotConfig*c){memset(c,0,sizeof(*c));}
void Fg_Edge(FgCapture*f,uint32_t t){(void)f;(void)t;}
void Speed_Update(SpeedEstimate*s,const FgCapture*f,uint32_t n,uint32_t t){(void)s;(void)f;(void)n;(void)t;}
float Pi_Update(PiController*p,float e,float d,float k,float i,float f,float l){(void)p;(void)e;(void)d;(void)k;(void)i;(void)f;(void)l;return 0;}
void Motor_Init(MotorController*m){memset(m,0,sizeof(*m));}
void Motor_Update(MotorController*m,float t,float r,const FgCapture*f,uint32_t n,uint32_t u,float d,int e,const RobotConfig*c,unsigned i){(void)m;(void)t;(void)r;(void)f;(void)n;(void)u;(void)d;(void)e;(void)c;(void)i;}
