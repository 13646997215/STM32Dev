#include "course.h"
#include <string.h>
void Config_Default(RobotConfig*c){memset(c,0,sizeof(*c));}
int Config_Valid(const RobotConfig*c){(void)c;return 0;}
int Config_Set(RobotConfig*c,uint16_t k,float v){(void)c;(void)k;(void)v;return 0;}
float Course_Wrap(float x){return x;}
float Course_Clamp(float x,float l,float h){(void)l;(void)h;return x;}
int Course_Finite(float x){(void)x;return 1;}
void Course_Mix(float v,float w,float t,float l,float o[2]){(void)v;(void)w;(void)t;(void)l;o[0]=o[1]=0;}
int Course_Inside(float x,float y,float a,const RobotConfig*c){(void)x;(void)y;(void)a;(void)c;return 0;}
