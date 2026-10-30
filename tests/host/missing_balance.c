#include "course.h"
#include <string.h>
void Config_Default(RobotConfig*c){memset(c,0,sizeof(*c));}
void Balance_Reset(BalanceController*b){memset(b,0,sizeof(*b));}
float Balance_Update(BalanceController*b,float p,uint32_t s,uint32_t n,int v,const RobotConfig*c){(void)b;(void)p;(void)s;(void)n;(void)v;(void)c;return 0;}
