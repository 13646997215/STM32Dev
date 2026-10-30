#include "course.h"
#include <string.h>
void App_Init(CourseApp*a){memset(a,0,sizeof(*a));}
uint8_t App_Command(CourseApp*a,const WireFrame*f,uint32_t t){(void)a;(void)f;(void)t;return RESULT_FORMAT;}
void App_Step(CourseApp*a,const FgCapture*f,uint32_t t,uint32_t u){(void)a;(void)f;(void)t;(void)u;}
void App_Stop(CourseApp*a,uint8_t f){(void)a;(void)f;}
size_t App_Telemetry(const CourseApp*a,uint32_t t,uint8_t*p,size_t n){(void)a;(void)t;(void)p;(void)n;return 0;}
void Wire_PutFloat(uint8_t*p,float x){memcpy(p,&x,4);}
