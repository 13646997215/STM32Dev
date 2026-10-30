#include "test.h"
static float float_args(float a,float b,float c,float d,float e,float f){return a+b+c+d+e+f;}
int main(void){volatile float a=COURSE_PI,b=2*COURSE_PI;float rem=fmodf(a,b);
 CHECK(sizeof(float)==4&&sizeof(uint32_t)==4);
 NEAR(rem,a,1e-6);NEAR(Course_Wrap(0),0,1e-6);NEAR(float_args(.1f,.2f,.3f,.4f,.5f,.6f),2.1f,1e-5);
 NEAR(cosf(0),1,1e-6);NEAR(sinf(COURSE_PI/2),1,1e-6);DONE();}
