/* Host-only C99 float adapters for TinyCC's old Windows CRT headers.
   The firmware uses newlib math.h. No algorithm or result is mocked here. */
#ifndef COURSE_HOST_MATH_H
#define COURSE_HOST_MATH_H
double sin(double);double cos(double);double atan2(double,double);double sqrt(double);
double fmod(double,double);double fabs(double);double floor(double);
#define NAN (0.0f/0.0f)
#define INFINITY (1.0f/0.0f)
#define sinf(x) ((float)sin((double)(x)))
#define cosf(x) ((float)cos((double)(x)))
#define atan2f(y,x) ((float)atan2((double)(y),(double)(x)))
#define sqrtf(x) ((float)sqrt((double)(x)))
#define fmodf(x,y) ((float)fmod((double)(x),(double)(y)))
#define fabsf(x) ((float)fabs((double)(x)))
/* Production code checks finite inputs before these calls. */
#define fmaxf(x,y) ((x)>(y)?(x):(y))
#define fminf(x,y) ((x)<(y)?(x):(y))
#endif
