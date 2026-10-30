/* Test-only baseline: deliberately absent functionality, never linked in firmware. */
#include "course.h"
#include <string.h>
void Imu_Init(ImuParser *p){memset(p,0,sizeof(*p));}
int Imu_Byte(ImuParser *p,uint8_t b,uint32_t ms){(void)p;(void)b;(void)ms;return 0;}
int Imu_Fresh(const ImuParser *p,uint32_t n,uint32_t t){(void)p;(void)n;(void)t;return 0;}
