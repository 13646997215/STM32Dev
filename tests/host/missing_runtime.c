#include "runtime.h"
CourseApp g_course;volatile uint32_t g_runtime_max_us,g_runtime_cycles;
void Runtime_Init(void){}void Runtime_Tick(uint32_t now){(void)now;}
size_t Wire_Encode(const WireFrame*f,uint8_t*p,size_t n){(void)f;(void)p;(void)n;return 0;}
