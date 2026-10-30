#include "course.h"
uint16_t Wire_Crc(const uint8_t*p,size_t n){(void)p;(void)n;return 0;}
size_t Wire_Encode(const WireFrame*f,uint8_t*b,size_t c){(void)f;(void)b;(void)c;return 0;}
int Wire_Byte(WireParser*p,uint8_t b,uint32_t t,WireFrame*o){(void)p;(void)b;(void)t;(void)o;return 0;}
void Wire_PutFloat(uint8_t*p,float x){(void)p;(void)x;}
float Wire_GetFloat(const uint8_t*p){(void)p;return 0;}
void Wire_PutU32(uint8_t*p,uint32_t x){(void)p;(void)x;}
uint32_t Wire_GetU32(const uint8_t*p){(void)p;return 0;}
