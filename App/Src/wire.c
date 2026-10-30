#include "course.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
uint16_t Wire_Crc(const uint8_t *p,size_t n)
{
    uint16_t c=0xffff;unsigned j;size_t i;
    for(i=0;i<n;i++){c^=(uint16_t)p[i]<<8;for(j=0;j<8;j++)c=(uint16_t)((c&0x8000)?(c<<1)^0x1021:c<<1);}
    return c;
}
void Wire_PutU32(uint8_t *p,uint32_t v){unsigned i;for(i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
uint32_t Wire_GetU32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
void Wire_PutFloat(uint8_t *p,float x){uint32_t u;memcpy(&u,&x,4);Wire_PutU32(p,u);}
float Wire_GetFloat(const uint8_t *p){uint32_t u=Wire_GetU32(p);float x;memcpy(&x,&u,4);return x;}
size_t Wire_Encode(const WireFrame *f,uint8_t *b,size_t cap)
{
    size_t n=f->length+10U;uint16_t crc;
    if(f->length>COURSE_MAX_PAYLOAD||cap<n)return 0;
    b[0]=0xa7;b[1]=0xc3;b[2]=1;b[3]=f->type;b[4]=(uint8_t)f->length;b[5]=(uint8_t)(f->length>>8);
    b[6]=(uint8_t)f->sequence;b[7]=(uint8_t)(f->sequence>>8);memcpy(b+8,f->payload,f->length);
    crc=Wire_Crc(b,n-2);b[n-2]=(uint8_t)crc;b[n-1]=(uint8_t)(crc>>8);return n;
}
static void shift(WireParser *p,unsigned n){p->size=(uint16_t)(p->size-n);memmove(p->data,p->data+n,p->size);}
int Wire_Byte(WireParser *p,uint8_t byte,uint32_t now,WireFrame *out)
{
    unsigned n;
    if(p->size&&(uint32_t)(now-p->last_ms)>100U){p->size=0;p->bad++;}
    p->last_ms=now;if(p->size==sizeof(p->data)){shift(p,1);p->bad++;}
    p->data[p->size++]=byte;
    while(p->size){
        if(p->data[0]!=0xa7){shift(p,1);continue;}
        if(p->size<2)return 0;
        if(p->data[1]!=0xc3){shift(p,1);continue;}
        if(p->size<8)return 0;
        if(p->data[2]!=1||u16(p->data+4)>COURSE_MAX_PAYLOAD){p->bad++;shift(p,1);continue;}
        n=u16(p->data+4)+10U;if(p->size<n)return 0;
        if(Wire_Crc(p->data,n-2)!=u16(p->data+n-2)){p->bad++;shift(p,1);continue;}
        out->type=p->data[3];out->length=u16(p->data+4);out->sequence=u16(p->data+6);
        memcpy(out->payload,p->data+8,out->length);p->good++;shift(p,n);return 1;
    }return 0;
}
