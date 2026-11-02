#include "course.h"
#include <string.h>

static int16_t signed_be(const uint8_t *b)
{
    uint32_t u=((uint32_t)b[0]<<8)|b[1];
    return (int16_t)(u>=32768U?(int32_t)u-65536:(int32_t)u);
}
static void consume(ImuParser *p,unsigned n)
{
    p->size=(uint8_t)(p->size-n);
    memmove(p->data,p->data+n,p->size);
}
static int legal(uint8_t type,uint8_t length)
{
    if(type==0x15||type==0x25||type==0x35||type==0x45)return length==6;
    if(type==0x65)return length==8;
    return (type==0x75||type==0x85)&&length>=1&&length<=8;
}
void Imu_Init(ImuParser *p){memset(p,0,sizeof(*p));}
int Imu_Fresh(const ImuParser *p,uint32_t now,uint32_t timeout)
{return p->have_euler&&(uint32_t)(now-p->euler_ms)<=timeout;}
int Imu_Byte(ImuParser *p,uint8_t byte,uint32_t now)
{
    unsigned i,n;uint8_t sum,type;int published=0,valid;
    if(p->size&&(uint32_t)(now-p->last_byte_ms)>50U){p->size=0;p->bad++;}
    p->last_byte_ms=now;
    if(p->size>=sizeof(p->data)){consume(p,1);p->dropped++;}
    p->data[p->size++]=byte;
    /* Each iteration consumes at least one byte or returns: bounded by 24 bytes. */
    while(p->size){
        if(p->data[0]!=0x5a){consume(p,1);p->dropped++;continue;}
        if(p->size<2)return published;
        if(p->data[1]!=0x5a){consume(p,1);p->dropped++;continue;}
        if(p->size<4)return published;
        if(!legal(p->data[2],p->data[3])){p->bad++;consume(p,1);continue;}
        n=p->data[3]+5U;if(p->size<n)return published;
        sum=0;for(i=0;i<n-1U;i++)sum=(uint8_t)(sum+p->data[i]);
        if(sum!=p->data[n-1U]){p->bad++;consume(p,1);continue;}
        type=p->data[2];valid=1;
        if(type==0x45){
            for(i=0;i<3;i++)if(signed_be(p->data+4+2*i)<-18000||signed_be(p->data+4+2*i)>18000)valid=0;
            if(valid){for(i=0;i<3;i++)p->euler_deg[i]=(float)signed_be(p->data+4+2*i)/100.0f;
                p->euler_ms=now;p->euler_seq++;p->have_euler=1;}
        }else if(type==0x65){
            for(i=0;i<4;i++)p->quaternion[i]=(float)signed_be(p->data+4+2*i)/10000.0f;
            p->quat_ms=now;
        }else if(type==0x15||type==0x25||type==0x35){
            int16_t *v=type==0x15?p->accel_raw:(type==0x25?p->gyro_raw:p->mag_raw);
            for(i=0;i<3;i++)v[i]=signed_be(p->data+4+2*i);
            if(type==0x15)p->accel_ms=now;else if(type==0x25)p->gyro_ms=now;else p->mag_ms=now;
        }else{p->status_len=p->data[3];p->status_type=type;memcpy(p->status,p->data+4,p->status_len);}
        if(valid){p->good++;published++;}else p->bad++;
        consume(p,n);
    }
    return published;
}
