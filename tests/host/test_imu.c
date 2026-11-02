#include "test.h"
static void frame(uint8_t type,const uint8_t *data,uint8_t len,uint8_t *out){unsigned i;uint8_t s=0;out[0]=out[1]=0x5a;out[2]=type;out[3]=len;memcpy(out+4,data,len);for(i=0;i<4U+len;i++)s+=out[i];out[4+len]=s;}
static int feed(ImuParser *p,const uint8_t *b,unsigned n,uint32_t ms){int count=0;unsigned i;for(i=0;i<n;i++)count+=Imu_Byte(p,b[i],ms);return count;}
int main(void){
 ImuParser p;uint8_t b[32],noise[3]={9,8,7};unsigned i;
 const uint8_t real[]={0x5a,0x5a,0x45,6,0xfa,0x88,0xfb,0x81,0xdd,0xe3,0xbd};
 Imu_Init(&p);CHECK(!Imu_Fresh(&p,0,200));
 CHECK(feed(&p,real,sizeof(real),100)==1);NEAR(p.euler_deg[0],-14,.0001);NEAR(p.euler_deg[1],-11.51,.0001);NEAR(p.euler_deg[2],-87.33,.0001);
 CHECK(p.good==1&&p.euler_seq==1);CHECK(Imu_Fresh(&p,300,200));CHECK(!Imu_Fresh(&p,301,200));
 for(i=0;i<=sizeof(real);i++){Imu_Init(&p);feed(&p,real,i,10);feed(&p,real+i,sizeof(real)-i,11);CHECK(p.good==1);}
 Imu_Init(&p);feed(&p,noise,3,0);Imu_Byte(&p,0x5a,1);CHECK(feed(&p,real,sizeof(real),2)==1);CHECK(p.good==1);
 memcpy(b,real,sizeof(real));b[10]^=1;Imu_Init(&p);CHECK(!feed(&p,b,sizeof(real),0));CHECK(feed(&p,real,sizeof(real),2)==1);CHECK(p.bad>0);
 Imu_Init(&p);feed(&p,real,5,0);CHECK(!feed(&p,real+5,6,100));CHECK(feed(&p,real,11,101)==1);
 {uint8_t q[]={0x27,0x10,0,0,0,0,0,0};frame(0x65,q,8,b);CHECK(feed(&p,b,13,110)==1);NEAR(p.quaternion[0],1,.00001);CHECK(p.euler_seq==1);}
 {uint8_t raw[]={0x80,0,0x7f,0xff,0xff,0xff};frame(0x25,raw,6,b);CHECK(feed(&p,b,11,120)==1);CHECK(p.gyro_raw[0]==-32768&&p.gyro_raw[1]==32767&&p.gyro_raw[2]==-1);}
 Imu_Init(&p);memcpy(b,real,11);b[3]=255;feed(&p,b,11,0);CHECK(feed(&p,real,11,1)==1);
 Imu_Init(&p);feed(&p,real,11,0xfffffff0U);CHECK(Imu_Fresh(&p,20,40));CHECK(!Imu_Fresh(&p,50,40));
 for(i=0;i<3000;i++){Imu_Byte(&p,(uint8_t)(i*37U),100+i);CHECK(p.size<=sizeof(p.data));}CHECK(feed(&p,real,11,4000)==1);
 DONE();
}
