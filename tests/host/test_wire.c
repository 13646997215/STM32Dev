#include "test.h"
int main(void){WireFrame f={0},r={0};WireParser p={0};uint8_t b[150],bad[150];size_t n,i,j;int found;
 CHECK(Wire_Crc((const uint8_t*)"123456789",9)==0x29b1);
 f.type=CMD_VELOCITY;f.sequence=0xabcd;f.length=8;Wire_PutFloat(f.payload,-.12f);Wire_PutFloat(f.payload+4,.3f);
 n=Wire_Encode(&f,b,sizeof(b));CHECK(n==18);CHECK(!Wire_Encode(&f,b,17));NEAR(Wire_GetFloat(f.payload),-.12,1e-6);if(n!=18){DONE();}
 for(i=0;i<n;i++)Wire_Byte(&p,b[i],100,&r);
 CHECK(p.good==1&&r.sequence==f.sequence&&r.type==CMD_VELOCITY&&r.length==8);NEAR(Wire_GetFloat(r.payload+4),.3,1e-6);
 for(j=0;j<=n;j++){memset(&p,0,sizeof(p));found=0;for(i=0;i<j;i++)found+=Wire_Byte(&p,b[i],0,&r);for(i=j;i<n;i++)found+=Wire_Byte(&p,b[i],1,&r);CHECK(found==1);}
 memcpy(bad,b,n);bad[n-1]^=1;memset(&p,0,sizeof(p));for(i=0;i<n;i++)Wire_Byte(&p,bad[i],0,&r);CHECK(!p.good);for(i=0;i<n;i++)Wire_Byte(&p,b[i],1,&r);CHECK(p.good==1&&p.bad>0);
 memset(&p,0,sizeof(p));Wire_Byte(&p,0xa7,0,&r);for(i=0;i<n;i++)Wire_Byte(&p,b[i],1,&r);CHECK(p.good==1);
 memset(&p,0,sizeof(p));for(i=0;i<5;i++)Wire_Byte(&p,b[i],0,&r);for(i=5;i<n;i++)Wire_Byte(&p,b[i],500,&r);CHECK(!p.good);for(i=0;i<n;i++)Wire_Byte(&p,b[i],501,&r);CHECK(p.good==1);
 Wire_PutU32(b,0xfedcba98U);CHECK(Wire_GetU32(b)==0xfedcba98U);
 f.length=97;CHECK(!Wire_Encode(&f,b,sizeof(b)));
 DONE();}
