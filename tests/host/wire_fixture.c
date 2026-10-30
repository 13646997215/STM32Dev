#include "course.h"
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv)
{
    WireFrame f={0};WireParser p={0};uint8_t bytes[106];size_t i,n;unsigned b;
    if(argc==2){
        n=strlen(argv[1]);if(n>212||n%2)return 2;
        for(i=0;i<n;i+=2){if(sscanf(argv[1]+i,"%2x",&b)!=1)return 2;
            if(Wire_Byte(&p,(uint8_t)b,0,&f))printf("%u %u %.3f %.3f\n",f.type,f.sequence,(double)Wire_GetFloat(f.payload),(double)Wire_GetFloat(f.payload+4));}
        return p.good?0:1;
    }
    f.type=CMD_VELOCITY;f.sequence=0x1234;f.length=8;Wire_PutFloat(f.payload,1.25f);Wire_PutFloat(f.payload+4,-2.5f);
    n=Wire_Encode(&f,bytes,sizeof(bytes));for(i=0;i<n;i++)printf("%02x",bytes[i]);puts("");return 0;
}
