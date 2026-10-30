#include "test.h"
int main(void){RobotConfig c;BalanceController b;float v,last;unsigned i;
 Config_Default(&c);Balance_Reset(&b);v=Balance_Update(&b,3,1,0,1,&c);CHECK(v>0&&v<=c.max_v);
 last=v;v=Balance_Update(&b,-3,1,20,1,&c);NEAR(v,last,1e-6);CHECK(b.last_pitch==3);
 v=Balance_Update(&b,3,2,20,0,&c);CHECK(v==0&&!b.stable);
 Balance_Reset(&b);for(i=0;i<=500;i++)Balance_Update(&b,0,i+1,20*i,1,&c);CHECK(b.stable);
 Balance_Update(&b,2,502,10020,1,&c);CHECK(!b.stable&&!b.stable_tracking);
 for(i=0;i<500;i++)Balance_Update(&b,0,503+i,10040+20*i,1,&c);CHECK(!b.stable);
 /* The derivative filter needs to settle before the new 10 s interval. */
 CHECK(b.stable_tracking);CHECK((uint32_t)(20020-b.stable_since_ms)<10000U);
 for(i=0;i<20;i++)Balance_Update(&b,0,1004+i,20040+20*i,1,&c);CHECK(b.stable);
 Balance_Update(&b,NAN,1025,20440,1,&c);CHECK(!b.stable&&b.v==0);
 Balance_Reset(&b);v=Balance_Update(&b,-3,1,0,1,&c);CHECK(v<0);
 /* Balanced but still swinging: D uses the same sign as the outside-band PID
    and the hold timer is interrupted. Mechanical damping still requires field tuning. */
 Balance_Reset(&b);Balance_Update(&b,-0.4f,1,0,1,&c);
 v=Balance_Update(&b,0.4f,2,20,1,&c);
 CHECK(fabsf(b.rate)>8.0f);CHECK(v>0);
 Balance_Reset(&b);Balance_Update(&b,0.4f,1,0,1,&c);
 v=Balance_Update(&b,-0.4f,2,20,1,&c);
 CHECK(b.rate< -8.0f);CHECK(v<0);
 /* Balanced and quiet: stop and let the hold timer run. */
 Balance_Reset(&b);Balance_Update(&b,0.1f,1,0,1,&c);
 CHECK(Balance_Update(&b,0.2f,2,20,1,&c)==0);
 CHECK(b.stable_tracking);
 /* A long positive tilt must not keep pushing positive after a sign reversal. */
 Config_ApplyMeasured(&c);c.balance_ki=.001f;Balance_Reset(&b);
 for(i=0;i<=1500;i++)Balance_Update(&b,4.5f,i+1,20*i,1,&c);
 CHECK(b.integral>.01f);
 for(i=0;i<=50;i++)v=Balance_Update(&b,-1.1f,1502+i,30020+20*i,1,&c);
 CHECK(v<0);
 DONE();}
