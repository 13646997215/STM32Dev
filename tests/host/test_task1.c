#include "test.h"
/* Task1 = PID on the pitch error, same structure as the course material:
   KP*e + KI*integral + KD*rate, integral and output clamped to max_v.
   With KI=KD=0 the behaviour must stay exactly the previous pure-P mapping. */
int main(void){RobotConfig c;BalanceController b;float v1,v2,v4,v7,vr,vp,vd;unsigned i;
 Config_Default(&c);c.max_v=.10f;c.balance_kp=.015f;c.min_v=.008f;
 c.balance_ki=0;c.balance_kd=0;
 /* --- pure P regression: same numbers as before the PID change --- */
 Balance_Reset(&b);
 v1=Task1_Update(&b,1.5f,1,0,1,&c);
 v2=Task1_Update(&b,2,2,20,1,&c);
 v4=Task1_Update(&b,4,3,40,1,&c);
 v7=Task1_Update(&b,7,4,60,1,&c);
 CHECK(v1>0);CHECK(v2>v1);CHECK(v4>v2);
 CHECK(v4>.04f);
 NEAR(v4,.06f,1e-5);
 NEAR(v7,c.max_v,1e-5);
 vr=Task1_Update(&b,-4,5,80,1,&c);
 NEAR(vr,-v4,1e-6);
 /* --- stop inside tolerance, restart only beyond twice that band --- */
 Balance_Reset(&b);
 CHECK(Task1_Update(&b,.4f,1,0,1,&c)==0);
 CHECK(Task1_Update(&b,.9f,2,20,1,&c)==0);
 CHECK(Task1_Update(&b,1.2f,3,40,1,&c)>0);
 /* --- beyond the tilt guard the command is dropped --- */
 Balance_Reset(&b);
 CHECK(Task1_Update(&b,c.max_tilt_deg+1,1,0,1,&c)==0);
 /* --- integral action: a steady error keeps raising the command --- */
 Balance_Reset(&b);c.balance_ki=.05f;
 v1=Task1_Update(&b,1.2f,1,0,1,&c);
 for(i=0;i<50;i++)v2=Task1_Update(&b,1.2f,(uint32_t)(i+2),(uint32_t)(20*(i+1)),1,&c);
 CHECK(v2>v1+.02f);
 CHECK(v2<=c.max_v);
 /* --- integral is cleared when the controller stops inside the band --- */
 Task1_Update(&b,.2f,60,1020,1,&c);
 CHECK(b.integral==0);
 /* --- derivative damps a rising error --- */
 c.balance_ki=0;c.balance_kd=0;
 Balance_Reset(&b);
 Task1_Update(&b,1.2f,1,0,1,&c);
 vp=Task1_Update(&b,1.7f,2,20,1,&c);
 c.balance_kd=.003f;
 Balance_Reset(&b);
 Task1_Update(&b,1.2f,1,0,1,&c);
 vd=Task1_Update(&b,1.7f,2,20,1,&c);
 CHECK(vd>vp);
 /* --- derivative cuts the command on a falling error, but never reverses it --- */
 Balance_Reset(&b);
 Task1_Update(&b,2.0f,1,0,1,&c);
 vr=Task1_Update(&b,1.5f,2,20,1,&c);
 CHECK(vr>=0);
 CHECK(vr<.0225f);
 DONE();}
