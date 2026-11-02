#include "test.h"
int main(void){RobotConfig c;FgCapture f={0};SpeedEstimate s={0};PiController pi={0};MotorController m;unsigned i;float x;
 Config_Default(&c);Speed_Update(&s,&f,0,250000);CHECK(!s.fresh&&s.hz==0);
 Fg_Edge(&f,0xfffffff0U);CHECK(f.total==1&&!f.have_period);Fg_Edge(&f,984);CHECK(f.total==2&&f.period_us==1000);
 Fg_Edge(&f,1000);CHECK(f.total==2&&f.glitches==1);Fg_Edge(&f,1984);CHECK(f.total==3&&f.period_us==1000);
 Speed_Update(&s,&f,1984,250000);NEAR(s.hz,1000,.01);CHECK(s.fresh);
 Speed_Update(&s,&f,252000,250000);CHECK(!s.fresh&&s.hz==0);
 for(i=0;i<1000;i++)x=Pi_Update(&pi,100,.01f,1,1,.4f,.55f);
 NEAR(x,.55,1e-6);NEAR(pi.integral,0,1e-6);
 x=Pi_Update(&pi,-.1f,.01f,.1f,1,.4f,.55f);CHECK(x<.4f&&pi.integral<0);
 CHECK(Pi_Update(&pi,1,NAN,1,1,0,1)==0);CHECK(Pi_Update(&pi,NAN,.01f,1,1,0,1)==0);
 memset(&f,0,sizeof(f));Motor_Init(&m);CHECK(m.phase==MOTOR_DISABLED);
 Motor_Update(&m,20,0,&f,0,0,.01f,0,&c,0);CHECK(m.duty==0);
 Motor_Update(&m,20,0,&f,10,10000,.01f,1,&c,0);CHECK(m.duty>0&&m.duty<=c.max_duty&&m.direction==1);
 Fg_Edge(&f,490000);Fg_Edge(&f,500000);
 Motor_Update(&m,-20,20,&f,500,500000,.01f,1,&c,0);CHECK(m.phase==MOTOR_WAIT&&m.duty==0&&m.direction==1);
 Motor_Update(&m,-20,20,&f,600,600000,.01f,1,&c,0);CHECK(m.duty==0&&m.direction==1);
 Motor_Update(&m,-20,0,&f,800,800000,.01f,1,&c,0);CHECK(m.direction==-1&&m.duty==0);
 Motor_Update(&m,-20,0,&f,810,810000,.01f,1,&c,0);CHECK(m.duty>0);
 Motor_Update(&m,0,0,&f,820,820000,.01f,1,&c,0);CHECK(m.duty==0&&m.pi.integral==0&&!m.direction_assumed);
 Motor_Init(&m);memset(&f,0,sizeof(f));Motor_Update(&m,20,0,&f,0,0,.01f,1,&c,0);Motor_Update(&m,20,0,&f,1600,1600000,.01f,1,&c,0);CHECK(m.phase==MOTOR_FAULT&&m.duty==0);
 Motor_Init(&m);Motor_Update(&m,10,0,&f,0,0,.1f,1,&c,0);CHECK(m.phase==MOTOR_FAULT&&m.duty==0);
 /* Regression: during an in-place turn the partner wheel's vibration keeps
    producing stray edges, so a fresh edge timestamp alone must not block the
    reversal forever when this wheel is not actually turning. */
 Motor_Init(&m);memset(&f,0,sizeof(f));
 Motor_Update(&m,20,0,&f,0,0,.01f,1,&c,0);CHECK(m.phase==MOTOR_RUN);
 Motor_Update(&m,-20,0,&f,10,10000,.01f,1,&c,0);CHECK(m.phase==MOTOR_WAIT);
 Fg_Edge(&f,300000);                                          /* stray edge this frame */
 Motor_Update(&m,-20,0,&f,150,300000,.01f,1,&c,0);
 CHECK(m.phase==MOTOR_WAIT&&m.direction==1);                  /* still blanking */
 Motor_Update(&m,-20,0,&f,250,320000,.01f,1,&c,0);
 CHECK(m.phase!=MOTOR_WAIT&&m.direction==-1);                 /* low speed => allowed */
 /* A wheel that is really spinning must still be blocked from hot reversal. */
 Motor_Init(&m);memset(&f,0,sizeof(f));
 Motor_Update(&m,20,0,&f,0,0,.01f,1,&c,0);
 Motor_Update(&m,-20,0,&f,10,10000,.01f,1,&c,0);
 Fg_Edge(&f,300000);
 Motor_Update(&m,-20,20,&f,400,300000,.01f,1,&c,0);
 CHECK(m.phase==MOTOR_WAIT&&m.direction==1);
 /* Running wheels are floored at min_duty: a loaded motor does not move below
    its stiction duty, so throttling down into the dead band is useless. */
 {RobotConfig d;Config_Default(&d);d.min_duty=.50f;d.deadband[0]=d.deadband[1]=.50f;d.kp[0]=.001f;d.ki[0]=0;
  Motor_Init(&m);memset(&f,0,sizeof(f));
  Motor_Update(&m,20,0,&f,0,0,.01f,1,&d,0);
  for(i=0;i<50;i++)Motor_Update(&m,20,19.5f,&f,20+i*10,(20+i*10)*1000,.01f,1,&d,0);
  CHECK(m.phase==MOTOR_RUN&&m.duty>=.50f-1e-6f);      /* overspeed error cannot throttle below the floor */
  Motor_Update(&m,0,0,&f,600,600000,.01f,1,&d,0);CHECK(m.duty==0);   /* stop is still stop */}
 /* An unreliable FG line must not stop the round: with fg_fault_en=0 a silent
    channel keeps driving instead of latching MOTOR. */
 {RobotConfig e;Config_Default(&e);e.fg_fault_en=0;
  Motor_Init(&m);memset(&f,0,sizeof(f));
  for(i=0;i<300;i++)Motor_Update(&m,20,0,&f,(uint32_t)(20*i),(uint32_t)(20000*i),.01f,1,&e,0);
  CHECK(m.phase!=MOTOR_FAULT&&m.duty>0);}
 DONE();}
