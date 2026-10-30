#include "test.h"
int main(void){RobotConfig c,b;float v[2];unsigned i;
 Config_Default(&c);NEAR(c.wheel_diameter_m,.067,1e-7);CHECK(c.cal_flags==0);CHECK(Config_Valid(&c));
 b=c;b.wheel_diameter_m=0;CHECK(!Config_Valid(&b));b=c;b.track_m=NAN;CHECK(!Config_Valid(&b));b=c;b.max_duty=1.1f;CHECK(!Config_Valid(&b));
 /* Full duty must be selectable: the tilt map has to be able to use the whole
    40%..100% range, otherwise a large tilt cannot command a fast correction. */
 b=c;b.max_duty=1.0f;CHECK(Config_Valid(&b));
 {RobotConfig d;Config_Default(&d);CHECK(Config_Set(&d,10,1.0f));NEAR(d.max_duty,1.0,1e-6);CHECK(!Config_Set(&d,10,1.0001f));}
 b=c;b.stable_deg=b.disturb_deg+1;CHECK(!Config_Valid(&b));
 CHECK(!Config_Set(&c,1,NAN));NEAR(c.wheel_diameter_m,.067,1e-7);CHECK(!Config_Set(&c,65535,1));
 CHECK(Config_Set(&c,1,.07f));NEAR(c.wheel_diameter_m,.07,1e-6);CHECK(Config_Set(&c,100,255));CHECK(c.cal_flags==255);
 CHECK(!Config_Set(&c,100,255.5f));CHECK(!Config_Set(&c,100,256));CHECK(!Config_Set(&c,101,2));
 /* +/-pi are equivalent; float multiplication can approach the endpoint from either side. */
 NEAR(fabs(Course_Wrap(COURSE_PI*3)),COURSE_PI,1e-5);NEAR(fabs(Course_Wrap(-COURSE_PI*3)),COURSE_PI,1e-5);
 CHECK(Course_Finite(0)&&!Course_Finite(NAN)&&!Course_Finite(INFINITY));NEAR(Course_Clamp(7,-2,3),3,1e-6);
 Course_Mix(.1,1,.14,.12,v);CHECK(v[1]<=.120001f&&v[0]>0);NEAR(v[0]/v[1],.03/.17,1e-5);
 Course_Mix(0,1,.14,.2,v);NEAR(v[0],-.07,1e-6);NEAR(v[1],.07,1e-6);
 Config_Default(&c);CHECK(Course_Inside(.5,0,0,&c));CHECK(!Course_Inside(.5,.1,0,&c));c.length_m=.29f;CHECK(!Course_Inside(.5,0,COURSE_PI/2,&c));
 for(i=0;i<10000;i++){float x=(float)i*.137f-500;float w=Course_Wrap(x);CHECK(w>=-COURSE_PI&&w<COURSE_PI);}
 /* The measured calibration installed at KEY start must be complete and valid. */
 {RobotConfig m;Config_Default(&m);Config_ApplyMeasured(&m);
  CHECK(Config_Valid(&m));CHECK(m.cal_flags==CAL_PATH);
  NEAR(m.track_m,.195,1e-6);NEAR(m.ppr[0],738,1e-6);NEAR(m.max_duty,1.0,1e-6);
  NEAR(m.deadband[0],.35,1e-6);NEAR(m.deadband[1],.35,1e-6);NEAR(m.min_duty,.35,1e-6);NEAR(m.kp[0],.012,1e-6);NEAR(m.ki[0],.02,1e-6);
  NEAR(m.max_rpm,50,1e-6);NEAR(m.rpm_slew,60,1e-6);
  NEAR(m.max_v,.06,1e-6);NEAR(m.min_v,.015,1e-6);
  NEAR(m.climb_end_m,.5,1e-6);NEAR(m.yaw_tol_rad,.035,1e-4);
  NEAR(m.board_heading_rad,-1.5708,1e-4);
  CHECK(m.yaw_sign==-1&&m.pitch_sign==1&&m.pitch_axis==1);
  CHECK(m.forward_release[0]==1&&m.forward_release[1]==0);
  CHECK(m.fg_fault_en==0);                          /* flaky FG must not stop a round */
  /* 1.0 deg is still inside the course 30 mm / 1600 mm = 1.07 deg criterion. */
  NEAR(m.stable_deg,1.0,1e-6);NEAR(m.stable_rate_dps,3.0,1e-6);
  CHECK(m.stable_deg<1.07f&&m.stable_deg<m.disturb_deg);
  CHECK(Config_Set(&m,45,1.2f));                    /* climb distance must be adjustable */
  NEAR(m.climb_end_m,1.2,1e-6);CHECK(!Config_Set(&m,45,1.6f));}
 DONE();}
