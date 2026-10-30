#include "test.h"
/* Start of one unified round: reference capture, measured calibration install,
   and entry into the alignment state. Everything else is covered by test_mission
   (state machine) and test_key (PA0 semantics). */
int main(void){CourseApp a;
 App_Init(&a);
 /* Without a real attitude sample there is nothing to use as a reference. */
 App_StartUnified(&a,0);
 CHECK(!a.armed&&a.mission.state==MS_IDLE);
 a.imu.have_euler=1;a.imu.euler_deg[0]=0;a.imu.euler_deg[1]=-2.5f;a.imu.euler_deg[2]=35;
 a.imu.euler_ms=0;a.imu.euler_seq=1;a.imu.good=1;
 App_StartUnified(&a,0);
 CHECK(a.armed&&a.mode==MODE_AUTO);
 CHECK(a.mission.state==MS_ALIGN&&a.mission.profile==PROFILE_BALANCE_RETURN);
 /* The current attitude became the zero, so the relative pitch is exactly 0. */
 NEAR(a.config.pitch_zero_deg,-2.5f,1e-4);
 NEAR(a.pitch,0,1e-4);
 /* Measured calibration was installed (boot default claims nothing). */
 CHECK(a.config.cal_flags==CAL_PATH);
 NEAR(a.config.track_m,.195,1e-6);
 NEAR(a.config.deadband[0],.35,1e-6);NEAR(a.config.deadband[1],.35,1e-6);
 NEAR(a.config.min_duty,.35,1e-6);
 NEAR(a.config.balance_kp,.006,1e-6);NEAR(a.config.balance_ki,.0003,1e-7);NEAR(a.config.balance_kd,.012,1e-6);
 NEAR(a.config.board_heading_rad,-1.5708,1e-4);
 CHECK(a.config.forward_release[0]==1&&a.config.forward_release[1]==0);
 CHECK(a.pose.valid==1&&a.pose.x==0&&a.pose.y==0);
 CHECK(a.pwm[0]==0&&a.pwm[1]==0);
 /* A config already loaded over the link must win over the baked values. */
 a.config.cal_flags=CAL_PATH;a.config.max_v=.12f;a.config.track_m=.20f;
 App_StartUnified(&a,0);
 NEAR(a.config.max_v,.12f,1e-6);NEAR(a.config.track_m,.20,1e-6);
 DONE();}
