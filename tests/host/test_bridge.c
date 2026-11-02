#include "test.h"

int main(void)
{
    RobotConfig c;
    Mission m = {0};
    MissionInput in = {0};
    unsigned i;
    Config_Default(&c);
    Config_ApplyMeasured(&c);
    in.imu_valid = in.pose_valid = in.pose.valid = 1;
    CHECK(Mission_Start(&m, PROFILE_BALANCE_RETURN, &in, &c));
    in.yaw_rad = .01f; in.now_ms = 20; in.imu_seq++;
    Mission_Update(&m, &in, &c);
    in.now_ms = 600; in.imu_seq++;
    Mission_Update(&m, &in, &c);
    CHECK(m.state == MS_CLIMB);
    /* Slope changes before the minimum approach distance cannot trigger. */
    in.pose.x = .3f; in.pitch_deg = 6.0f; in.now_ms += 20; in.imu_seq++;
    Mission_Update(&m, &in, &c);
    in.pitch_deg = 2.0f;
    for (i = 0; i < 8; i++) {
        in.now_ms += 20; in.imu_seq++; Mission_Update(&m, &in, &c);
        CHECK(m.state == MS_CLIMB);
    }
    in.pose.x = .5f; in.pitch_deg = 4.8f; in.yaw_rad = .05f;
    for (i = 0; i < 40; i++) {
        in.now_ms += 20; in.imu_seq++; Mission_Update(&m, &in, &c);
        CHECK(m.state == MS_CLIMB);
        CHECK(m.v > 0 && m.w < 0);
    }
    /* A single erroneous-looking low sample and a repeated sequence are not
       sufficient to confirm a flip. */
    in.pitch_deg = 2.7f; in.now_ms += 20; in.imu_seq++;
    Mission_Update(&m, &in, &c); CHECK(m.state == MS_CLIMB);
    in.now_ms += 60; Mission_Update(&m, &in, &c); CHECK(m.state == MS_CLIMB);
    in.pitch_deg = 4.8f; in.now_ms += 20; in.imu_seq++;
    Mission_Update(&m, &in, &c); CHECK(m.state == MS_CLIMB);
    /* A rapid sustained change toward level indicates the first flip. */
    for (i = 0; i < 20 && m.state == MS_CLIMB; i++) {
        in.pitch_deg -= .15f; in.now_ms += 20; in.imu_seq++;
        Mission_Update(&m, &in, &c);
    }
    CHECK(m.state == MS_BALANCE);

    /* Leaving an in-place turn: suppress both PWM outputs until both motor
       direction interlocks have completed, then release both in the same tick. */
    {
        CourseApp a;
        FgCapture fg[2] = {{0}};
        int released = 0, waited = 0;
        App_Init(&a); Config_ApplyMeasured(&a.config);
        a.armed = 1; a.mode = MODE_AUTO;
        a.mission.state = MS_ALIGN; a.mission.profile = PROFILE_BALANCE_RETURN;
        a.mission.tip_tracking = 1; a.mission.trigger_ms = 1;
        a.imu.have_euler = 1; a.imu.euler_seq = 1;
        a.imu.euler_deg[2] = 90; a.motor[1].direction = -1;
        for (i = 0; i < 70; i++) {
            uint32_t now = 600U + i * 10U;
            /* Simulate one wheel still coasting after the turn. */
            if (i < 36) { fg[0].have_edge = fg[0].have_period = 1; fg[0].period_us = 1500; fg[0].last_us = now * 1000U; }
            else memset(&fg[0], 0, sizeof(fg[0]));
            a.imu.euler_ms = now; a.imu.euler_seq++;
            App_Step(&a, fg, now, now * 1000U);
            if (a.mission.state != MS_CLIMB) continue;
            if (a.motor[0].phase == MOTOR_WAIT || a.motor[1].phase == MOTOR_WAIT) waited = 1;
            if (a.pwm[0] > 0 || a.pwm[1] > 0) {
                CHECK(a.pwm[0] > 0 && a.pwm[1] > 0);
                NEAR(a.motor[0].ramp_rpm, a.motor[1].ramp_rpm, .001f);
                released = 1; break;
            }
        }
        CHECK(waited && released);
    }
    DONE();
}
