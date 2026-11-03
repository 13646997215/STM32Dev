#include "test.h"
#include <math.h>

int main(void)
{
    RobotConfig c;
    BalanceController b;
    float near, far, approaching;
    Config_Default(&c); Config_ApplyMeasured(&c);
    c.balance_kp = .1f; /* Exercise the limiter with deliberately high demand. */
    Balance_Reset(&b);
    far = Balance_Update(&b, 5.0f, 1, 20, 1, &c);
    CHECK(fabsf(far) <= .07001f && far > 0);
    Balance_Reset(&b);
    near = Balance_Update(&b, 1.1f, 1, 20, 1, &c);
    CHECK(near > 0 && fabsf(near) <= .027f);
    Balance_Reset(&b);
    Balance_Update(&b, 3.2f, 1, 20, 1, &c);
    approaching = Balance_Update(&b, 3.0f, 2, 40, 1, &c);
    CHECK(fabsf(approaching) <= .035f);
    c.balance_kp = .006f;
    Balance_Reset(&b);
    Balance_Update(&b, 2.0f, 1, 20, 1, &c);
    Balance_Update(&b, 2.0f, 2, 40, 1, &c);
    CHECK(b.integral != 0);
    CHECK(Balance_Update(&b, .5f, 3, 60, 1, &c) == 0);
    Balance_Reset(&b);
    c.balance_kp = .1f;
    CHECK(Balance_Update(&b, -5.0f, 1, 20, 1, &c) >= -.07001f);
    {
        float wheels[2];
        /* Heading correction must not bypass the active wheel speed cap. */
        Course_Mix(.025f, .7f, c.track_m, .025f, wheels);
        CHECK(fabsf(wheels[0]) <= .02501f && fabsf(wheels[1]) <= .02501f);
    }
    DONE();
}
