#ifndef COURSE_RUNTIME_H
#define COURSE_RUNTIME_H
#include "course.h"
extern CourseApp g_course;
extern volatile uint32_t g_runtime_max_us,g_runtime_cycles;
/* Wheel speed symmetry probe: watch these three directly instead of comparing
   two numbers by eye. Positive g_rpm_diff means the left wheel is faster. */
extern volatile float g_rpm_l,g_rpm_r,g_rpm_diff;
/* Queued request counts are diagnostics, not proof of a sensor response. */
extern volatile uint32_t g_imu_query_sent,g_imu_query_failed,g_imu_last_query_ms,g_imu_last_command;
void Runtime_Init(void);
void Runtime_Tick(uint32_t now_ms);
#endif
