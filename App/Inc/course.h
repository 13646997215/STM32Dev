#ifndef COURSE_H
#define COURSE_H
#include <stdint.h>
#include <stddef.h>

#define COURSE_VERSION "1.1.0-task1-pitch"
#define COURSE_PI 3.14159265358979323846f
#define COURSE_MAX_PAYLOAD 96U
#define CAL_GEOMETRY 1U
#define CAL_SCALE 2U
#define CAL_SIGNS 4U
#define CAL_IMU 8U
#define CAL_SPEED 16U
#define CAL_POWER 32U
#define CAL_ENTRY 64U
#define CAL_BALANCE 128U
#define CAL_MANUAL (CAL_SCALE|CAL_SIGNS|CAL_SPEED|CAL_POWER)
#define CAL_BOARD 255U
#define CAL_PATH (CAL_GEOMETRY|CAL_SCALE|CAL_SIGNS|CAL_IMU|CAL_SPEED|CAL_POWER)

typedef struct {
    float wheel_diameter_m, track_m, length_m, width_m;
    float ppr[2], wheel_scale[2];
    float max_duty, deadband[2], kp[2], ki[2], max_rpm, rpm_slew;
    float max_v, max_w, min_v, heading_kp;
    float pitch_zero_deg, board_heading_rad, yaw_sign, pitch_sign;
    float balance_kp, balance_ki, balance_kd, balance_sign;
    float stable_deg, stable_rate_dps, disturb_deg, max_tilt_deg;
    float tip_delta_deg, seek_min_m, seek_max_m, climb_end_m;
    float start_x_m, start_y_m, return_x_m;
    float path_a_m, path_b_m, dock_x_m, dock_y_m;
    float pos_tol_m, yaw_tol_rad, boundary_margin_m, min_duty;
    uint32_t cal_flags, imu_timeout_ms, fg_timeout_ms, reversal_quiet_ms;
    uint8_t pitch_axis, forward_release[2], start_requires_ramp, fg_fault_en;
} RobotConfig;
void Config_Default(RobotConfig *c);
int Config_Valid(const RobotConfig *c);
int Config_Set(RobotConfig *c, uint16_t key, float value);
float Course_Wrap(float rad);
float Course_Clamp(float x, float lo, float hi);
int Course_Finite(float x);
void Course_Mix(float v, float w, float track, float limit, float out[2]);
int Course_Inside(float x, float y, float yaw, const RobotConfig *c);

typedef struct {
    uint8_t data[24], size;
    uint32_t good, bad, dropped, last_byte_ms, euler_ms, euler_seq;
    uint32_t gyro_ms, accel_ms, mag_ms, quat_ms;
    float euler_deg[3], quaternion[4];
    int16_t gyro_raw[3], accel_raw[3], mag_raw[3];
    uint8_t status[8], status_len, status_type;
    uint8_t have_euler;
} ImuParser;
void Imu_Init(ImuParser *p);
int Imu_Byte(ImuParser *p, uint8_t b, uint32_t now_ms);
int Imu_Fresh(const ImuParser *p, uint32_t now_ms, uint32_t timeout);

typedef struct { uint8_t type; uint16_t sequence, length; uint8_t payload[COURSE_MAX_PAYLOAD]; } WireFrame;
typedef struct { uint8_t data[COURSE_MAX_PAYLOAD+10]; uint16_t size; uint32_t good,bad,last_ms; } WireParser;
enum { CMD_PING=1,CMD_STATUS,CMD_CONFIG,CMD_ARM,CMD_VELOCITY,CMD_START,CMD_STOP,CMD_CLEAR,CMD_IMU_QUERY,CMD_DUTY,
       CMD_TASK1_BENCH_START,CMD_BENCH_KEEPALIVE,
       MSG_ACK=0x80,MSG_ERROR=0x81,MSG_TELEMETRY=0x90,MSG_IMU_RAW=0x91 };
uint16_t Wire_Crc(const uint8_t *p, size_t n);
size_t Wire_Encode(const WireFrame *f, uint8_t *out, size_t capacity);
int Wire_Byte(WireParser *p,uint8_t b,uint32_t now_ms,WireFrame *out);
void Wire_PutFloat(uint8_t *p,float x);
float Wire_GetFloat(const uint8_t *p);
void Wire_PutU32(uint8_t *p,uint32_t v);
uint32_t Wire_GetU32(const uint8_t *p);

typedef struct { uint32_t total,last_us,period_us,glitches; uint8_t have_edge,have_period; } FgCapture;
void Fg_Edge(FgCapture *f,uint32_t now_us);
typedef struct { uint32_t previous_total,previous_us; float hz; uint8_t initialized,fresh; } SpeedEstimate;
void Speed_Update(SpeedEstimate *s,const FgCapture *f,uint32_t now_us,uint32_t timeout_us);
typedef struct { float integral; } PiController;
float Pi_Update(PiController *p,float error,float dt,float kp,float ki,float feedforward,float limit);
enum { MOTOR_DISABLED,MOTOR_READY,MOTOR_RUN,MOTOR_WAIT,MOTOR_FAULT };
typedef struct {
    PiController pi;
    float ramp_rpm,duty;
    uint32_t phase_ms,run_ms;
    int8_t direction;
    uint8_t phase,direction_assumed;
} MotorController;
void Motor_Init(MotorController *m);
void Motor_Update(MotorController *m,float target_rpm,float measured_abs_rpm,
                  const FgCapture *fg,uint32_t now_ms,uint32_t now_us,float dt,
                  int enabled,const RobotConfig *c,unsigned index);

typedef struct { float x,y,yaw,distance; uint32_t last_count[2]; uint8_t initialized,valid; } Pose;
void Pose_Reset(Pose *p,float x,float y,float yaw,const FgCapture fg[2]);
void Pose_Update(Pose *p,const FgCapture fg[2],const int8_t direction[2],
                 const uint8_t trusted[2],float yaw,const RobotConfig *c);
typedef struct { float v,w; uint8_t reached; } MotionOutput;
MotionOutput Motion_Goto(const Pose *p,float x,float y,float final_yaw,int reverse,const RobotConfig *c);
void Motion_WheelPoint(const Pose *p,float rx,float ry,float *x,float *y);
typedef struct {
    float integral,last_pitch,rate,v;
    uint32_t last_seq,last_ms,stable_since_ms;
    uint8_t initialized,stable_tracking,stable;
} BalanceController;
void Balance_Reset(BalanceController *b);
float Balance_Update(BalanceController *b,float pitch,uint32_t sample_seq,uint32_t now_ms,int valid,const RobotConfig *c);
/* Task1: configured Pitch zero, sign-preserving P control; Yaw is not an input. */
float Task1_Update(BalanceController *b,float pitch,uint32_t sample_seq,uint32_t now_ms,int valid,const RobotConfig *c);

/* Alignment damping: rate feedback on the heading error, in seconds. */
#define ALIGN_DAMP 0.35f

enum { PROFILE_A=1,PROFILE_ENTRY,PROFILE_BALANCE_RETURN,PROFILE_WEIGHT_RETURN,PROFILE_REWEIGHT,PROFILE_PRELIM,PROFILE_RECTANGLE };
enum { MS_IDLE,MS_ALIGN,MS_APPROACH,MS_CLIMB,MS_SEEK,MS_BALANCE,MS_HOLD,MS_RETURN,MS_ARRIVED,MS_PATH,MS_DOCK,MS_DONE,MS_FAULT };
enum { FAULT_NONE,FAULT_CONFIG,FAULT_IMU,FAULT_FEEDBACK,FAULT_TIMING,FAULT_BOUNDARY,FAULT_TIMEOUT,FAULT_TILT,FAULT_MOTOR,FAULT_UART,FAULT_STOPPED,FAULT_POSE };
typedef struct {
    uint32_t now_ms,imu_seq;
    Pose pose;
    float pitch_deg,yaw_rad;
    uint8_t imu_valid,pose_valid;
} MissionInput;
typedef struct {
    BalanceController balance;
    uint32_t started_ms,state_ms,trigger_ms,return_ms,balance_ms;
    float origin_x,origin_y,heading,initial_pitch,seek_start_x;
    float align_ref;int8_t align_sign,align_dir;
    float align_prev;uint32_t align_prev_ms;
    uint8_t profile,state,previous_state,fault,path_index,tip_tracking,ramp_seen;
    uint8_t balance_achieved,weight_events;
    uint32_t warnings;
    float v,w;
} Mission;
int Mission_Start(Mission *m,uint8_t profile,const MissionInput *in,const RobotConfig *c);
void Mission_Update(Mission *m,const MissionInput *in,const RobotConfig *c);
int Mission_TipDetected(Mission *m,float pitch,int ready,uint32_t now,const RobotConfig *c);

enum { MODE_DIAGNOSTIC,MODE_MANUAL_SPEED,MODE_MANUAL_DUTY,MODE_AUTO,MODE_BENCH_TASK1 };
enum { RESULT_OK,RESULT_FORMAT,RESULT_RANGE,RESULT_LOCKED,RESULT_NOT_READY,RESULT_DUPLICATE,RESULT_FAULT };
typedef struct {
    RobotConfig config;
    ImuParser imu;
    Mission mission;
    MotorController motor[2];
    SpeedEstimate speed[2];
    Pose pose;
    uint32_t last_control_ms,last_cmd_ms,frames,commands_rejected;
    uint16_t last_start_seq;
    uint16_t bench_sequence;
    float manual[2],pwm[2],pitch,yaw,last_raw_yaw,continuous_yaw;
    uint32_t last_yaw_seq,warnings;
    uint8_t armed,mode,fault,have_control,have_yaw,have_start_seq;
    uint8_t have_bench_sequence;
    int8_t direction[2];
    uint8_t direction_known[2]; /* Command-based assumption, retained while coasting. */
} CourseApp;
void App_Init(CourseApp *a);
uint8_t App_Command(CourseApp *a,const WireFrame *f,uint32_t now_ms);
/* One short KEY press runs one unified round: reference capture, then the task
   1-6 flow. A 2 s hold interrupts or clears a latched fault. */
enum { KEY_NONE, KEY_START, KEY_STOP };
uint8_t Key_Event(uint32_t held_ms,int released,uint8_t state,int armed);
void App_StartUnified(CourseApp *a,uint32_t now_ms);
void Config_ApplyMeasured(RobotConfig *c);
void App_Step(CourseApp *a,const FgCapture fg[2],uint32_t now_ms,uint32_t now_us);
void App_Stop(CourseApp *a,uint8_t fault);
size_t App_Telemetry(const CourseApp *a,uint32_t now_ms,uint8_t *out,size_t capacity);
#endif
