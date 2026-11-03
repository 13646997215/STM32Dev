#include "runtime.h"
#include "board.h"
#include <string.h>
#include <math.h>
CourseApp g_course;
volatile uint32_t g_runtime_max_us,g_runtime_cycles;
volatile float g_rpm_l,g_rpm_r,g_rpm_diff;
volatile uint32_t g_imu_query_sent,g_imu_query_failed,g_imu_last_query_ms,g_imu_last_command;
static WireParser wire;
static uint32_t telemetry_ms,raw_ms,errors_previous,imu_poll_ms;
static uint32_t key_held,key_last,key_start;static uint8_t key_long_done,key_down;
typedef struct {float pitch_sum,yaw_anchor,yaw_offset_sum,pitch_anchor,roll_anchor;uint32_t count,seq,first_ms,last_ms;uint8_t failed;} SceneCapture;
static SceneCapture scene;
static uint16_t tx_seq;
static int imu_query(unsigned selector,uint32_t now)
{
    static const uint8_t single_read[]={0xC5,0xE5,0x95};
    uint8_t cmd[3];
    if(selector>=sizeof(single_read))return 0;
    cmd[0]=0xA5;cmd[1]=single_read[selector];cmd[2]=(uint8_t)(cmd[0]+cmd[1]);
    if(!Board_Write(0,cmd,sizeof(cmd))){g_imu_query_failed++;return 0;}
    g_imu_query_sent++;g_imu_last_command=cmd[1];g_imu_last_query_ms=now;
    if(selector==2)imu_poll_ms=now;
    return 1;
}
static void send_frame(WireFrame *f)
{
    uint8_t bytes[COURSE_MAX_PAYLOAD+10];size_t n=Wire_Encode(f,bytes,sizeof(bytes));
    if(n)Board_Write(1,bytes,n); /* Queue full drops telemetry; it never blocks control. */
}
void Runtime_Init(void)
{
    Board_Init();Board_Stop();App_Init(&g_course);memset(&wire,0,sizeof(wire));
    telemetry_ms=raw_ms=imu_poll_ms=key_last=key_start=0;key_held=0;key_long_done=key_down=0;memset(&scene,0,sizeof(scene));errors_previous=Board_Errors();tx_seq=0;
    g_imu_query_sent=g_imu_query_failed=g_imu_last_query_ms=g_imu_last_command=0;
}
void Runtime_Tick(uint32_t now)
{
    uint8_t b;unsigned i;WireFrame f,reply;FgCapture fg[2];uint32_t started=Board_Micros(),errors;
    for(i=0;i<256&&Board_Read(0,&b);i++)Imu_Byte(&g_course.imu,b,now);
    for(i=0;i<256&&Board_Read(1,&b);i++)if(Wire_Byte(&wire,b,now,&f)){
        uint8_t result=App_Command(&g_course,&f,now);
        if(result!=RESULT_OK&&result!=RESULT_DUPLICATE)g_course.commands_rejected++;
        if(f.type==CMD_STOP&&result==RESULT_OK)Board_Stop();
        if(f.type==CMD_IMU_QUERY&&result==RESULT_OK&&!imu_query(f.payload[0],now))result=RESULT_NOT_READY;
        memset(&reply,0,sizeof(reply));reply.type=result==RESULT_OK||result==RESULT_DUPLICATE?MSG_ACK:MSG_ERROR;
        reply.sequence=f.sequence;reply.length=3;reply.payload[0]=f.type;reply.payload[1]=result;reply.payload[2]=g_course.fault;send_frame(&reply);
    }
    /* Deterministic 50 Hz acquisition. Single reads never toggle a running stream.
       One bounded attempt per tick; real received frames alone establish freshness. */
    if((uint32_t)(now-imu_poll_ms)>=20U){imu_poll_ms=now;imu_query(2,now);}
    /* In idle, a 2 s press captures a stationary reference; during a run it stops. */
    if(Board_Key()){
        if(!key_down){key_down=1;key_start=now;key_held=0;memset(&scene,0,sizeof(scene));scene.seq=g_course.imu.euler_seq;}
        key_held=now-key_start;
        if(!key_long_done&&!g_course.armed&&!g_course.fault&&g_course.mode==MODE_DIAGNOSTIC&&
           g_course.imu.have_euler&&scene.seq!=g_course.imu.euler_seq){
            float pitch=g_course.imu.euler_deg[g_course.config.pitch_axis],yaw=g_course.imu.euler_deg[2],roll=g_course.imu.euler_deg[0],offset;
            scene.seq=g_course.imu.euler_seq;
            if(!Course_Finite(pitch)||!Course_Finite(yaw)||!Course_Finite(roll))scene.failed=1;
            else{
                if(!scene.count){scene.first_ms=now;scene.pitch_anchor=pitch;scene.yaw_anchor=yaw;scene.roll_anchor=roll;}
                else if((uint32_t)(now-scene.last_ms)>120U)scene.failed=1;
                offset=Course_Wrap((yaw-scene.yaw_anchor)*COURSE_PI/180.0f)*180.0f/COURSE_PI;
                if(fabsf(pitch-scene.pitch_anchor)>2.0f||fabsf(roll-scene.roll_anchor)>2.0f||fabsf(offset)>3.0f)scene.failed=1;
                scene.pitch_sum+=pitch;scene.yaw_offset_sum+=offset;scene.last_ms=now;scene.count++;
            }
        }
        if(!key_long_done&&key_held>=2000U){key_long_done=1;
            if(!g_course.armed&&!g_course.fault&&g_course.mode==MODE_DIAGNOSTIC){
                g_course.scene_valid=0;
                if(!scene.failed&&scene.count>=80U&&(uint32_t)(scene.last_ms-scene.first_ms)>=1800U&&
                   (uint32_t)(now-scene.last_ms)<=120U&&Imu_Fresh(&g_course.imu,now,g_course.config.imu_timeout_ms)){
                    g_course.scene_pitch_deg=scene.pitch_sum/(float)scene.count;
                    g_course.scene_yaw_deg=Course_Wrap((scene.yaw_anchor+scene.yaw_offset_sum/(float)scene.count)*COURSE_PI/180.0f)*180.0f/COURSE_PI;
                    if(!g_course.config.cal_flags)Config_ApplyMeasured(&g_course.config);
                    g_course.config.pitch_zero_deg=g_course.scene_pitch_deg;
                    g_course.config.board_heading_rad=Course_Wrap(g_course.scene_yaw_deg*COURSE_PI/180.0f*g_course.config.yaw_sign);
                    g_course.scene_valid=1;
                }
            }else if(Key_Event(key_held,0,g_course.mission.state,g_course.armed)==KEY_STOP){App_Stop(&g_course,0);Board_Stop();}}
    }else{
        if(key_down&&key_held&&!key_long_done&&Key_Event(key_held,1,g_course.mission.state,g_course.armed)==KEY_START)
            App_StartUnified(&g_course,now);
        key_held=0;key_long_done=key_down=0;
    }
    key_last=now;
    errors=Board_Errors();if(errors!=errors_previous){if(g_course.armed&&g_course.mode==MODE_AUTO)g_course.warnings|=64U;else{App_Stop(&g_course,FAULT_UART);Board_Stop();}errors_previous=errors;}
    Board_Snapshot(fg);App_Step(&g_course,fg,now,Board_Micros());
    /* Directly readable symmetry probe, so the two wheel speeds never have to be
       compared by eye. Positive diff means the left wheel is the faster one. */
    g_rpm_l=(g_course.config.ppr[0]>0)?g_course.speed[0].hz*60.0f/g_course.config.ppr[0]:0;
    g_rpm_r=(g_course.config.ppr[1]>0)?g_course.speed[1].hz*60.0f/g_course.config.ppr[1]:0;
    g_rpm_diff=g_rpm_l-g_rpm_r;
    Board_Apply(g_course.pwm,g_course.direction,&g_course.config);
    /* Preserve the original balance / return / fault indication. */
    Board_Led(g_course.fault?((now/150U)&1U):
        (g_course.mission.state==MS_RETURN?((now/250U)&1U):
        (!g_course.scene_valid||g_course.mission.balance.stable_tracking||g_course.mission.state==MS_ARRIVED||g_course.mission.state==MS_DONE)));
    if((uint32_t)(now-telemetry_ms)>=50U){
        telemetry_ms=now;memset(&f,0,sizeof(f));f.type=MSG_TELEMETRY;f.sequence=tx_seq++;
        f.length=(uint16_t)App_Telemetry(&g_course,now,f.payload,sizeof(f.payload));send_frame(&f);
    }
    if((uint32_t)(now-raw_ms)>=100U){
        raw_ms=now;memset(&f,0,sizeof(f));f.type=MSG_IMU_RAW;f.sequence=tx_seq++;f.length=66;
        for(i=0;i<3;i++){Wire_PutFloat(f.payload+4*i,g_course.imu.euler_deg[i]);
            f.payload[12+2*i]=(uint8_t)g_course.imu.accel_raw[i];f.payload[13+2*i]=(uint8_t)((uint16_t)g_course.imu.accel_raw[i]>>8);
            f.payload[18+2*i]=(uint8_t)g_course.imu.gyro_raw[i];f.payload[19+2*i]=(uint8_t)((uint16_t)g_course.imu.gyro_raw[i]>>8);
            f.payload[24+2*i]=(uint8_t)g_course.imu.mag_raw[i];f.payload[25+2*i]=(uint8_t)((uint16_t)g_course.imu.mag_raw[i]>>8);}
        for(i=0;i<4;i++)Wire_PutFloat(f.payload+30+4*i,g_course.imu.quaternion[i]);
        Wire_PutU32(f.payload+46,g_course.imu.accel_ms);Wire_PutU32(f.payload+50,g_course.imu.gyro_ms);
        Wire_PutU32(f.payload+54,g_course.imu.mag_ms);Wire_PutU32(f.payload+58,g_course.imu.quat_ms);
        Wire_PutU32(f.payload+62,g_course.imu.euler_seq);send_frame(&f);
    }
    g_runtime_cycles++;started=Board_Micros()-started;if(started>g_runtime_max_us)g_runtime_max_us=started;
}
