#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "ramp_contorller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 5

#define DM_P_MIN     (-12.5f)
#define DM_P_MAX     12.5f
#define DM_V_MIN     (-30.0f)
#define DM_V_MAX     30.0f
#define DM_T_MIN     (-10.0f)
#define DM_T_MAX     10.0f
#define DM_KP_MIN    0.0f
#define DM_KP_MAX    500.0f
#define DM_KD_MIN    0.0f
#define DM_KD_MAX    5.0f

typedef struct
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float angle_single_round;
    float total_angle;
    float half_angle;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
} DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_mit;    // MIT模式下的位置值
    uint16_t velocity_mit;    // MIT模式下的速度值
    uint16_t position_torque; // MIT力控模式下的位置值
    uint16_t velocity_torque; // MIT力控模式下的速度值
    float position_sp;        // 位置速度模式下的位置值
    float velocity_sp;        // 位置速度模式下的速度值
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

typedef struct
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller; // 电机设置
    float pid_out;

    PID_Instance torque_PID;
    PID_Instance speed_PID;
    PID_Instance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    float speed_ref;   // 位置速度模式下的速度参考
    /* MIT模式下PID */ // T_kef = mit_kp * (p_des - θ_m) + mit_kd * (v_des - dθ) + t_ff
    float mit_kp;
    float mit_kd; // ! kd = 0不能在kd = 0时，否则会出现震荡甚至失控 ！！！
    RampController_Instance angle_ramp;
    DMMotor_Controll_Type_e control_type;
    Motor_Working_Type_e stop_flag;
    CAN_Instance *motor_can_instace;
    Daemon_Instance *motor_daemon;
    uint32_t lost_cnt;
} DM_MotorInstance; // 达妙电机实例

typedef enum {
    DM_CMD_MOTOR_MODE    = 0xfc, // 使能,会响应指令
    DM_CMD_RESET_MODE    = 0xfd, // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR   = 0xfb  // 清除电机过热错误
} DMMotor_Mode_e;

DM_MotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DM_MotorInstance *motor, float ref);
void DMMotorSetSpeedRef(DM_MotorInstance *motor, float ref);

void DMMotorOuterLoop(DM_MotorInstance *motor, Closeloop_Type_e closeloop_type);

void DMMotorEnable(DM_MotorInstance *motor);

void DMMotorStop(DM_MotorInstance *motor);
void DMMotorCaliEncoder(DM_MotorInstance *motor);
void DMMotorClearErr(DM_MotorInstance *motor);
void DMMotorControlInit();
void DMMotorRampEnable(DM_MotorInstance *motor);
void DMMotorRampDisable(DM_MotorInstance *motor);
uint8_t DMMotorPositionCheck(DM_MotorInstance *motor, float ref);
#endif // !DMMOTOR