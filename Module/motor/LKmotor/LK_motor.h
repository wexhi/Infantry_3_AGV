#ifndef LK_MOTOR_H
#define LK_MOTOR_H
#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

// 最多允许4个LK电机使用多电机指令,挂载在一条总线上，若需要多电机支持，请在上位机修改
#define LK_MOTOR_MX_CNT        4

#define CURRENT_SMOOTH_COEF_LK 0.9f  // LK电机电流平滑系数
#define SPEED_SMOOTH_COEF_LK   0.85f // LK电机速度平滑系数
#define REDUCTION_RATIO_DRIVEN 1
#define ECD_ANGLE_COEF_LK      (360.0f / 65536.0f)
#define CURRENT_TORQUE_COEF_LK 0.003645f // 电流设定值转换成扭矩的系数,算出来的设定值除以这个系数就是扭矩值

typedef struct // 4010
{
    uint8_t cmd_mode;         // 电机控制模式，详情查看PDF
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 当前编码器值
    float angle_single_round; // 单圈角度
    float speed_rads;         // speed rad/s
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度,C°

    float total_angle;   // 总角度
    int32_t total_round; // 总圈数

    float feed_dt;
    uint32_t feed_dwt_cnt;
} LKMotor_Measure_t;

typedef struct
{
    LKMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller; // 电机控制器

    LKMotor_Working_Type_e motor_working_type; // 电机工作类型
    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag; // 启停标志

    CAN_Instance *motor_can_ins;

    Daemon_Instance *daemon;

} LKMotor_Instance;

/**
 * @brief 初始化LK电机
 *
 * @param config 电机配置
 * @return LKMotor_Instance* 返回实例指针
 */
LKMotor_Instance *LKMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void LKMotorSetRef(LKMotor_Instance *motor, float ref);

/**
 * @brief 为所有LK电机计算pid/反转/模式控制,并通过bspcan发送电流值(发送CAN报文)
 *
 */
void LKMotorControl();

/**
 * @brief 停止LK电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void LKMotorStop(LKMotor_Instance *motor);

/**
 * @brief 启动LK电机
 *
 * @param motor
 */
void LKMotorEnable(LKMotor_Instance *motor);

uint8_t LKMotorIsOnline(LKMotor_Instance *motor);

#endif // LK_MOTOR_H