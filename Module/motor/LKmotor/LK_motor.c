#include "LK_motor.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"

static uint8_t idx;
static LKMotor_Instance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};
static CAN_Instance *sender_instance; // 多电机发送时使用的caninstance(当前保存的是注册的第一个电机的caninstance)
// 目前只考虑单电机模式。 @TODO: 加入多电机模式

void LKMotorStop(LKMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LKMotorEnable(LKMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void LKMotorDecode(CAN_Instance *_instance)
{
    uint8_t *rx_buff           = _instance->rx_buff;
    LKMotor_Instance *motor    = (LKMotor_Instance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    LKMotor_Measure_t *measure = &motor->measure;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = DWT_GetDeltaT(&measure->feed_dwt_cnt);

    measure->cmd_mode = rx_buff[0];

    measure->last_ecd = measure->ecd;
    measure->ecd      = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF_LK) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF_LK * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));

    measure->real_current = (1 - CURRENT_SMOOTH_COEF_LK) * measure->real_current +
                            CURRENT_SMOOTH_COEF_LK * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));

    measure->temperature = rx_buff[1];

    if (measure->ecd - measure->last_ecd > 8191) // -32768 啊？
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -8191)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void LKMotorLostCallback(void *motor_ptr)
{
    LKMotor_Instance *motor __attribute__((unused)) = (LKMotor_Instance *)motor_ptr;
}

LKMotor_Instance *LKMotorInit(Motor_Init_Config_s *config)
{
    LKMotor_Instance *motor = (LKMotor_Instance *)malloc(sizeof(LKMotor_Instance));
    memset(motor, 0, sizeof(LKMotor_Instance));

    motor->motor_settings = config->controller_setting_init_config;

    // 电机的基本设置
    motor->motor_type         = config->motor_type;                     // 电机类型 GM6020/M3508/M2006
    motor->motor_settings     = config->controller_setting_init_config; // 电机控制设置 正反转,闭环类型等
    motor->motor_working_type = config->motor_working_type;
    if (motor->motor_working_type == LK_MULTI_MOTOR) {
        sender_instance        = motor->motor_can_ins;
        sender_instance->tx_id = 0x280; //  修改tx_id为0x280,用于多电机发送,不用管其他LKMotorInstance的tx_id,它们仅作初始化用
    }

    // 电机的PID初始化
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.id                  = motor;
    config->can_init_config.can_module_callback = LKMotorDecode;
    config->can_init_config.rx_id               = 0x140 + config->can_init_config.tx_id;
    config->can_init_config.tx_id               = 0x140 + config->can_init_config.tx_id;
    motor->motor_can_ins                        = CANRegister(&config->can_init_config);

    // DWT_GetDeltaT(&motor->measure.feed_dwt_cnt);

    Daemon_Init_Config_s daemon_config = {
        .callback     = LKMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 5, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    LKMotorEnable(motor);
    lkmotor_instance[idx++] = motor;
    return motor;
}

void LKMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    int32_t set32;
    LKMotor_Instance *motor;
    LKMotor_Measure_t *measure;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller; // 电机控制器

    for (size_t i = 0; i < idx; ++i) {
        motor            = lkmotor_instance[i];
        measure          = &motor->measure;
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        pid_ref          = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP) {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref                         = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            motor_controller->pid_angle_out = pid_ref; // 保存位置环输出
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_rads;
            // 更新pid_ref进入下一个环
            pid_ref                         = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            motor_controller->pid_speed_out = pid_ref; // 保存速度环输出
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
            pid_ref *= -1;
        }

        // 获取最终输出
        motor_controller->pid_out = pid_ref; // 保存电流环输出
        set                       = pid_ref;
        set32                     = pid_ref;

        if (motor->motor_working_type == LK_MULTI_MOTOR) { // 这里随便写的,大概率有BUG,为了兼容多电机命令.后续应该将tx_id以更好的方式表达电机id,单独使用一个CANInstance,而不是用第一个电机的CANInstance
            memcpy(sender_instance->tx_buff + (motor->motor_can_ins->tx_id - 0x280) * 2, &set, sizeof(uint16_t));

            if (motor->stop_flag == MOTOR_STOP) { // 若该电机处于停止状态,直接将发送buff置零
                memset(sender_instance->tx_buff + (motor->motor_can_ins->tx_id - 0x280) * 2, 0, sizeof(uint16_t));
            }
        } else if (motor->motor_working_type == LK_SINGLE_MOTOR) {
            motor->motor_can_ins->tx_buff[0] = 0xA2; // 电机速度控制指令
            memcpy(motor->motor_can_ins->tx_buff + 4, &set32, sizeof(int32_t));

            if (motor->stop_flag == MOTOR_STOP) { // 若该电机处于停止状态,直接将发送buff置零
                memset(motor->motor_can_ins->tx_buff + 4, 0, sizeof(int32_t));
            }
        }
    }

    if (idx) // 如果有电机注册了,不论单电机还是多电机，都应该只发送一次
    {
        motor = lkmotor_instance[0]; // 只观察第一个电机，因为1条总线上要么单电机，要么多电机
        if (motor->motor_working_type == LK_MULTI_MOTOR) {
            CANTransmit(sender_instance, 0.2f);
        }
        if (motor->motor_working_type == LK_SINGLE_MOTOR) {
            CANTransmit(motor->motor_can_ins, 0.2f);
        }
    }
}

void LKMotorSetRef(LKMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

uint8_t LKMotorIsOnline(LKMotor_Instance *motor)
{
    return DaemonIsOnline(motor->daemon);
}