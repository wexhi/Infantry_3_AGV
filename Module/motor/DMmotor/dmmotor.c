#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"

static uint8_t idx;
static DM_MotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DM_MotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief 解析达妙电机反馈数据
 *
 * @param motor_can 电机CAN实例
 */
static void DMMotorDecode(CAN_Instance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff             = motor_can->rx_buff;
    DM_MotorInstance *motor     = (DM_MotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->id            = rxbuff[0];
    measure->state         = (rxbuff[0] >> 4) & 0xf;
    measure->last_position = measure->position;
    tmp                    = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position      = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    if (measure->position < 0.0f)
        measure->angle_single_round = measure->position / (4 * PI) * 360.0f + 360;
    else
        measure->angle_single_round = measure->position / (4 * PI) * 360.0f;
    if (measure->position - measure->last_position > 2 * PI)
        measure->total_round--;
    else if (measure->position - measure->last_position < -2 * PI)
        measure->total_round++;
    measure->total_angle = measure->total_round * 2 * 360.0f + measure->position / (4 * PI) * 360.0f;

    tmp               = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp             = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos   = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DM_MotorInstance *motor = (DM_MotorInstance *)motor_ptr;
    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorCaliEncoder(DM_MotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

void DMMotorClearErr(DM_MotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    DWT_Delay(0.1);
}

/**
 * @brief 根据电机控制模式配置CAN ID
 *
 * @param motor 电机实例
 * @param config CAN初始化配置
 */
static void DMMotorConfigModel(DM_MotorInstance *motor, CAN_Init_Config_s *config)
{
    switch (motor->control_type) {
        case MOTOR_CONTROL_MIT:
        case MOTOR_CONTROL_MIT_ONLY_TORQUE:
            config->tx_id = config->tx_id;
            break;
        case MOTOR_CONTROL_POSITION_AND_SPEED:
            config->tx_id = 0x100 + config->tx_id;
            break;
        case MOTOR_CONTROL_SPEED:
            config->tx_id = 0x200 + config->tx_id;
            break;
        case MOTOR_CONTROL_E_MIT:
            break;
        default:
            break;
    }

    // 检查是否发生id冲突
    for (size_t i = 0; i < idx; i++) {
        if (dm_motor_instance[i]->motor_can_instace->can_handle == config->can_handle &&
            dm_motor_instance[i]->motor_can_instace->tx_id == config->tx_id) {
            uint16_t can_bus __attribute__((unused)) = config->can_handle == &hcan1 ? 1 : 2;
            while (1) // 当控制模式相同且ID相同时,死循环等待
                ;     // 请检查can id是否冲突
        }
    }
}

/**
 * @brief 达妙电机初始化,所有达妙电机都应该调用此函数进行初始化
 *
 * @param config 电机初始化配置
 * @return DM_MotorInstance* 电机实例
 */
DM_MotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DM_MotorInstance *motor = (DM_MotorInstance *)malloc(sizeof(DM_MotorInstance));
    memset(motor, 0, sizeof(DM_MotorInstance));

    if (!idx)
        DWT_Delay(1);

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->torque_PID, &config->controller_param_init_config.torque_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->mit_kp                   = config->controller_param_init_config.dm_mit_PID.Kp;
    motor->mit_kd                   = config->controller_param_init_config.dm_mit_PID.Kd;
    motor->speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->control_type             = config->control_type;
    RampController_Init(&motor->angle_ramp, &config->controller_param_init_config.angle_ramp);
    if (motor->mit_kp != 0 && motor->mit_kd == 0) {
        while (1) // 进入死循环，请进行安全检查
            ;     // kd = 0不能在kd = 0时，否则会出现震荡甚至失控 ！！！
    }

    motor->control_type = config->control_type;
    DMMotorConfigModel(motor, &config->can_init_config);
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id                  = motor;

    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback     = DMMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DWT_Delay(0.1);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 记得打开
    // 失能，测量数据用
    // DMMotorSetMode(DM_CMD_RESET_MODE, motor);
    // !!! 慎用，懒得焊TXRX线 DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 设置电机位置参考值
 *
 * @param motor 电机实例
 * @param ref 位置参考值
 */
void DMMotorSetRef(DM_MotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

/**
 * @brief 设置电机速度参考值
 *
 * @param motor 电机实例
 * @param ref 速度参考值
 */
void DMMotorSetSpeedRef(DM_MotorInstance *motor, float ref)
{
    motor->speed_ref = ref;
}

/**
 * @brief 电机使能
 *
 * @param motor 电机实例
 */
void DMMotorEnable(DM_MotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 电机停止
 *
 * @param motor 电机实例
 */
void DMMotorStop(DM_MotorInstance *motor) // 不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 设置电机外环控制模式
 *
 * @param motor 电机实例
 * @param type 控制模式
 */
void DMMotorOuterLoop(DM_MotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/**
 * @brief 设置电机斜坡激活
 *
 */
void DMMotorRampEnable(DM_MotorInstance *motor)
{
    motor->motor_settings.angle_ramp_flag = MOTOR_RAMP_ENABLE;
}

/**
 * @brief 设置电机斜坡关闭
 *
 */
void DMMotorRampDisable(DM_MotorInstance *motor)
{
    motor->motor_settings.angle_ramp_flag = MOTOR_RAMP_DISABLE;
}

/**
 * @brief 电机位置检查
 *
 * @param motor 电机实例
 * @param ref 位置参考值
 * @return uint8_t 1为在位置范围内，0为不在位置范围内
 */
uint8_t DMMotorPositionCheck(DM_MotorInstance *motor, float ref)
{
    if (motor->measure.position > ref - 0.08f && motor->measure.position < ref + 0.08f)
        return 1;
    return 0;
}

/**
 * @brief MIT模式下的电机控制
 *
 * @param motor 电机实例
 * @param ref 位置参考值
 * @param send 发送数据结构体
 */
static void DMMotorMITContoroll(DM_MotorInstance *motor, float ref, DMMotor_Send_s *send)
{
    DM_Motor_Measure_s *measure = &motor->measure;
    if (motor->motor_settings.angle_ramp_flag == MOTOR_RAMP_ENABLE) {
        StartRamp(&motor->angle_ramp, measure->position, ref);
        ref = UpdateRamp(&motor->angle_ramp);
    }
    send->position_sp = ref;
    LIMIT_MIN_MAX(ref, DM_P_MIN, DM_P_MAX);
    send->position_mit = float_to_uint(ref, DM_P_MIN, DM_P_MAX, 16);
    send->velocity_mit = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
    if (motor->mit_kp != 0 && motor->mit_kd != 0) {
        send->Kp = float_to_uint(motor->mit_kp, DM_KP_MIN, DM_KP_MAX, 12);
        send->Kd = float_to_uint(motor->mit_kd, DM_KD_MIN, DM_KD_MAX, 12);
    } else {
        send->Kp = float_to_uint(1.f, DM_KP_MIN, DM_KP_MAX, 12);
        send->Kd = float_to_uint(1.f, DM_KD_MIN, DM_KD_MAX, 12);
    }
    send->torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

    if (motor->stop_flag == MOTOR_STOP)
        send->torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

    motor->motor_can_instace->tx_buff[0] = (uint8_t)(send->position_mit >> 8);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)(send->position_mit);
    motor->motor_can_instace->tx_buff[2] = (uint8_t)(send->velocity_mit >> 4);
    motor->motor_can_instace->tx_buff[3] = (uint8_t)(((send->velocity_mit & 0xF) << 4) | (send->Kp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(send->Kp);
    motor->motor_can_instace->tx_buff[5] = (uint8_t)(send->Kd >> 4);
    motor->motor_can_instace->tx_buff[6] = (uint8_t)(((send->Kd & 0xF) << 4) | (send->torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] = (uint8_t)(send->torque_des);
}

/**
 * @brief 力控模式下的电机控制
 *
 * @param motor 电机实例
 * @param ref 力参考值
 * @param send 发送数据结构体
 */
static void DMMotorMITOnlyTorqueContoroll(DM_MotorInstance *motor, float ref, DMMotor_Send_s *send)
{
    float _pid_ref, _set;
    DM_Motor_Measure_s *_measure;
    Motor_Control_Setting_s *_setting;
    Motor_Controller_s *_motor_controller; // 电机控制器
    _measure          = &motor->measure;
    _setting          = &motor->motor_settings;
    _motor_controller = &motor->motor_controller;
    _set              = ref;

    if ((_setting->close_loop_type & ANGLE_LOOP) && (_setting->outer_loop_type & ANGLE_LOOP)) {
        if (_setting->angle_feedback_source == OTHER_FEED) {
            _set = PIDCalculate(&motor->angle_PID, *motor->other_angle_feedback_ptr, _set);
        } else if (_setting->angle_feedback_source == MOTOR_FEED) {
            _set = PIDCalculate(&motor->angle_PID, _measure->total_angle, _set);
        }
    }
    if ((_setting->close_loop_type & SPEED_LOOP) && (_setting->outer_loop_type & (SPEED_LOOP | ANGLE_LOOP))) {
        if (_setting->speed_feedback_source == OTHER_FEED) {
            _set = PIDCalculate(&motor->speed_PID, *motor->other_speed_feedback_ptr, _set);
        } else if (_setting->speed_feedback_source == MOTOR_FEED) {
            _set = PIDCalculate(&motor->speed_PID, _measure->velocity, _set);
        }
        if (_setting->feedforward_flag & SPEED_FEEDFORWARD)
            _set += *_motor_controller->speed_feedforward_ptr;
    }

    if ((_setting->close_loop_type & TORQUE_LOOP) && (_setting->outer_loop_type & (TORQUE_LOOP | SPEED_LOOP | ANGLE_LOOP))) {
        _set = PIDCalculate(&motor->torque_PID, _measure->torque, _set);
    }

    _pid_ref              = _set;
    motor->pid_out        = _set;
    send->position_torque = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
    send->velocity_torque = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
    send->torque_des      = float_to_uint(_pid_ref, DM_T_MIN, DM_T_MAX, 12);
    send->Kp              = 0;
    send->Kd              = 0;
    LIMIT_MIN_MAX(_pid_ref, DM_T_MIN, DM_T_MAX);

    if (motor->stop_flag == MOTOR_STOP)
        send->torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

    motor->motor_can_instace->tx_buff[0] = (uint8_t)(send->position_torque >> 8);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)(send->position_torque);
    motor->motor_can_instace->tx_buff[2] = (uint8_t)(send->velocity_torque >> 4);
    motor->motor_can_instace->tx_buff[3] = (uint8_t)(((send->velocity_torque & 0xF) << 4) | (send->Kp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(send->Kp);
    motor->motor_can_instace->tx_buff[5] = (uint8_t)(send->Kd >> 4);
    motor->motor_can_instace->tx_buff[6] = (uint8_t)(((send->Kd & 0xF) << 4) | (send->torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] = (uint8_t)(send->torque_des);
}

/**
 * @brief 位置速度模式下的电机控制
 *
 * @param motor 电机实例
 * @param pos_ref 位置参考值
 * @param speed_ref 速度参考值
 * @param send 发送数据结构体
 */
static void DMMotorPositonSpeedContoroll(DM_MotorInstance *motor, float pos_ref, float speed_ref, DMMotor_Send_s *send)
{
    DM_Motor_Measure_s *measure = &motor->measure;
    float pos_target;
    pos_target = pos_ref;
    if (motor->stop_flag == MOTOR_STOP)
        send->velocity_sp = 0;
    else
        send->velocity_sp = speed_ref;
    if (motor->motor_settings.angle_ramp_flag == MOTOR_RAMP_ENABLE) {
        StartRamp(&motor->angle_ramp, measure->position, pos_target);
        pos_target = UpdateRamp(&motor->angle_ramp);
    }
    send->position_sp = pos_target;

    memcpy(motor->motor_can_instace->tx_buff, &send->position_sp, 4);
    memcpy(motor->motor_can_instace->tx_buff + 4, &send->velocity_sp, 4);
}

/**
 * @brief 电机控制任务，每个初始化的达妙电机都有一个属于自己的任务
 *
 * @param argument 达妙电机实例
 *
 * @todo 目前实现了MIT模式和位置速度模式和力控模式，后续需要增加其他控制模式请自行添加
 */
static int time = 0;
void DMMotorTask(void const *argument)
{
    float pid_ref, speed_ref;
    DM_MotorInstance *motor = (DM_MotorInstance *)argument;
    // DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    // CANInstance *motor_can = motor->motor_can_instace;
    // uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    while (1) {
        time++;
        // 未使能且电机应当激活时，发送激活指令
        if (time % 200 == 0 && motor->stop_flag == MOTOR_ENALBED) {
            if (motor->measure.state != 0 && motor->measure.state != 1) {
                DMMotorEnable(motor);
                DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
                osDelay(2);
                DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
                osDelay(2);
                return;
            }
            DMMotorEnable(motor);
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
            osDelay(2);
        }

        pid_ref   = motor->pid_ref;
        speed_ref = motor->speed_ref;

        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        switch (motor->control_type) {
            case MOTOR_CONTROL_MIT:
                DMMotorMITContoroll(motor, pid_ref, &motor_send_mailbox);
                break;
            case MOTOR_CONTROL_POSITION_AND_SPEED:
                DMMotorPositonSpeedContoroll(motor, pid_ref, speed_ref, &motor_send_mailbox);
                break;
            case MOTOR_CONTROL_MIT_ONLY_TORQUE:
                DMMotorMITOnlyTorqueContoroll(motor, pid_ref, &motor_send_mailbox);
                break;
            default:
                break;
        }
        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}
/**
 * @brief 达妙电机RTOS任务初始化，
 *  因为每个电机都要延时2ms进行CAN发送，避免堵塞
 *
 */
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++) {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}