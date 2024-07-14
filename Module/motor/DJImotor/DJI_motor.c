#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"

/* ---------------------------------------- 私有函数声明  ------------------------------------- */
static void MotorSenderGrouping(DJIMotor_Instance *motor, CAN_Init_Config_s *can_config);
static void DecodeDJIMotor(CAN_Instance *can_instance);
static void DJIMotorLostCallback(void *motor_ptr);

/* ------------------------------------------ 变量声明  --------------------------------------- */
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotor_Instance *dji_motor_instances[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CAN_Instance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[6] = {0};

/* ---------------------------------------- 公有函数实现  ------------------------------------- */

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJIMotorInstance*
 */
DJIMotor_Instance *DJIMotorInit(Motor_Init_Config_s *config)
{
    if (idx > DJI_MOTOR_CNT) {
        while (1) {
            // 超出最大注册数量
        }
    }

    DJIMotor_Instance *motor = (DJIMotor_Instance *)malloc(sizeof(DJIMotor_Instance));
    memset(motor, 0, sizeof(DJIMotor_Instance));

    // 电机的基本设置
    motor->motor_type     = config->motor_type;                     // 电机类型 GM6020/M3508/M2006
    motor->motor_settings = config->controller_setting_init_config; // 电机控制设置 正反转,闭环类型等

    // 电机的PID初始化
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    MotorSenderGrouping(motor, &config->can_init_config); // 电机分组,因为至多4个电机可以共用一帧CAN控制报文

    // 电机的CAN初始化
    config->can_init_config.can_module_callback = DecodeDJIMotor; // 设置回调函数
    config->can_init_config.id                  = motor;          // 设置拥有can实例的模块指针
    motor->motor_can_instance                   = CANRegister(&config->can_init_config);

    // 注册守护进程
    Daemon_Init_Config_s daemon_config = {
        .callback     = DJIMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    motor->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(motor); // 修改电机启动标志

    dji_motor_instances[idx++] = motor; // 添加到实例数组
    return motor;                       // 返回实例指针
}

/**
 * @brief 修改电机启动标志
 *
 * @param motor 电机实例指针
 */
void DJIMotorEnable(DJIMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJIMotorStop(DJIMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 * @param ptr   目标反馈数据指针
 */
void DJIMotorChangeFeed(DJIMotor_Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type, float *ptr)
{
    if (loop == ANGLE_LOOP) {
        motor->motor_settings.angle_feedback_source      = type;
        motor->motor_controller.other_angle_feedback_ptr = ptr;
    } else if (loop == SPEED_LOOP) {
        motor->motor_settings.speed_feedback_source      = type;
        motor->motor_controller.other_speed_feedback_ptr = ptr;
    } else
        return; // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJIMotorOuterLoop(DJIMotor_Instance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJIMotorSetRef(DJIMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/**
 * @brief 为所有电机实例计算三环PID,发送控制报文，
 *        该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJIMotorControl(void)
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotor_Instance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i) { // 减小访存开销,先保存指针引用
        motor            = dji_motor_instances[i];
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        pid_ref          = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

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
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
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

        // 获取最终输出
        set                             = (int16_t)pid_ref;
        motor->motor_controller.pid_out = set; // 保存电流环输出

        // 分组填入发送数据
        group                                         = motor->sender_group;
        num                                           = motor->message_num;
        sender_assignment[group].tx_buff[2 * num]     = (uint8_t)(set >> 8);     // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, 2u);
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i) {
        if (sender_enable_flag[i]) {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}

/* ---------------------------------------- 私有函数实现  ------------------------------------- */

/**
 * @brief 电机分组,因为至多4个电机可以共用一帧CAN控制报文
 *
 * @param motor 电机实例指针
 * @param can_config CAN初始化结构体
 */
static void MotorSenderGrouping(DJIMotor_Instance *motor, CAN_Init_Config_s *can_config)
{
    uint8_t motor_id = can_config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) {
        case M2006:
        case M3508:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = can_config->can_handle == &hcan1 ? 1 : 4;
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = can_config->can_handle == &hcan1 ? 0 : 3;
            }

            // 计算接收id
            can_config->rx_id = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
            // 设置分组发送id
            sender_enable_flag[motor_grouping] = 1;              // 设置发送标志位,防止发送空帧
            motor->message_num                 = motor_send_num; // 发送id
            motor->sender_group                = motor_grouping; // 分组

            // 检查id是否冲突
            for (uint8_t i = 0; i < idx; ++i) {
                if (dji_motor_instances[i]->motor_can_instance->can_handle == can_config->can_handle && dji_motor_instances[i]->motor_can_instance->rx_id == can_config->rx_id) {
                    // id冲突,进入错误处理
                    uint16_t can_bus __attribute__((unused)) = can_config->can_handle == &hcan1 ? 1 : 2;
                    while (1);
                }
            }
            break;

        case GM6020:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = can_config->can_handle == &hcan1 ? 0 : 3;
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = can_config->can_handle == &hcan1 ? 2 : 5;
            }

            can_config->rx_id                  = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
            sender_enable_flag[motor_grouping] = 1;                    // 设置发送标志位,防止发送空帧
            motor->message_num                 = motor_send_num;       // 发送id
            motor->sender_group                = motor_grouping;       // 分组

            for (uint8_t i = 0; i < idx; ++i) {
                if (dji_motor_instances[i]->motor_can_instance->can_handle == can_config->can_handle && dji_motor_instances[i]->motor_can_instance->rx_id == can_config->rx_id) {
                    // id冲突,进入错误处理
                    uint16_t can_bus __attribute__((unused)) = can_config->can_handle == &hcan1 ? 1 : 2;
                    while (1);
                }
            }
            break;

        default:
            while (1);
    }
}

/**
 * @brief dji电机的CAN回调函数,用于解析电机的反馈报文,并对电机的反馈数据进行滤波
 *
 * @param can_instance  电机的CAN实例
 */
static void DecodeDJIMotor(CAN_Instance *can_instance)
{
    /**
     * @brief 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
     *  _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,
     *  再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
     */
    uint8_t *rxbuff              = can_instance->rx_buff;
    DJIMotor_Instance *motor     = (DJIMotor_Instance *)can_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon); // 重载守护进程
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd           = measure->ecd;
    measure->ecd                = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps          = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/**
 * @brief  电机守护进程的回调函数,用于检测电机是否丢失,如果丢失则停止电机
 *
 * @param motor_ptr
 */
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotor_Instance *motor                 = (DJIMotor_Instance *)motor_ptr;
    motor->stop_flag                         = MOTOR_STOP;
    uint16_t can_bus __attribute__((unused)) = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
}
