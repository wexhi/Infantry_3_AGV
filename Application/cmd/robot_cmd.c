/**
 * @file robot_cmd.c
 * @author your name (you@domain.com)
 * @brief 机器人核心控制任务
 * @attention 因为底盘接的是遥控器，但是云台需要进行视觉传输，因此在预编译时不应该屏蔽掉RobotCMDInit，
 *             否则会出现内存访问错误，应在该文件中修改预编译指令。
 *             由于底盘板和云台板的任务都包含有云台电机的任务，因此应该在此处进行双板之间的通信。
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */
// application layer for robot command
#include "robot_cmd.h"
#include "robot_def.h"

// module layer
#include "remote.h"
#include "miniPC_process.h"
#include "message_center.h"
#include "user_lib.h"
#include "miniPC_process.h"
#include "referee_protocol.h"
#include "arm_math.h"
#include "DJI_motor.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE     (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)     // pitch水平时电机的角度,0-360
#define CHASSIS_VX_MAX      25000.f                                      // 底盘最大速度
#define CHASSIS_VY_MAX      25000.f                                      // 底盘最大速度
#define CHASSIS_WZ_MAX      300.f                                        // 底盘最大速度
#define SPEED_UP_RATE       50.f                                         // 底盘加速度
#define SPEED_DOWN_RATE     160.f                                        // 底盘减速度
// 对双板的兼容,条件编译
#ifdef GIMBAL_BOARD
#include "C_comm.h"
static CAN_Comm_Instance *cmd_can_comm; // 双板通信
#endif

#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif
static Vision_Recv_s *vision_ctrl; // 视觉控制信息
static RC_ctrl_t *rc_data;         // 遥控器数据指针,初始化时返回
// 若使用图传链路,则需要初始化图传链路
#ifdef VIDEO_LINK
#include "VideoTransmitter.h"
static Video_ctrl_t *video_data; // 视觉数据指针,初始化时返回
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息
static float forward_speed, back_speed, left_speed, right_speed;
static float chassis_speed_buff;
static void RemoteControlSet(void);  // 遥控器控制量设置
static void MouseKeySet(void);       // 图传链路控制量设置
static void RemoteMouseKeySet(void); // 通过遥控器的键鼠控制
static void EmergencyHandler(void) __attribute__((used));
static void CalcOffsetAngle(void); // 计算云台和底盘的偏转角度

static Robot_Status_e robot_state; // 机器人整体工作状态
/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 * @attention 工程机器人使用两块板子，它们之间的数据使用UART 1进行通信
 *
 */
void RobotCMDInit(void)
{
    // 初始化遥控器,使用串口3
    rc_data = RemoteControlInit(&huart3); // 初始化遥控器,C板上使用USART3
#ifdef VIDEO_LINK
    video_data = VideoTransmitterControlInit(&huart6); // 初始化图传链路
#endif
    vision_ctrl     = VisionInit(&huart1); // 初始化视觉控制
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub   = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub  = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef GIMBAL_BOARD
    CAN_Comm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id      = 0x312,
            .rx_id      = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD

    shoot_cmd_send.attack_mode  = NORMAL;
    shoot_cmd_send.dead_time    = 300;
    shoot_cmd_send.bullet_speed = SMALL_AMU_30;
    gimbal_cmd_send.pitch       = 0;
    gimbal_cmd_send.yaw         = 0;
    robot_state                 = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void)
{
    // 获取各个模块的数据
#ifdef ONE_BOARD
    // 获取底盘反馈信息
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    shoot_cmd_send.rest_heat = chassis_fetch_data.shoot_limit - chassis_fetch_data.shoot_heat - 20; // 计算剩余热量
    if (!rc_data[TEMP].rc.switch_right ||
        switch_is_down(rc_data[TEMP].rc.switch_right)) // 当收不到遥控器信号时，使用图传链路
    {
        MouseKeySet();
    } else if (switch_is_mid(rc_data[TEMP].rc.switch_right)) // 当收到遥控器信号时,且右拨杆为中，使用遥控器
    {
        RemoteControlSet();
    } else if (switch_is_up(rc_data[TEMP].rc.switch_right)) {
        // EmergencyHandler();
        RemoteMouseKeySet();
    }

    // 设置视觉发送数据,还需增加加速度和角速度数据
    static float yaw, pitch, roll, bullet_speed, yaw_speed;
    yaw          = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    pitch        = gimbal_fetch_data.gimbal_imu_data.Roll;
    roll         = gimbal_fetch_data.gimbal_imu_data.Pitch;
    bullet_speed = chassis_fetch_data.bullet_speed;
    yaw_speed    = gimbal_fetch_data.gimbal_imu_data.Gyro[2];

    VisionSetDetectColor(chassis_fetch_data.self_color);
    VisionSetAltitude(yaw, pitch, roll, bullet_speed, yaw_speed);

    // 发送控制信息
    // 推送消息,双板通信,视觉通信等
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.vision_mode   = vision_ctrl->is_tracking ? LOCK : UNLOCK;

    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    VisionSend();
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_LESS_THAN_4096                                  // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
#if YAW_CHASSIS_REVERSE
    chassis_cmd_send.offset_angle = -chassis_cmd_send.offset_angle;
#endif
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet(void)
{
    robot_state                     = ROBOT_READY;
    shoot_cmd_send.shoot_mode       = SHOOT_ON;
    chassis_cmd_send.chassis_mode   = CHASSIS_SLOW; // 底盘模式
    gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE;
    chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;

    // 左侧开关状态为[下],或视觉未识别到目标,纯遥控器拨杆控制
    if (switch_is_down(rc_data[TEMP].rc.switch_left) || !vision_ctrl->is_tracking) {
        // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw -= 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch += 0.0003f * (float)rc_data[TEMP].rc.rocker_r1;
    }

    // 云台参数,确定云台控制数据
    if ((switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) && vision_ctrl->is_tracking) // 左侧开关状态为[中] / [上],视觉模式
    {
        gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
        gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
    }
    // 云台软件限位
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    // max 70.f,参数过大会达到电机的峰值速度，导致底盘漂移等问题，且毫无意义
    chassis_cmd_send.vx = 100.0f * (float)rc_data[TEMP].rc.rocker_l1; // 1水平方向
    chassis_cmd_send.vy = 100.0f * (float)rc_data[TEMP].rc.rocker_l_; // _竖直方向
    chassis_cmd_send.wz = -1.0f * (float)rc_data[TEMP].rc.dial;

    // 发射参数
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[上],弹舱打开
        shoot_cmd_send.lid_mode = LID_OPEN;           // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        shoot_cmd_send.lid_mode = LID_CLOSE; // 弹舱舵机控制,待添加servo_motor模块,关闭     // _水平方向

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_up(rc_data[TEMP].rc.switch_left)) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;

    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;

    // if (vision_ctrl->is_shooting == 0)
    //     shoot_cmd_send.load_mode = LOAD_STOP;

    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
}

static void RemoteMouseKeySet(void)
{
    // switch (video_data[TEMP].key_count[V_KEY_PRESS_WITH_CTRL][V_Key_X] % 2) {
    //     case 0:
    //         EmergencyHandler();
    //         return; // 当没有按下激活键时,直接返回
    //     default:
    //         break; // 当按下激活键时,继续执行
    // }
    robot_state                     = ROBOT_READY;
    shoot_cmd_send.shoot_mode       = SHOOT_ON;
    chassis_cmd_send.chassis_mode   = CHASSIS_SLOW; // 底盘模式
    gimbal_cmd_send.gimbal_mode     = GIMBAL_GYRO_MODE;
    chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 0:
            chassis_speed_buff              = 0.8f;
            chassis_cmd_send.chassis_mode   = CHASSIS_SLOW;
            chassis_cmd_send.super_cap_mode = SUPER_CAP_OFF;
            break;
        default:
            chassis_speed_buff              = 1.2f;
            chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
            chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
            break;
    }

    if (rc_data[TEMP].key[KEY_PRESS].x) {
        chassis_speed_buff              = 0.8f;
        chassis_cmd_send.chassis_mode   = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
    }

    // 若在底盘跟随云台模式下按住shift键，则强制改为小陀螺模式
    if (rc_data[TEMP].key[KEY_PRESS].shift && chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        chassis_speed_buff              = 1.2f;
        chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
        chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
    }

    // chassis_cmd_send.vx = (video_data[TEMP].key[KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * 35000 * chassis_speed_buff; // 系数待测
    // chassis_cmd_send.vy = (video_data[TEMP].key[KEY_PRESS].d - video_data[TEMP].key[KEY_PRESS].a) * 35000 * chassis_speed_buff;
    // chassis_cmd_send.wz = video_data[TEMP].key[KEY_PRESS].shift * 1000 * chassis_speed_buff;

    if (rc_data[TEMP].key[KEY_PRESS].w) {
        forward_speed += SPEED_UP_RATE;
    } else {
        if (forward_speed > CHASSIS_VX_MAX / 4) {
            forward_speed -= SPEED_DOWN_RATE * 4;
        } else {
            forward_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].s) {
        back_speed += SPEED_UP_RATE;
    } else {
        if (back_speed > CHASSIS_VX_MAX / 4) {
            back_speed -= SPEED_DOWN_RATE * 4;
        } else {
            back_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].a) {
        left_speed += SPEED_UP_RATE;
    } else {
        if (left_speed > CHASSIS_VY_MAX / 4) {
            left_speed -= SPEED_DOWN_RATE * 4;
        } else {
            left_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].d) {
        right_speed += SPEED_UP_RATE;
    } else {
        if (right_speed > CHASSIS_VY_MAX / 4) {
            right_speed -= SPEED_DOWN_RATE * 4;
        } else {
            right_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (rc_data[TEMP].key[KEY_PRESS].shift) {
        chassis_cmd_send.wz += 1;
    } else {
        chassis_cmd_send.wz = 0;
    }

    if (forward_speed < 0) forward_speed = 0;
    if (back_speed < 0) back_speed = 0;
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;

    chassis_cmd_send.vx = forward_speed - back_speed;
    chassis_cmd_send.vy = right_speed - left_speed;

    if (chassis_cmd_send.vx > CHASSIS_VX_MAX * chassis_speed_buff) chassis_cmd_send.vx = CHASSIS_VX_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vx < -CHASSIS_VX_MAX * chassis_speed_buff) chassis_cmd_send.vx = -CHASSIS_VX_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vy > CHASSIS_VY_MAX * chassis_speed_buff) chassis_cmd_send.vy = CHASSIS_VY_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vy < -CHASSIS_VY_MAX * chassis_speed_buff) chassis_cmd_send.vy = -CHASSIS_VY_MAX * chassis_speed_buff;
    if (chassis_cmd_send.wz > CHASSIS_WZ_MAX * chassis_speed_buff) chassis_cmd_send.wz = CHASSIS_WZ_MAX * chassis_speed_buff;

    gimbal_cmd_send.yaw -= (float)rc_data[TEMP].mouse.x / 660 * 2.5; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 2.5;

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) {
        case 0:
            chassis_cmd_send.vision_lock_mode = ARMOR;
            gimbal_cmd_send.vision_lock_mode  = ARMOR;
            VisionSetEnergy(0);
            break;
        default:
            chassis_cmd_send.vision_lock_mode = RUNNE;
            gimbal_cmd_send.vision_lock_mode  = RUNNE;
            VisionSetEnergy(1);
            break;
    }

    if (rc_data[TEMP].mouse.z != 0) {
        VisionSetReset(1);
    }else{
        VisionSetReset(0);
    }

    if (vision_ctrl->is_tracking) {
        if (vision_ctrl->is_shooting) {
            chassis_cmd_send.vision_mode = LOCK;
            gimbal_cmd_send.vision_mode  = LOCK;
        } else {
            chassis_cmd_send.vision_mode = UNLOCK;
            gimbal_cmd_send.vision_mode  = UNLOCK;
        }
        if (rc_data[TEMP].mouse.press_r) // 右键开启自瞄
        {
            gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
            gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
        }
    } else {
        chassis_cmd_send.vision_mode = UNLOCK;
        gimbal_cmd_send.vision_mode  = UNLOCK;
    }

    // 云台软件限位
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    // V键刷新UI
    if (rc_data[TEMP].key[KEY_PRESS].v) {
        chassis_cmd_send.ui_mode = UI_REFRESH;
    } else {
        chassis_cmd_send.ui_mode = UI_KEEP;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键开关摩擦轮
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_B] % 4) // B键切换发弹模式
    {
        case 0:
            shoot_cmd_send.load_mode     = LOAD_SLOW;
            chassis_cmd_send.loader_mode = LOAD_SLOW; // 在此处处理是为了刷新UI
            shoot_cmd_send.shoot_rate    = 4;
            break;
        case 1:
            shoot_cmd_send.load_mode     = LOAD_FAST;
            chassis_cmd_send.loader_mode = LOAD_FAST;
            shoot_cmd_send.shoot_rate    = 8;
            break;
        case 2:
            shoot_cmd_send.load_mode     = LOAD_1_BULLET;
            chassis_cmd_send.loader_mode = LOAD_1_BULLET;
            shoot_cmd_send.dead_time     = 600;
            break;
        default:
            shoot_cmd_send.load_mode     = LOAD_3_BULLET;
            chassis_cmd_send.loader_mode = LOAD_3_BULLET;
            shoot_cmd_send.dead_time     = 300;
            break;
    }

    if (!rc_data[TEMP].mouse.press_l ||
        shoot_cmd_send.friction_mode == FRICTION_OFF ||
        shoot_cmd_send.rest_heat <= 0) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) {
        case 0:
            chassis_cmd_send.vision_is_shoot = IS_SHOOTING_ON;
            if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1) {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
            break;
        case 1:
            chassis_cmd_send.vision_is_shoot = IS_SHOOTING_OFF;
            break;
    }

    // 这行代码在演
    // if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1 &&
    //     rc_data[TEMP].mouse.press_r) {
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // }

    if (rc_data[TEMP].key[KEY_PRESS].f) // F键开启拨盘反转模式
    {
        shoot_cmd_send.load_mode     = LOAD_REVERSE;
        chassis_cmd_send.loader_mode = LOAD_REVERSE;
    }

    // switch (video_data[TEMP].key_count[KEY_PRESS][V_Key_E] % 2) // E键开关弹舱
    // {
    //     case 0:
    //         shoot_cmd_send.lid_mode = LID_CLOSE;
    //         break;
    //     default:
    //         shoot_cmd_send.lid_mode = LID_OPEN;
    //         break;
    // }
}

/**
 * @brief 图传链路以及自定义控制器的模式和控制量设置
 *
 */

static void MouseKeySet(void)
{

    if (chassis_fetch_data.current_HP == 0) {
        // 下面是为了处理图传链路死亡时的bug,当死亡时图传链路不会继续更新数据，因此我们只需跳出
        gimbal_cmd_send.yaw   = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Roll;
        return;
    }

    robot_state                 = ROBOT_READY;
    shoot_cmd_send.shoot_mode   = SHOOT_ON;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

#ifdef VIDEO_LINK
    // switch (video_data[TEMP].key_count[V_KEY_PRESS_WITH_CTRL][V_Key_X] % 2) {
    //     case 0:
    //         EmergencyHandler();
    //         return; // 当没有按下激活键时,直接返回
    //     default:
    //         break; // 当按下激活键时,继续执行
    // }

    switch (video_data[TEMP].key_count[KEY_PRESS][Key_C] % 2) {
        case 0:
            chassis_speed_buff              = 0.8f;
            chassis_cmd_send.chassis_mode   = CHASSIS_SLOW;
            chassis_cmd_send.super_cap_mode = SUPER_CAP_OFF;
            break;
        default:
            chassis_speed_buff              = 1.2f;
            chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
            chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
            break;
    }

    if (video_data[TEMP].key[KEY_PRESS].x) {
        chassis_speed_buff              = 0.8f;
        chassis_cmd_send.chassis_mode   = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
    }

    // 若在底盘跟随云台模式下按住shift键，则强制改为小陀螺模式
    if (video_data[TEMP].key[KEY_PRESS].shift && chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
        chassis_speed_buff              = 1.2f;
        chassis_cmd_send.chassis_mode   = CHASSIS_MEDIUM;
        chassis_cmd_send.super_cap_mode = SUPER_CAP_FORCE_ON;
    }

    // chassis_cmd_send.vx = (video_data[TEMP].key[KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * 35000 * chassis_speed_buff; // 系数待测
    // chassis_cmd_send.vy = (video_data[TEMP].key[KEY_PRESS].d - video_data[TEMP].key[KEY_PRESS].a) * 35000 * chassis_speed_buff;
    // chassis_cmd_send.wz = video_data[TEMP].key[KEY_PRESS].shift * 1000 * chassis_speed_buff;

    if (video_data[TEMP].key[KEY_PRESS].w) {
        if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) {
            forward_speed += 5 * SPEED_UP_RATE;
        } else {
            forward_speed += SPEED_UP_RATE;
        }
    } else {
        if (forward_speed > CHASSIS_VX_MAX / 4) {
            forward_speed -= SPEED_DOWN_RATE * 4;
        } else {
            forward_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (video_data[TEMP].key[KEY_PRESS].s) {
        back_speed += SPEED_UP_RATE;
    } else {
        if (back_speed > CHASSIS_VX_MAX / 4) {
            back_speed -= SPEED_DOWN_RATE * 4;
        } else {
            back_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (video_data[TEMP].key[KEY_PRESS].a) {
        left_speed += SPEED_UP_RATE;
    } else {
        if (left_speed > CHASSIS_VY_MAX / 4) {
            left_speed -= SPEED_DOWN_RATE * 4;
        } else {
            left_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (video_data[TEMP].key[KEY_PRESS].d) {
        right_speed += SPEED_UP_RATE;
    } else {
        if (right_speed > CHASSIS_VY_MAX / 4) {
            right_speed -= SPEED_DOWN_RATE * 4;
        } else {
            right_speed -= SPEED_DOWN_RATE / 2;
        }
    }
    if (video_data[TEMP].key[KEY_PRESS].shift) {
        chassis_cmd_send.wz += 1;
    } else {
        chassis_cmd_send.wz = 0;
    }

    if (forward_speed < 0) forward_speed = 0;
    if (back_speed < 0) back_speed = 0;
    if (left_speed < 0) left_speed = 0;
    if (right_speed < 0) right_speed = 0;

    chassis_cmd_send.vx = forward_speed - back_speed;
    chassis_cmd_send.vy = right_speed - left_speed;

    if (chassis_cmd_send.vx > CHASSIS_VX_MAX * chassis_speed_buff) chassis_cmd_send.vx = CHASSIS_VX_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vx < -CHASSIS_VX_MAX * chassis_speed_buff) chassis_cmd_send.vx = -CHASSIS_VX_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vy > CHASSIS_VY_MAX * chassis_speed_buff) chassis_cmd_send.vy = CHASSIS_VY_MAX * chassis_speed_buff;
    if (chassis_cmd_send.vy < -CHASSIS_VY_MAX * chassis_speed_buff) chassis_cmd_send.vy = -CHASSIS_VY_MAX * chassis_speed_buff;
    if (chassis_cmd_send.wz > CHASSIS_WZ_MAX * chassis_speed_buff) chassis_cmd_send.wz = CHASSIS_WZ_MAX * chassis_speed_buff;

    gimbal_cmd_send.yaw -= (float)video_data[TEMP].key_data.mouse_x / 660 * 2.5; // 系数待测
    gimbal_cmd_send.pitch += (float)video_data[TEMP].key_data.mouse_y / 660 * 2.5;

    switch (video_data[TEMP].key_count[KEY_PRESS][Key_Z] % 2) {
        case 0:
            chassis_cmd_send.vision_lock_mode = ARMOR;
            gimbal_cmd_send.vision_lock_mode  = ARMOR;
            VisionSetEnergy(0);
            break;
        default:
            chassis_cmd_send.vision_lock_mode = RUNNE;
            gimbal_cmd_send.vision_lock_mode  = RUNNE;
            VisionSetEnergy(1);
            break;
    }

    if (video_data->key_data.mouse_z != 0){
        VisionSetReset(1);
    } else {
        VisionSetReset(0);
    }

    if (vision_ctrl->is_tracking) {
        if (vision_ctrl->is_shooting) {
            chassis_cmd_send.vision_mode = LOCK;
            gimbal_cmd_send.vision_mode  = LOCK;
        } else {
            chassis_cmd_send.vision_mode = UNLOCK;
            gimbal_cmd_send.vision_mode  = UNLOCK;
        }
        if (video_data[TEMP].key_data.right_button_down) // 右键开启自瞄
        {
            gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
            gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
        }
    } else {
        chassis_cmd_send.vision_mode = UNLOCK;
        gimbal_cmd_send.vision_mode  = UNLOCK;
    }

    // 云台软件限位
    if (gimbal_cmd_send.pitch > PITCH_MAX_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MAX_ANGLE;
    else if (gimbal_cmd_send.pitch < PITCH_MIN_ANGLE)
        gimbal_cmd_send.pitch = PITCH_MIN_ANGLE;

    // V键刷新UI
    if (video_data[TEMP].key[KEY_PRESS].v) {
        chassis_cmd_send.ui_mode = UI_REFRESH;
    } else {
        chassis_cmd_send.ui_mode = UI_KEEP;
    }

    switch (video_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键开关摩擦轮
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
    }

    switch (video_data[TEMP].key_count[KEY_PRESS][Key_B] % 4) // B键切换发弹模式
    {
        case 0:
            shoot_cmd_send.load_mode     = LOAD_SLOW;
            chassis_cmd_send.loader_mode = LOAD_SLOW; // 在此处处理是为了刷新UI
            shoot_cmd_send.shoot_rate    = 4;
            break;
        case 1:
            shoot_cmd_send.load_mode     = LOAD_FAST;
            chassis_cmd_send.loader_mode = LOAD_FAST;
            shoot_cmd_send.shoot_rate    = 8;
            break;
        case 2:
            shoot_cmd_send.load_mode     = LOAD_1_BULLET;
            chassis_cmd_send.loader_mode = LOAD_1_BULLET;
            shoot_cmd_send.dead_time     = 600;
            break;
        default:
            shoot_cmd_send.load_mode     = LOAD_3_BULLET;
            chassis_cmd_send.loader_mode = LOAD_3_BULLET;
            shoot_cmd_send.dead_time     = 300;
            break;
    }

    if (!video_data[TEMP].key_data.left_button_down ||
        shoot_cmd_send.friction_mode == FRICTION_OFF ||
        shoot_cmd_send.rest_heat <= 0) {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    switch (video_data[TEMP].key_count[KEY_PRESS][Key_E] % 2) {
        case 0:
            chassis_cmd_send.vision_is_shoot = IS_SHOOTING_ON;
            if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1) {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
            break;
        case 1:
            chassis_cmd_send.vision_is_shoot = IS_SHOOTING_OFF;
            break;
    }

    // 这行代码在演
    // if (vision_ctrl->is_shooting == 0 && vision_ctrl->is_tracking == 1 &&
    //     rc_data[TEMP].mouse.press_r) {
    //     shoot_cmd_send.load_mode = LOAD_STOP;
    // }

    if (video_data[TEMP].key[KEY_PRESS].f) // F键开启拨盘反转模式
    {
        shoot_cmd_send.load_mode     = LOAD_REVERSE;
        chassis_cmd_send.loader_mode = LOAD_REVERSE;
    }

    // switch (video_data[TEMP].key_count[KEY_PRESS][V_Key_E] % 2) // E键开关弹舱
    // {
    //     case 0:
    //         shoot_cmd_send.lid_mode = LID_CLOSE;
    //         break;
    //     default:
    //         shoot_cmd_send.lid_mode = LID_OPEN;
    //         break;
    // }
#endif
}

/**
 * @brief  紧急停止,包括遥控器右侧上侧拨杆打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler(void)
{
    // 急停
    robot_state                     = ROBOT_STOP;
    gimbal_cmd_send.gimbal_mode     = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode   = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.super_cap_mode = SUPER_CAP_ON;
    shoot_cmd_send.shoot_mode       = SHOOT_OFF;
    shoot_cmd_send.friction_mode    = FRICTION_OFF;
    shoot_cmd_send.load_mode        = LOAD_STOP;
    shoot_cmd_send.lid_mode         = LID_CLOSE;
}
