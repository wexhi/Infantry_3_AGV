/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人定义,包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"
#include "ins_task.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // ! 单板控制整车，beta选项，建议别选上
#define CHASSIS_BOARD // 底盘板
// #define GIMBAL_BOARD // 云台板

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */

#define VISION_USE_VCP // 是否使用虚拟串口
// #define VISION_USE_UART // 是否使用硬件串口

#define VIDEO_LINK // 是否有图传链路
// #define REMOTE_LINK // 是否有常规链路

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD     600   // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0     // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD         2100  // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE           25.f  // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE           -30.f // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 45    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 36.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE         8     // 拨盘一圈的装载量
// 底盘参数
// #define CHASSIS_OMNI_WHEEL // 是否为全向轮底盘
// #define CHASSIS_MCNAMEE_WHEEL     // 是否为麦克纳姆轮底盘
#define CHASSIS_STEERING_WHEEL               // 是否为舵轮
#define STEERING_CHASSIS_ALIGN_ECD_LF   7630 // 舵电机 A 编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_LB   4089 // 舵电机 B 编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_RF   1402 // 舵电机 C 编码器值，若有机械改动需要修改
#define STEERING_CHASSIS_ALIGN_ECD_RB   659  // 舵电机 D 编码器值，若有机械改动需要修改

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // 舵轮 A 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // 舵轮 B 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // 舵轮 C 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // 舵轮 D 对齐角度

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(ARM_BOARD)) ||     \
    (defined(CHASSIS_BOARD) && defined(ARM_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

// 检查是否出现底盘类型定义冲突，只允许一个底盘类型定义存在，否则编译会自动报错
#if (defined(CHASSIS_OMNI_WHEEL) && defined(CHASSIS_MCNAMEE_WHEEL)) ||  \
    (defined(CHASSIS_OMNI_WHEEL) && defined(CHASSIS_STEERING_WHEEL)) || \
    (defined(CHASSIS_MCNAMEE_WHEEL) && defined(CHASSIS_STEERING_WHEEL))
#error Conflict chassis definition! You can only define one chassis type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum {
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum {
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum {
    CHASSIS_ZERO_FORCE = 0, // 电流零输入
    CHASSIS_FAST,           // 底盘转速快
    CHASSIS_MEDIUM,         // 底盘转速中等
    CHASSIS_SLOW,           // 底盘转速慢
    CHASSIS_FOLLOW_GIMBAL_YAW,
} chassis_mode_e;

typedef enum {
    SUPER_CAP_OFF = 0, // 超级电容关闭
    SUPER_CAP_ON,      // 超级电容开启
} super_cap_mode_e;

typedef enum {
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum {
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum {
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum {
    NORMAL = 0, // 正常模式
    VIOLENT,    // 狂暴模式, 提高单发射频
} attack_mode_e;

typedef enum {
    LOAD_STOP = 0, // 停止发射
    LOAD_REVERSE,  // 反转
    LOAD_SLOW,     // 慢速
    LOAD_MEDIUM,   // 中速
    LOAD_FAST,     // 快速
    LOAD_1_BULLET, // 单发
    LOAD_3_BULLET, // 三发
} loader_mode_e;

typedef enum {
    BULLET_SPEED_NONE = 0,
    BIG_AMU_10        = 10,
    SMALL_AMU_15      = 15,
    BIG_AMU_16        = 16,
    SMALL_AMU_18      = 18,
    SMALL_AMU_30      = 30,
} Bullet_Speed_e;

typedef enum {
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum {
    UNLOCK = 0, // 未开启自瞄或视觉未锁定
    CONVERGE,   // 视觉锁定过程中，正在校准
    LOCK,       // 视觉锁定, 可以开火
} vision_mode_e;

typedef enum {
    ARMOR = 0, // 瞄准装甲板
    RUNNE,     // 瞄准打符
} vision_lock_mode_e;

// 功率限制,从裁判系统获取,是否有必要保留?
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

// UI模式设置
typedef enum {
    UI_KEEP = 0,
    UI_REFRESH,
} ui_mode_e;

// 机械臂模式设置
typedef enum {
    ARM_ZERO_FORCE = 0,   // 电流零输入
    ARM_HUM_CONTORL,      // 自定义控制器控制
    ARM_VISION_CONTROL,   // 视觉控制
    ARM_SLIGHTLY_CONTROL, // 轻微控制
    ARM_KEY_CONTROL,      // 键盘控制
    ARM_AUTO_CONTORL,     // 自动控制
    ARM_LIFT_INIT,        // 高度初始化
} arm_mode_e;

// 机械臂控制状态设置,注意与机械臂模式区分,这里可以看作机械臂模式的子模式
typedef enum {
    ARM_NORMAL = 0,  // 正常状态,能够被其他模式正常控制
    ARM_RECYCLE,     // 回收状态,机械臂回收到初始位置
    ARM_GETCARROCK,  // 抓取状态,机械臂抓取石块
    ARM_GETCARROCK2, // 抓取状态2,机械臂抓取石块后取出
    ARM_GETROCK,     // 抓取状态,机械臂抓取石块,一键取矿
} arm_status_e;

typedef enum {
    SUCKER_OFF = 0, // 涵道风机关
    SUCKER_ON,      // 涵道风机开
} sucker_mode_e;

typedef enum {
    LIFT_OFF = 0,    // 机械臂升降关闭
    LIFT_ANGLE_MODE, // 机械臂升降角度模式
    LIFT_SPEED_MODE, // 机械臂升降速度模式
    LIFT_KEEP,       // 机械臂升降保持模式
    LIFT_INIT_MODE,  // 机械臂升降初始化模式
} lift_mode_e;

typedef enum {
    ROLL_OFF = 0,    // 机械臂roll关闭
    ROLL_ANGLE_MODE, // 机械臂roll开
    ROLL_SPEED_MODE, // 机械臂roll开
    ROLL_KEEP,       // 机械臂roll保持
} roll_mode_e;

typedef enum {
    DOWNLOAD_OFF = 0, // 关闭调试模式
    DOWNLOAD_ON,      // 开启调试模式,用于控制大疆电机的调试，防止下载时电机转动
} download_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,pc在云台,遥控器和裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    super_cap_mode_e super_cap_mode;
    // UI部分
    ui_mode_e ui_mode;                   //  UI状态
    friction_mode_e friction_mode;       //  摩擦轮状态
    loader_mode_e loader_mode;           //  射频状态
    vision_mode_e vision_mode;           //  视觉状态
    vision_lock_mode_e vision_lock_mode; // 视觉锁定的目标状态
    lid_mode_e lid_mode;                 //  弹舱盖状态
    //  ...

} Chassis_Ctrl_Cmd_s;

// cmd发布的机械臂控制数据,由arm订阅
typedef struct
{
    float maximal_arm;             // 机械臂大臂目标角度
    float minimal_arm;             // 机械臂小臂目标角度
    float finesse;                 // 机械臂手腕目标角度
    float pitch_arm;               // 机械臂pitch目标角度
    float lift;                    // 机械臂高度
    float roll;                    // 机械臂roll目标角度
    lift_mode_e lift_mode;         // 机械臂上升标志
    roll_mode_e roll_mode;         // 机械臂roll标志
    sucker_mode_e sucker_mode;     // 涵道风机状态
    arm_mode_e arm_mode;           // 机械臂状态
    arm_mode_e arm_mode_last;      // 机械臂上一次状态
    arm_status_e arm_status;       // 机械臂控制状态(状态子模式)
    download_mode_e download_mode; // 下载模式
    int8_t lift_init;              // 机械臂初始化
} Arm_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{ // 云台角度控制
    float yaw;
    float pitch;
    float chassis_rotate_wz;

    gimbal_mode_e gimbal_mode;
    vision_mode_e vision_mode;
    vision_lock_mode_e vision_lock_mode;
} Gimbal_Ctrl_Cmd_s;

// 双板时，下板cmd发布控制云台控制数据，由gimbal订阅
typedef struct
{
    float yaw;
    float up_yaw;
    float up_speed;

    uint8_t is_init;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Yaw_Cmd_s;

typedef struct
{
    float pitch;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Pitch_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    int16_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    float dead_time;  // 发射冷却时间, 目前仅单发使用
    attack_mode_e attack_mode;
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD) // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif

    // 后续增加底盘的真实速度
    // float real_vx;
    // float real_vy;
    // float real_wz;
    // 底盘反馈数据
    // uint8_t is_video_link; // 是否有图传链路
    uint16_t shoot_heat;  // 枪口热量
    uint16_t shoot_limit; // 枪口热量上限
    float bullet_speed;   // 裁判系统弹速
    // Gimbal_Ctrl_Cmd_s gimbal_ctrl_cmd;
    // Shoot_Ctrl_Cmd_s shoot_ctrl_cmd;
    // Bullet_Speed_e bullet_speed; // 弹速限制

} Chassis_Upload_Data_s;

// 机械臂反馈数据
typedef struct
{
    float maximal_arm;
    float minimal_arm;
    float finesse;
    float pitch_arm;
    float height;
    float roll;
} Arm_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;

#pragma pack() // 取消压缩
#endif