/**
 * @file VideoTransmitter.h
 * @author wexhi (wexhi@qq.com)
 * @brief  用于图传链路的接收以及解析
 * @version 1.1
 * @date 2024-05-06
 * @todo 图传链路应该属于遥控器控制的附庸，后续考虑合并或转移
 *
 * @copyright Copyright (c) 2024 CQU QianLi EC 2024 all rights reserved
 *
 */

#ifndef VIDEOTRANSMITTER_H
#define VIDEOTRANSMITTER_H

#include "stdint.h"
#include "main.h"
#include "usart.h"

#include "referee_protocol.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LASTV 1
#define TEMPV 0

// 获取按键操作
#define V_KEY_PRESS            0
#define V_KEY_STATE            1
#define V_KEY_PRESS_WITH_CTRL  1
#define V_KEY_PRESS_WITH_SHIFT 2

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[V_Key_PRESS][V_Key_W]获取W键是否按下,后续改为位域后删除
#define V_Key_W     0
#define V_Key_S     1
#define V_Key_A     2
#define V_Key_D     3
#define V_Key_Shift 4
#define V_Key_Ctrl  5
#define V_Key_Q     6
#define V_Key_E     7
#define V_Key_R     8
#define V_Key_F     9
#define V_Key_G     10
#define V_Key_Z     11
#define V_Key_X     12
#define V_Key_C     13
#define V_Key_V     14
#define V_Key_B     15

#pragma pack(1)

typedef union {
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} V_Key_t;
typedef struct
{
    float maximal_arm_target; // 大臂的目标值
    float minimal_arm_target; // 小臂的目标值
    float finesse_target;     // 手腕的目标值
    float pitch_arm_target;   // pitch的目标值
    float roll_arm_target;    // roll的目标值
    float height;             // z轴的目标值
} Custom_contorl_t;

typedef struct
{
    float delta_x;
    float delta_y;
    float delta_z;
    float delta_yaw;
    float delta_pitch;
    float delta_roll;
} slightly_controll_data;

typedef struct
{
    xFrameHeader FrameHeader;        // 接收到的帧头信息
    uint16_t CmdID;                  // 命令码
    custom_robot_data_t custom_data; // 自定义数据
    uint8_t custom_control_mode;     // 自定义控制模式
    Custom_contorl_t cus;            // 解算后的自定义数据
    slightly_controll_data scd;      // 轻微控制数据
    remote_control_t key_data;       // 遥控器数据

    V_Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} Video_ctrl_t;

#pragma pack()

Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);
#endif // !VIDEOTRANSMITTER_H