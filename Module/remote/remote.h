/**
 * @file remote.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief 遥控器模块
 * @version 0.1
 * @date 2024-01-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef REMOTE_H
#define REMOTE_H

#include "stdint.h"
#include "main.h"
#include "usart.h"
#include "key_define.h"

// 检查接收值是否出错
#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP   ((uint16_t)1) // 开关向上时的值
#define RC_SW_MID  ((uint16_t)3) // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    struct
    {
        int16_t rocker_l_; // 左水平
        int16_t rocker_l1; // 左竖直
        int16_t rocker_r_; // 右水平
        int16_t rocker_r1; // 右竖直
        int16_t dial;      // 侧边拨轮

        uint8_t switch_left;  // 左侧开关
        uint8_t switch_right; // 右侧开关
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

    uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

#endif // REMOTE_H