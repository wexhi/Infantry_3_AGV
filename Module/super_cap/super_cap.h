/**
 * @file super_cap.h
 * @author Bi KaiXiang (wexhi@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-05-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SUP_CAP_H
#define SUP_CAP_H

#include "bsp_can.h"
#include "daemon.h"

#pragma pack(1)
typedef struct
{
    uint16_t voltage; // 电压
    uint16_t power;   // 功率
    uint8_t status;   // 状态
} SuperCapData_t;

typedef struct {
    uint16_t buffer; // 缓冲能量
    uint16_t power;  // 底盘功率
    uint8_t state;   // 状态
} SupCapSend_t;
#pragma pack()

/* 超级电容实例 */
typedef struct
{
    CAN_Instance *can_ins;   // CAN实例
    SuperCapData_t cap_data; // 超级电容信息
    SupCapSend_t send_data;  // 发送数据
    Daemon_Instance *daemon; // 守护实例
} SuperCap_Instance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
} SuperCap_Init_Config_s;

/**
 * @brief 初始化超级电容
 *
 * @param config 超级电容初始化配置
 * @return SuperCap_Instance* 超级电容实例
 */
SuperCap_Instance *SuperCapInit(SuperCap_Init_Config_s *config);

/**
 * @brief 设置超级电容数据
 *
 * @param buffer 缓冲能量
 * @param power 底盘功率
 * @param state 状态
 */
void SuperCapSet(uint16_t buffer, uint16_t power, uint8_t state);

/**
 * @brief 发送超级电容数据
 *
 *
 */
void SuperCapSend(void);
#endif // !SUP_CAP_H