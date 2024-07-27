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
    uint16_t buffer;         // 缓冲能量
    uint16_t power;          // 底盘功率
    uint8_t state;           // 状态
    uint16_t motor1_current; // 电机1电流
    uint16_t motor2_current; // 电机2电流
    uint16_t motor3_current; // 电机3电流
    uint16_t motor4_current; // 电机4电流
} SupCapSend_t;
#pragma pack()

typedef enum {
    SUP_CAP_STATE_DISCHARGING = 0, // 放电,根据电压状态判断当前需要充电还是放电
    SUP_CAP_STATE_CHARGING,        // 充电,需要进行功率限制帮助超电充电
} SupCapState_e;

/* 超级电容实例 */
typedef struct
{
    CAN_Instance *can_ins;                  // CAN实例
    CAN_Instance *can_ins_trans_motor_data; // CAN实例
    SuperCapData_t cap_data;                // 超级电容信息
    SupCapSend_t send_data;                 // 发送数据
    SupCapState_e state;                    // 状态
    Daemon_Instance *daemon;                // 守护实例
} SuperCap_Instance;

/* 超级电容初始化配置 */
typedef struct
{
    CAN_Init_Config_s can_config;
    CAN_Init_Config_s can_motor_data_config;
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

/**
 * @brief 设置电机电流
 *
 * @param motor1_current 电机1电流
 * @param motor2_current 电机2电流
 * @param motor3_current 电机3电流
 * @param motor4_current 电机4电流
 */
void SuperCapSetMotor(int16_t motor1_current, int16_t motor2_current, int16_t motor3_current, int16_t motor4_current);

/**
 * @brief 发送电机电流数据
 *
 */
void SuperCapMotorSend(void);
#endif // !SUP_CAP_H