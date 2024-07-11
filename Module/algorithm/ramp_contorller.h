/**
 * @file ramp_contorller.h
 * @author Bi KaiXiang (wexhi@qq.com)
 * @brief 用于处理斜坡函数的控制器
 * @version 0.1
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef RAMP_CONTORLLER_H
#define RAMP_CONTORLLER_H

#include "stdint.h"

/**
 * @brief 斜坡控制器,Config
 *
 */
typedef struct {
    float rampTime; // 斜坡总时长，秒
} RampController_Config_s;

/**
 * @brief 斜坡控制器
 *
 */
typedef struct {
    float currentSetpoint; // 当前斜坡设定点
    float targetSetpoint;  // 目标设定点
    float rampTime;        // 斜坡总时长，秒
    float elapsedTime;     // 已过时间，秒
    float lastPosition;    // 最后一次记录的电机位置
    int isActive;          // 是否激活斜坡

    uint32_t DWT_CNT;
    float dt;

} RampController_Instance;

/**
 * @brief 初始化斜坡控制器
 *
 * @param config 斜坡控制器配置
 * @param rampControllerInstance 斜坡控制器实例
 */
void RampController_Init(RampController_Instance *rampControllerInstance, RampController_Config_s *config);

/**
 * @brief 开始斜坡
 *
 * @param ramp 斜坡控制器实例
 * @param current_point 当前点
 * @param target_point 目标点
 */
void StartRamp(RampController_Instance *ramp, float current_point, float target_point);

/**
 * @brief 更新斜坡控制器
 *
 * @param ramp 斜坡控制器实例
 * @param deltaTime 时间间隔
 * @return float 当前设定点
 */
float UpdateRamp(RampController_Instance *ramp);
#endif