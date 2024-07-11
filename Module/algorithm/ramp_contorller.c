#include "ramp_contorller.h"
#include "arm_math.h"
#include "bsp_dwt.h"

/**
 * @brief 初始化斜坡控制器
 *
 * @param rampControllerInstance 斜坡控制器实例
 * @param config 斜坡控制器配置
 */
void RampController_Init(RampController_Instance *rampControllerInstance, RampController_Config_s *config)
{
    rampControllerInstance->currentSetpoint = 0;
    rampControllerInstance->targetSetpoint  = 0;
    rampControllerInstance->rampTime        = config->rampTime;
    rampControllerInstance->elapsedTime     = 0;
    rampControllerInstance->lastPosition    = 0;
    rampControllerInstance->isActive        = 0; // 初始时不激活斜坡

    DWT_GetDeltaT(&rampControllerInstance->DWT_CNT);
}

void StartRamp(RampController_Instance *ramp, float current_point, float target_point)
{
    ramp->lastPosition    = current_point;
    ramp->targetSetpoint  = target_point;
    ramp->currentSetpoint = current_point; // 设置当前设定点为电机当前位置
    ramp->elapsedTime     = 0.0;           // 重置时间
    ramp->isActive        = 1;             // 激活斜坡
}

float UpdateRamp(RampController_Instance *ramp)
{
    ramp->dt = DWT_GetDeltaT(&ramp->DWT_CNT); // 获取两次pid计算的时间间隔,用于积分和微分
    if (ramp->isActive) {
        if (ramp->elapsedTime < ramp->rampTime) {
            ramp->elapsedTime += ramp->dt;
            float progress        = ramp->elapsedTime / ramp->rampTime;
            float sCurveProgress  = (1 - arm_cos_f32(progress * PI)) / 2; // S曲线计算
            ramp->currentSetpoint = ramp->lastPosition + (ramp->targetSetpoint - ramp->lastPosition) * sCurveProgress;
        } else {
            ramp->currentSetpoint = ramp->targetSetpoint; // 确保最终达到目标设定点
            ramp->isActive        = 0;                    // 完成斜坡后停止更新
        }
    }
    return ramp->currentSetpoint;
}