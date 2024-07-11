#include "test.h"
// 测试代码
// 请在这里添加测试代码
// 请不要在其他地方添加测试代码
// 请不要在这里添加非测试代码
#include "dmmotor.h"

static DM_MotorInstance *dm_motor_test;
static uint8_t is_init;
void TESTInit(void)
{
    // 测试代码初始化
    Motor_Init_Config_s motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .rx_id      = 0x05, // Master ID
            .tx_id      = 2,    // MIT模式下为id，速度位置模式为0x100 + id
        },
        .controller_param_init_config = {
            .speed_PID   = {.Kp            = 0, // 0?
                            .Ki            = 0,
                            .Kd            = 0,
                            .Improve       = PID_Integral_Limit,
                            .IntegralLimit = 5,
                            .MaxOut        = 10},
            .current_PID = {.Kp            = 0, // 0
                            .Ki            = 0,
                            .Kd            = 0,
                            .Improve       = PID_Integral_Limit,
                            .IntegralLimit = 5,
                            .MaxOut        = 10},
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type    = SPEED_LOOP,
            .close_loop_type    = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .control_type = MOTOR_CONTROL_POSITION_AND_SPEED,
    };
    dm_motor_test = DMMotorInit(&motor_config);

}

void TESTTask(void)
{
    if (!is_init) {
        DMMotorControlInit();
        DMMotorSetRef(dm_motor_test, dm_motor_test->measure.position);
        DMMotorSetSpeedRef(dm_motor_test, 0.5);
        is_init = 1;
    }
    // DMMotorSetRef(dm_motor_test, 5);
    // DMMotorSetSpeedRef(dm_motor_test, 0.5);
}