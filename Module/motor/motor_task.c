#include "motor_task.h"
#include "dji_motor.h"
#include "LK_motor.h"

long long time_motor = 0;
void MotorControlTask()
{
    time_motor++;
    // static uint8_t cnt = 0; 设定不同电机的任务频率
    // if(cnt%5==0) //200hz
    // if(cnt%10==0) //100hz
    DJIMotorControl();
    /* 如果有对应的电机则取消注释,可以加入条件编译或者register对应的idx判断是否注册了电机 */
    LKMotorControl();
}