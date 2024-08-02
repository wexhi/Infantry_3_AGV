/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "remote.h"
#include "VideoTransmitter.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量
static char *UIGetLevel();

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color       = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID          = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID         = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);                       // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) __attribute__((used)); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info            = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data             = UI_data;                           // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static String_Data_t UI_State_sta[8];  // 机器人状态,静态只需画一次
static String_Data_t UI_State_dyn[7];  // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[10] = {540, 978, 490, 443, 425, 453, 433};
// +18
void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务

    if (Interactive_data->ui_mode == UI_KEEP)
        return;
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制发射基准线
    // 此线修改为动态,用于识别自瞄是否识别
    UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 8, UI_Color_White, 3, 710 + 18, shoot_line_location[0], 1210 + 18, shoot_line_location[0]);
    UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 8, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);

    UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810 + 18, shoot_line_location[2], 1110 + 18, shoot_line_location[2]);
    // 5m前哨辅助线
    UILineDraw(&UI_shoot_line[5], "sl5", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930 + 18, shoot_line_location[5], 990 + 18, shoot_line_location[5]);
    UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 880 + 18, shoot_line_location[3], 1040 + 18, shoot_line_location[3]);
    UILineDraw(&UI_shoot_line[6], "sl6", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 930 + 18, shoot_line_location[6], 990 + 18, shoot_line_location[6]);
    // 8m前哨辅助线
    UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 910 + 18, shoot_line_location[4], 1010 + 18, shoot_line_location[4]);
    UIGraphRefresh(&referee_recv_info->referee_id, 7, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4], UI_shoot_line[5], UI_shoot_line[6]);

    // 绘制车辆状态标志指示
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 150, 800, "level:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 100, 750, "C chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 100, 700, "Z target:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 100, 650, "C cap:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 100, 600, "Q frict:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);
    UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 100, 550, "E IS_sho:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    UICharDraw(&UI_State_sta[6], "ss6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 100, 850, " B Bounce:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[6]);
    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    // 等级显示，动态
    // UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 21, 2, 270, 800, "1");
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 800, UIGetLevel());
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    // UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "fast     ");
    switch (Interactive_data->chassis_mode) {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
            break;
        case CHASSIS_FAST:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "fast     ");
            break;
        case CHASSIS_MEDIUM:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "medium   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "follow   ");
            break;
        case CHASSIS_SLOW:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "slow     ");
            break;
    }
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    // UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "armor");
    switch (Interactive_data->vision_lock_mode) {
        case ARMOR: {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "armor");
            break;
        }
        case RUNNE: {
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Yellow, 15, 2, 270, 700, "runne");
            break;
        }
    }
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    // UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "off");
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, Interactive_data->super_cap_mode == SUPER_CAP_ON ? "on " : "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    // UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, "off");
    UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 600, Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    // UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, Interactive_data->vision_is_shoot == IS_SHOOTING_ON ? "on " : "off");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    // UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "slow    ");
    switch (Interactive_data->loader_mode) {
        case LOAD_REVERSE:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "reverse ");
            break;
        case LOAD_SLOW:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "slow    ");
            break;
        case LOAD_MEDIUM:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "medium  ");
            break;
        case LOAD_FAST:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "fast    ");
            break;
        case LOAD_STOP:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "stop    ");
            break;
        case LOAD_1_BULLET:
            UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_ADD, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "1_bullet");
            break;
        default:
            break;
    }
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);

    // 底盘功率显示，静态
    UICharDraw(&UI_State_sta[7], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[7]);
    // 能量条框
    UIRectangleDraw(&UI_Energy[0], "ss7", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // 底盘功率显示,动态
    UIFloatDraw(&UI_Energy[1], "sd7", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // 能量条初始状态
    UILineDraw(&UI_Energy[2], "sd8", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

// 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count   = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50) {
        count = 0;
        count1++;
    }
    switch (count1 % 4) {
        case 0: {
            // _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
            _Interactive_data->gimbal_mode   = GIMBAL_ZERO_FORCE;
            _Interactive_data->shoot_mode    = SHOOT_ON;
            _Interactive_data->friction_mode = FRICTION_ON;
            // _Interactive_data->lid_mode      = LID_OPEN;
            _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5f;
            if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
                _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
            break;
        }
        case 1: {
            // _Interactive_data->chassis_mode = CHASSIS_ROTATE;
            _Interactive_data->gimbal_mode   = GIMBAL_FREE_MODE;
            _Interactive_data->shoot_mode    = SHOOT_OFF;
            _Interactive_data->friction_mode = FRICTION_OFF;
            // _Interactive_data->lid_mode      = LID_CLOSE;
            break;
        }
        case 2: {
            // _Interactive_data->chassis_mode = CHASSIS_NO_FOLLOW;
            _Interactive_data->gimbal_mode   = GIMBAL_GYRO_MODE;
            _Interactive_data->shoot_mode    = SHOOT_ON;
            _Interactive_data->friction_mode = FRICTION_ON;
            // _Interactive_data->lid_mode      = LID_OPEN;
            break;
        }
        case 3: {
            // _Interactive_data->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            _Interactive_data->gimbal_mode   = GIMBAL_ZERO_FORCE;
            _Interactive_data->shoot_mode    = SHOOT_OFF;
            _Interactive_data->friction_mode = FRICTION_OFF;
            // _Interactive_data->lid_mode      = LID_CLOSE;
            break;
        }
        default:
            break;
    }
}

static char *UIGetLevel()
{
    switch (referee_recv_info->GameRobotState.robot_level) {
        case 1:
            return "1";
        case 2:
            return "2";
        case 3:
            return "3";
        case 4:
            return "4";
        case 5:
            return "5";
        case 6:
            return "6";
        case 7:
            return "7";
        case 8:
            return "8";
        case 9:
            return "9";
        case 10:
            return "10";
    }
    return "0";
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // level
    if (_Interactive_data->Referee_Interactive_Flag.level_flag == 1) {
        UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 800, UIGetLevel());
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.level_flag = 0;
    }
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1) {
        switch (_Interactive_data->chassis_mode) {
            case CHASSIS_ZERO_FORCE:
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
                break;
            case CHASSIS_FAST:
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 750, "fast     ");
                break;
            case CHASSIS_MEDIUM:
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 750, "medium   ");
                // 此处注意字数对齐问题，字数相同才能覆盖掉
                break;
            case CHASSIS_FOLLOW_GIMBAL_YAW:
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Cyan, 15, 2, 270, 750, "follow   ");
                break;
            case CHASSIS_SLOW:
                UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 750, "slow     ");
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // lock_mode
    if (_Interactive_data->Referee_Interactive_Flag.vision_lock_flag == 1) {
        switch (_Interactive_data->vision_lock_mode) {
            case ARMOR: {
                UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 700, "armor");
                break;
            }
            case RUNNE: {
                UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Main, 15, 2, 270, 700, "runne");
                break;
            }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.vision_lock_flag = 0;
    }
    // super_cap
    if (_Interactive_data->Referee_Interactive_Flag.super_cap_flag == 1) {
        switch (_Interactive_data->super_cap_mode) {
            case SUPER_CAP_ON:
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, "on   ");
                break;
            case SUPER_CAP_OFF:
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 650, "off  ");
                break;
            case SUPER_CAP_FORCE_ON:
                UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 650, "force");
                break;
            default:
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.super_cap_flag = 0;
    }
    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1) {
        UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }
    // lid
    // if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1) {
    //     UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    // }
    // loader
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1) {
        switch (_Interactive_data->loader_mode) {
            case LOAD_REVERSE:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Purplish_red, 15, 2, 270, 850, "reverse ");
                break;
            case LOAD_SLOW:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Green, 15, 2, 270, 850, "slow    ");
                break;
            case LOAD_MEDIUM:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Yellow, 15, 2, 270, 850, "medium  ");
                break;
            case LOAD_FAST:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 270, 850, "fast    ");
                break;
            case LOAD_STOP:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Black, 15, 2, 270, 850, "stop    ");
                break;
            case LOAD_1_BULLET:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Cyan, 15, 2, 270, 850, "1_bullet");
                break;
            case LOAD_3_BULLET:
                UICharDraw(&UI_State_dyn[6], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 850, "3_bullet");
                break;
            default:
                break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[6]);
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }
    // power
    if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1) {
        UIFloatDraw(&UI_Energy[1], "sd7", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
        // if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 12)
        UILineDraw(&UI_Energy[2], "sd8", UI_Graph_Change, 8, UI_Color_Pink, 34, 722, 160, (uint32_t)722 + ((_Interactive_data->Chassis_Power_Data.chassis_power_mx - 9) / 15 * 496), 160);
        // else
        //     UILineDraw(&UI_Energy[2], "sd8", UI_Graph_Change, 8, UI_Color_Black, 30, 720, 160, (uint32_t)720 + ((_Interactive_data->Chassis_Power_Data.chassis_power_mx - 12) / 15 * 496), 160);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
        _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    }
    // // is_tracking
    if (_Interactive_data->Referee_Interactive_Flag.tracking_flag == 1) {
        UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_Change, 8, _Interactive_data->vision_mode == LOCK ? UI_Color_Pink : UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
        UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_Change, 8, _Interactive_data->vision_mode == LOCK ? UI_Color_Pink : UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_shoot_line[0], UI_shoot_line[1]);
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 0;
    }
    // vision_is_shooting
    if (_Interactive_data->Referee_Interactive_Flag.vision_is_shoot_flag == 1) {
        UICharDraw(&UI_State_dyn[5], "sd5", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->vision_is_shoot == IS_SHOOTING_ON ? "on " : "off");
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[5]);
        _Interactive_data->Referee_Interactive_Flag.vision_is_shoot_flag = 0;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode                     = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->vision_lock_mode != _Interactive_data->vision_last_lock_mode) {
        _Interactive_data->Referee_Interactive_Flag.vision_lock_flag = 1;
        _Interactive_data->vision_last_lock_mode                     = _Interactive_data->vision_lock_mode;
    }

    if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
        _Interactive_data->shoot_last_mode                     = _Interactive_data->shoot_mode;
    }

    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode                     = _Interactive_data->friction_mode;
    }

    if (_Interactive_data->vision_is_shoot != _Interactive_data->vision_last_is_shoot) {
        _Interactive_data->Referee_Interactive_Flag.vision_is_shoot_flag = 1;
        _Interactive_data->vision_last_is_shoot                          = _Interactive_data->vision_is_shoot;
    }

    if (_Interactive_data->loader_mode != _Interactive_data->loader_mode_last) {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_mode_last                     = _Interactive_data->loader_mode;
    }

    if (_Interactive_data->Chassis_Power_Data.chassis_power_mx != _Interactive_data->Chassis_last_Power_Data.chassis_power_mx) {
        _Interactive_data->Referee_Interactive_Flag.Power_flag      = 1;
        _Interactive_data->Chassis_last_Power_Data.chassis_power_mx = _Interactive_data->Chassis_Power_Data.chassis_power_mx;
    }

    if (_Interactive_data->level != _Interactive_data->level_last) {
        _Interactive_data->Referee_Interactive_Flag.level_flag = 1;
        _Interactive_data->level_last                          = _Interactive_data->level;
    }

    if (_Interactive_data->vision_mode != _Interactive_data->vision_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.tracking_flag = 1;
        _Interactive_data->vision_last_mode                       = _Interactive_data->vision_mode;
    }

    if (_Interactive_data->super_cap_mode != _Interactive_data->super_cap_last_mode) {
        _Interactive_data->Referee_Interactive_Flag.super_cap_flag = 1;
        _Interactive_data->super_cap_last_mode                     = _Interactive_data->super_cap_mode;
    }
}
