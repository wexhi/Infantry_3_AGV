/**
 * @file robot_cmd.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人指令,用于和上位机通信,以及遙控器通信
 * @version 0.1
 * @date 2024-01-03
 *
 * @copyright Copyright (c) 2024
 *
 */

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 *
 */
void RobotCMDInit(void);

/**
 * @brief 机器人核心控制任务,会被RobotTask()调用
 *
 */
void RobotCMDTask(void);