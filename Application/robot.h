/**
 * @file robot.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief 机器人初始化任务以及机器人rtos任务的入口
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef ROBOT_H
#define ROBOT_H

/**
 * @brief 机器人初始化
 *
 */
void RobotInit(void);

/**
 * @brief 机器人任务入口
 *
 */
void RobotTask(void);

#endif // ROBOT_H