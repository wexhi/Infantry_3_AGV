<!--
 * @file VideoTransmitter.h
 * @author wexhi (wexhi@qq.com)
 * @brief  用于图传链路的接收以及解析
 * @version 2.1
 * @date 2024-08-30
 *
 * @copyright Copyright (c) 2024 CQU QianLi EC 2024 all rights reserved
 *
-->

# 图传链路

**注意**：图传链路在赛场上有**丢控**的可能，并且在**死亡时**会保留图传发送的**最后一帧**，出现的现象可能会导致**Yaw值**在死亡时一直**累加**，复活时头一直旋转，因此需要在死亡时进行**额外判断**

## 代码结构

新版本对遥控器中的remote.h进行了解耦，将图传链路的接收和解析单独提取出来，方便后续的维护和修改

## 外部接口

``` c
/**
 * @brief 初始化图传链路
 *
 * @param video_usart_handle 图传串口句柄
 * @return Video_ctrl_t* 图传链路控制实例
 */
Video_ctrl_t *VideoTransmitterControlInit(UART_HandleTypeDef *video_usart_handle);
```

## 私有函数和变量

``` c
static Video_ctrl_t video_ctrl[2]; // 用于存储图传链路的控制数据,[0]:当前数据TEMPV,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t is_init;
// 图传拥有的串口实例,因为图传是单例,所以这里只有一个,就不封装了
static USART_Instance *video_usart_instance;
static Daemon_Instance *video_daemon_instance;

static void VideoDataContorl();

/**
 * @brief 图传数据解析函数
 *
 * @param buff 图传数据
 */
static void VideoRead(uint8_t *buff);

/**
 * @brief  图传数据接收回调函数
 *
 */
static void VideoTransmitterCallback()

static void VideoTransmitterLostCallback()
```

## 如何使用？

1. 在需要使用图传链路的文件中包含头文件并且可以创建一个图传链路实例指针

    ``` c
    #include "VideoTransmitter.h"
    static Video_ctrl_t *video_data; // 视觉数据指针,初始化时返回
    ```

2. 在初始化的地方文件中调用初始化函数

    ``` c
    video_data = VideoTransmitterControlInit(&huart1);
    ```

3. 在需要使用图传数据的地方调用数据控制函数

    ``` c
    switch (video_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_X] % 2) {
        case 0:
            EmergencyHandler();
            return; // 当没有按下激活键时,直接返回
        default:
            break; // 当按下激活键时,继续执行
    }
    ```

    以上只是一个例子，作用于当未按下激活键时执行紧急处理函数，按下后可以使能整车，具体功能可以根据需求自行设计，当我需要设置底盘速度或云台位置时

    ``` c
    chassis_cmd_send.vx = (video_data[TEMP].key[V_KEY_PRESS].d - video_data[TEMP].key[KEY_PRESS].a) * 20000 * chassis_speed_buff; // 系数待测
    chassis_cmd_send.vy = (video_data[TEMP].key[V_KEY_PRESS].w - video_data[TEMP].key[KEY_PRESS].s) * 20000 * chassis_speed_buff;
    chassis_cmd_send.wz = video_data[TEMP].key[V_KEY_PRESS].shift * 14000 * chassis_speed_buff;

    gimbal_cmd_send.yaw -= (float)video_data[TEMP].key_data.mouse_x / 660 * 2.5; // 系数待测
    gimbal_cmd_send.pitch += (float)video_data[TEMP].key_data.mouse_y / 660 * 2.5;
    ```
