<!-- 
 * @file super_cap.h
 * @author Bi KaiXiang (wexhi@qq.com)
 * @brief
 * @version 0.3
 * @date 2024-08-30
 *
 * @copyright Copyright (c) 2024
 *
 -->

# super_cap  

## 代码结构

考虑到当前还有许多代码未使用bsp_can的框架，因此在此处还会进行drv_can的兼容，请自行查看自己代码中的板级CAN代码为bsp_can(新框架)或~~drv_can(老代码)~~

## 以下为bsp_can的代码结构

.h中放置的是数据定义和外部接口，以及协议的定义和宏，.c中包含一些私有函数。

## 外部接口

``` c
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

// 下为舵轮中超电所需的函数
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
```

## 私有函数和变量

``` c
static SuperCap_Instance *super_cap_instance = NULL;
static void SuperCapRxCallback(CAN_Instance *_instance);
static void SuperCapLostCallback(void *cap_ptr);
```

## 如何使用？

1. 在需要使用超级电容的文件中包含头文件并且可以创建一个超级电容实例指针以更方便的查看数据

    ``` c
    #include "super_cap.h"
    static SuperCap_Instance *super_cap;                                 // 超级电容实例

    ```

2. 在需要使用超级电容的地方初始化超级电容

    ``` c
    SuperCap_Init_Config_s super_cap_config = {
            .can_config = {
                .can_handle = &hcan1,
                .tx_id      = 0x302,
                .rx_id      = 0x301,
            },
        };
        super_cap = SuperCapInit(&super_cap_config); // 超级电容初始化
    ```

3. 在需要发送超级电容数据的地方设置超级电容数据

    ``` c
    SuperCapSet(a, b, c); // 设置超级电容数据, a为缓冲能量, b为底盘功率, c为设置超级电容状态
    ```

4. 在需要发送超级电容数据的地方发送超级电容数据

    ``` c
    SuperCapSend(); // 发送超级电容数据
    ```

5. 在舵轮中，因为一条CAN线上已经挂载了8个电机，无法再挂载超级电容，因此超电需要挂在另一条CAN线上，此时超电无法读取电机电流数据，我们还需额外设置4个3508电机的电流值，并发送

    ``` c
    SuperCapSetMotor(motor_lf->measure.real_current, motor_rf->measure.real_current, motor_lb->measure.real_current, motor_rb->measure.real_current);
    SuperCapMotorSend();
    ```

## ~~以下为drv_can的代码~~

后续**不会维护**drv_can的代码，建议使用bsp_can的代码

## 发送数据

1. 在drv_can.c中定义发送函数

    ``` c
    void can_remote(uint8_t tx_buff[], CAN_HandleTypeDef *_can_ins, uint32_t can_send_id, uint32_t len) // 调用can来发送遥控器数据
    {
    CAN_TxHeaderTypeDef tx_header;

    tx_header.StdId = can_send_id;                         // 如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
    tx_header.IDE = CAN_ID_STD;                            // 标准帧
    tx_header.RTR = CAN_RTR_DATA;                          // 数据帧
    tx_header.DLC = len;                                   // 发送数据长度（字节）
    while (HAL_CAN_GetTxMailboxesFreeLevel(_can_ins) == 0) // 等待邮箱空闲
    {
    }
    HAL_CAN_AddTxMessage(_can_ins, &tx_header, tx_buff, (uint32_t *)CAN_TX_MAILBOX0);
    }
    ```

2. 在需要使用超级电容数据的地方使用

    ``` c
    // 在需要使用超级电容数据的地方使用
    static uint8_t tx2up[8];   // 用于发送数据给上位机
    while (1)
    {
        // temp为缓冲能量，temp1为底盘功率，cap_state_cmd为超级电容状态,请自行修改
    tx2up[0] = (temp >> 8) & 0xff;
    tx2up[1] = temp & 0xff;
    tx2up[2] = (temp1 >> 8) & 0xff;
    tx2up[3] = temp1 & 0xff;
    tx2up[4] = cap_state_cmd;
    can_remote(tx2up, &hcan1, 0x302, 5);
    osDelay(5);
    }
    ```

## 接收数据

在drv_can.c的回调函数中添加接收数据的处理

``` c

#define POWERDATA_ID 0x301

uint16_t cap_voltage = 0;
uint16_t chassis_power = 0;
uint16_t cap_state = 0;
CAN_RxHeaderTypeDef rx_header1;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    uint8_t rx_data1[8];
     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1, rx_data1); // receive can2 data
         // 超级电容接收
    if (rx_header1.StdId == POWERDATA_ID) // 0x301
    {
      cap_voltage = (((uint16_t)rx_data1[0] << 8) | rx_data1[1]) / 1000;
      chassis_power = (uint16_t)(rx_data1[2] << 8) | rx_data1[3] / 1000;
      cap_state = rx_data1[4];
    }
}
```
