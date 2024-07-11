/**
 * @file C_comm.c
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   用于多块C板之间的CAN通信
 * @version 0.1
 * @date 2024-01-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef C_COMM_H
#define C_COMM_H

#include "bsp_can.h"
#include "daemon.h"

#define CAN_COMM_MAX_BUFFSIZE 60  // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER       's' // 帧头
#define CAN_COMM_TAIL         'e' // 帧尾
#define CAN_COMM_OFFSET_BYTES 4   // 's'+ datalen + 'e' + crc8

#pragma pack(1)

/* C板通信实例 */
typedef struct
{
    CAN_Instance *can_instance; // 用于和CAN通信的CAN实例

    /* 发送部分 */
    uint8_t send_data_len;                                                // 发送数据长度
    uint8_t send_buff_len;                                                // 发送缓冲区长度
    uint8_t raw_send_buff[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 发送缓冲区，额外预留4字节用于帧头帧尾和校验位

    /* 接收部分 */
    uint8_t recv_data_len;                                                // 接收数据长度
    uint8_t recv_buff_len;                                                // 接收缓冲区长度
    uint8_t raw_recv_buff[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 接收缓冲区，额外预留4字节用于帧头帧尾和校验位
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];                    // 解包后的数据

    /* 接收和更新标志位 */
    uint8_t recv_flag;    // 接收标志位
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    uint8_t update_flag;  // 数据更新标志位,当接收到新数据时,会将此标志位置1,调用CANCommGet()后会将此标志位置0

    Daemon_Instance *comm_daemon;
} CAN_Comm_Instance;
#pragma pack()

/* 板间通信初始化结构体 */
typedef struct
{
    CAN_Init_Config_s can_config; // CAN初始化结构体
    uint8_t send_data_len;        // 发送数据长度
    uint8_t recv_data_len;        // 接收数据长度

    uint16_t daemon_count; // 守护进程计数,用于初始化守护进程
} CAN_Comm_Init_Config_s;

/**
 * @brief 初始化CANComm
 *
 * @param config CANComm初始化结构体
 * @return CANCommInstance*
 */
CAN_Comm_Instance *CANCommInit(CAN_Comm_Init_Config_s *comm_config);

/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 */
void CANCommSend(CAN_Comm_Instance *instance, uint8_t *data);

/**
 * @brief 获取CANComm接收的数据,需要自己使用强制类型转换将返回的void指针转换成指定类型
 *
 * @return void* 返回的数据指针
 * @attention 注意如果希望直接通过转换指针访问数据,如果数据是union或struct,要检查是否使用了pack(n)
 *            CANComm接收到的数据可以看作是pack(1)之后的,是连续存放的.
 *            如果使用了pack(n)可能会导致数据错乱,并且无法使用强制类型转换通过memcpy直接访问,转而需要手动解包.
 *            强烈建议通过CANComm传输的数据使用pack(1)
 */
void *CANCommGet(CAN_Comm_Instance *instance);

#endif // C_COMM_H