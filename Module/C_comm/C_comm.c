#include "C_comm.h"
#include "memory.h"
#include "stdlib.h"
#include "crc8.h"
#include "bsp_dwt.h"
/* ---------------------------------------- 私有函数声明  ------------------------------------- */
static void CANCommResetRx(CAN_Comm_Instance *ins);
static void CANCommRxCallback(CAN_Instance *can);
static void CANCommLostCallback(void *cancomm);

    /* ---------------------------------------- 公有函数实现  ------------------------------------- */
    /**
     * @brief 初始化CANComm
     *
     * @param config CANComm初始化结构体
     * @return CANCommInstance*
     */
    CAN_Comm_Instance *CANCommInit(CAN_Comm_Init_Config_s *comm_config)
{
    CAN_Comm_Instance *comm_instance = (CAN_Comm_Instance *)malloc(sizeof(CAN_Comm_Instance));
    memset(comm_instance, 0, sizeof(CAN_Comm_Instance));

    comm_instance->recv_data_len = comm_config->recv_data_len;
    comm_instance->recv_buff_len = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
    comm_instance->send_data_len = comm_config->send_data_len;
    comm_instance->send_buff_len = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail

    comm_instance->raw_send_buff[0]                                = CAN_COMM_HEADER;            // 帧头,直接设置避免每次发送时都要重新赋值
    comm_instance->raw_send_buff[1]                                = comm_config->send_data_len; // 数据长度,直接设置避免每次发送时都要重新赋值
    comm_instance->raw_send_buff[comm_instance->send_buff_len - 1] = CAN_COMM_TAIL;              // 帧尾,直接设置避免每次发送时都要重新赋值

    // CAN初始化
    comm_config->can_config.id                  = comm_instance; // CANComm的实例指针作为CANInstance的id,回调函数中会用到
    comm_config->can_config.can_module_callback = CANCommRxCallback; // 设置CAN回调函数
    comm_instance->can_instance                 = CANRegister(&comm_config->can_config);

    // 守护进程初始化
    Daemon_Init_Config_s daemon_config = {
        .callback     = CANCommLostCallback,
        .owner_id     = (void *)comm_instance,
        .reload_count = comm_config->daemon_count,
    };
    comm_instance->comm_daemon = DaemonRegister(&daemon_config);

    return comm_instance;
}

/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 */
void CANCommSend(CAN_Comm_Instance *instance, uint8_t *data)
{
    static uint8_t crc8;
    static uint8_t send_len;
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_send_buff + 2, data, instance->send_data_len);
    crc8                                                 = crc_8(data, instance->send_data_len);
    instance->raw_send_buff[2 + instance->send_data_len] = crc8;

    // 发送数据,CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buff_len; i += 8) {
        // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        send_len = instance->send_buff_len - i >= 8 ? 8 : instance->send_buff_len - i;
        CANSetDLC(instance->can_instance, send_len);
        memcpy(instance->can_instance->tx_buff, instance->raw_send_buff + i, send_len);
        CANTransmit(instance->can_instance, 1);
    }
}

/**
 * @brief 获取CANComm接收的数据,需要自己使用强制类型转换将返回的void指针转换成指定类型
 *
 * @return void* 返回的数据指针
 * @attention 注意如果希望直接通过转换指针访问数据,如果数据是union或struct,要检查是否使用了pack(n)
 *            CANComm接收到的数据可以看作是pack(1)之后的,是连续存放的.
 *            如果使用了pack(n)可能会导致数据错乱,并且无法使用强制类型转换通过memcpy直接访问,转而需要手动解包.
 *            强烈建议通过CANComm传输的数据使用pack(1)
 */
void *CANCommGet(CAN_Comm_Instance *instance)
{
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

/* ---------------------------------------- 私有函数实现  ------------------------------------- */

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CAN_Comm_Instance *ins)
{
    // 当前已经收到的buffer清零
    memset(ins->raw_recv_buff, 0, ins->cur_recv_len);
    ins->recv_flag    = 0; // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

static void CANCommRxCallback(CAN_Instance *can)
{
    CAN_Comm_Instance *comm = (CAN_Comm_Instance *)can->id; // 将can instance的id强制转换为CANCommInstance*类型

    /* 当前接收状态判断 */
    if (can->rx_buff[0] == CAN_COMM_HEADER && comm->recv_flag == 0) // 之前尚未开始接收且此次包里第一个位置是帧头
    {
        if (can->rx_buff[1] == comm->recv_data_len) // 数据长度正确,暂不支持动态包长
            comm->recv_flag = 1;                    // 接收状态置1,表示开始接收
        else
            return; // 数据长度不正确,直接返回
    }

    if (comm->recv_flag) // 已经验证过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (comm->cur_recv_len + can->rx_len > comm->recv_buff_len) {
            CANCommResetRx(comm);
            return; // 直接返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(comm->raw_recv_buff + comm->cur_recv_len, can->rx_buff, can->rx_len);
        comm->cur_recv_len += can->rx_len; // 当前接收到的长度增加

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (comm->cur_recv_len == comm->recv_buff_len) {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (comm->raw_recv_buff[comm->recv_buff_len - 1] == CAN_COMM_TAIL) {
                // 通过校验,复制数据到unpack_data中
                if (comm->raw_recv_buff[comm->recv_buff_len - 2] == crc_8(comm->raw_recv_buff + 2, comm->recv_data_len)) {
                    // 数据量大的话考虑使用DMA
                    memcpy(comm->unpacked_recv_data, comm->raw_recv_buff + 2, comm->recv_data_len);
                    comm->update_flag = 1;           // 数据更新标志位置1
                    DaemonReload(comm->comm_daemon); // 重载守护进程
                }
            }
            CANCommResetRx(comm); // 重置接收状态
            return;
        }
    }
}

static void CANCommLostCallback(void *cancomm)
{
    CAN_Comm_Instance *comm = (CAN_Comm_Instance *)cancomm;
    CANCommResetRx(comm);
}