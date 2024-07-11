#include "UARTComm.h"
#include "memory.h"
#include "stdlib.h"
#include "crc8.h"
#include "bsp_dwt.h"

static UARTComm_Instance *ucomm;

static void UARTCommRxCallback(void)
{
    /* 当前接收状态判断 */
    if (ucomm->uart_instance->recv_buff[0] == UARTCOMM_HEADER && ucomm->recv_state == 0) {
        if (ucomm->uart_instance->recv_buff[1] == ucomm->recv_data_len) // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
        {
            ucomm->recv_state = 1; // 设置接收状态为1,说明已经开始接收
        } else
            return; // 直接跳过即可
    }
    if (ucomm->recv_state) // 已经收到过帧头
    {
        if (ucomm->uart_instance->recv_buff[ucomm->recv_buf_len - 1] == UARTCOMM_TAIL) // 如果帧尾正确
        {
            uint8_t crc8 = crc_8(ucomm->uart_instance->recv_buff + 2, ucomm->recv_data_len); // 计算crc8
            if (crc8 == ucomm->uart_instance->recv_buff[ucomm->recv_buf_len - 2])            // 如果crc8正确
            {
                memcpy(ucomm->unpacked_recv_data, ucomm->uart_instance->recv_buff + 2, ucomm->recv_data_len); // 拷贝数据
                DaemonReload(ucomm->ucomm_daemon);
            }
        }
    }
    ucomm->recv_state = 0; // 接收状态重置
}

static void UARTCommLostCallback(void *arg)
{
    USARTServiceInit(ucomm->uart_instance);
}

UARTComm_Instance *UARTCommInit(UARTComm_Init_Config_s *config)
{
    ucomm = (UARTComm_Instance *)malloc(sizeof(UARTComm_Instance));
    memset(ucomm, 0, sizeof(UARTComm_Instance));

    ucomm->recv_data_len                                                  = config->recv_data_len;
    ucomm->recv_buf_len                                                   = config->recv_data_len + UARTCOMM_OFFSET_BYTES; //'s' + data length + crc8 + 'e'
    ucomm->send_data_len                                                  = config->send_data_len;
    ucomm->send_buf_len                                                   = config->send_data_len + UARTCOMM_OFFSET_BYTES;
    ucomm->raw_sendbuf[0]                                                 = UARTCOMM_HEADER;       // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ucomm->raw_sendbuf[1]                                                 = config->send_data_len; // datalen
    ucomm->raw_sendbuf[config->send_data_len + UARTCOMM_OFFSET_BYTES - 1] = UARTCOMM_TAIL;         // tail
    // usart_instance
    USART_Init_Config_s usart_config;
    usart_config.module_callback = UARTCommRxCallback;
    usart_config.usart_handle    = config->uart_handle;
    usart_config.recv_buff_size  = ucomm->recv_buf_len;
    ucomm->uart_instance         = USARTRegister(&usart_config);

    Daemon_Init_Config_s daemon_conf = {
        .callback     = UARTCommLostCallback,
        .owner_id     = ucomm,
        .reload_count = config->daemon_counter,
    };
    ucomm->ucomm_daemon = DaemonRegister(&daemon_conf);

    return ucomm;
}

void UARTCommSend(UARTComm_Instance *ins, uint8_t *send_data)
{
    static uint8_t crc8;
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(ins->raw_sendbuf + 2, send_data, ins->send_data_len);
    crc8                                     = crc_8(send_data, ins->send_data_len);
    ins->raw_sendbuf[2 + ins->send_data_len] = crc8;
    // 发送
    USARTSend(ucomm->uart_instance, ins->raw_sendbuf, ins->send_buf_len, USART_TRANSFER_BLOCKING);
}

void *UARTCommGet(UARTComm_Instance *instance)
{
    return instance->unpacked_recv_data;
}
