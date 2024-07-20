#include "miniPC_process.h"
#include "string.h"
#include "robot_def.h"
#include "crc_ref.h"
#include "daemon.h"
#include <math.h>

static Vision_Instance *vision_instance; // 用于和视觉通信的串口实例
static uint8_t *vis_recv_buff __attribute__((unused));
static Daemon_Instance *vision_daemon_instance;
// 全局变量区
extern uint16_t CRC_INIT;

// 标准化角度到0到360度范围内
static float StandardizeAngle(float angle)
{
    float mod_angle = fmod(angle, 360.f);
    if (mod_angle < 0) {
        mod_angle += 360.0;
    }
    return mod_angle;
}

// 计算并返回新的 total_angle，使其接近目标角度
static float AdjustNearestAngle(float total_angle, float target_angle)
{
    // 标准化当前角度
    float standard_current_angle = StandardizeAngle(total_angle);
    // 计算角度差
    float delta_angle = target_angle - standard_current_angle;
    // 调整角度差到 -180 到 180 范围内
    if (delta_angle > 180.f) {
        delta_angle -= 360.f;
    } else if (delta_angle < -180.f) {
        delta_angle += 360.f;
    }
    // 计算新的目标 totalangle
    return total_angle + delta_angle;
}

/**
 * @brief 处理视觉传入的数据
 *
 * @param recv
 * @param rx_buff
 */
static void RecvProcess(Vision_Recv_s *recv, uint8_t *rx_buff)
{
    /* 使用memcpy接收浮点型小数 */
    recv->is_tracking = rx_buff[1];

    recv->is_shooting = rx_buff[2];
    memcpy(&recv->yaw, &rx_buff[3], 4);
    memcpy(&recv->pitch, &rx_buff[7], 4);
    memcpy(&recv->distance, &rx_buff[11], 4);

    /* 视觉数据处理 */
    float yaw_total  = vision_instance->send_data->yaw; // 保存当前总角度
    float yaw_target = recv->yaw;
    recv->yaw        = AdjustNearestAngle(yaw_total, yaw_target);
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    // memset(vision_instance->recv_data + 1, 0, sizeof(Vision_Recv_s) - 1);
#ifdef VISION_USE_UART
    USARTServiceInit(vision_instance->usart);
#endif // !VISION_USE_UART
}

/**
 * @brief
 *
 * @param yaw
 * @param pitch
 * @param roll
 * @param bullet_speed
 */
void VisionSetAltitude(float yaw, float pitch, float roll, float bullet_speed, float yaw_speed)
{
    vision_instance->send_data->yaw       = yaw;
    vision_instance->send_data->pitch     = pitch;
    vision_instance->send_data->roll      = roll;
    vision_instance->send_data->yaw_speed = yaw_speed;
    if (bullet_speed > 0) {
        vision_instance->send_data->bullet_speed = bullet_speed;
    } else {
        vision_instance->send_data->bullet_speed = 25;
    }
}

/**
 * @brief 设置是否击打能量机关
 *
 * @param is_energy_mode 0-默认瞄准装甲板，1-瞄准能量机关
 */
void VisionSetEnergy(uint8_t is_energy_mode)
{
    vision_instance->send_data->is_energy_mode = is_energy_mode;
}

/**
 * @brief 发送数据处理函数
 *
 * @param send 待发送数据
 * @param tx_buff 发送缓冲区
 *
 */
static void SendProcess(Vision_Send_s *send, uint8_t *tx_buff)
{
    /* 发送帧头，目标颜色，是否重置等数据 */
    tx_buff[0] = send->header;
    tx_buff[1] = send->is_energy_mode;
    tx_buff[2] = send->detect_color;
    // tx_buff[3] = send->reset_tracker;
    // tx_buff[4] = 1;

    /* 使用memcpy发送浮点型小数 */
    memcpy(&tx_buff[3], &send->roll, 4);
    memcpy(&tx_buff[7], &send->yaw, 4);
    memcpy(&tx_buff[11], &send->pitch, 4);
    memcpy(&tx_buff[15], &send->bullet_speed, 4);
    memcpy(&tx_buff[19], &send->yaw_speed, 4);

    /* 发送校验位 */
    send->checksum = Get_CRC16_Check_Sum(&tx_buff[0], VISION_SEND_SIZE - 3u, CRC_INIT);
    memcpy(&tx_buff[VISION_SEND_SIZE - 3u], &send->checksum, 2);
    memcpy(&tx_buff[VISION_SEND_SIZE - 1u], &send->tail, 1);
}

/**
 * @brief 用于注册一个视觉接收数据结构体,返回一个视觉接收数据结构体指针
 *
 * @param recv_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionRecvRegister(Vision_Recv_Init_Config_s *recv_config)
{
    Vision_Recv_s *recv_data = (Vision_Recv_s *)malloc(sizeof(Vision_Recv_s));
    memset(recv_data, 0, sizeof(Vision_Recv_s));

    recv_data->header = recv_config->header;

    return recv_data;
}

/**
 * @brief 用于注册一个视觉发送数据结构体,返回一个视觉发送数据结构体指针
 *
 * @param send_config
 * @return Vision_Send_s*
 */
Vision_Send_s *VisionSendRegister(Vision_Send_Init_Config_s *send_config)
{
    Vision_Send_s *send_data = (Vision_Send_s *)malloc(sizeof(Vision_Send_s));
    memset(send_data, 0, sizeof(Vision_Send_s));

    send_data->header       = send_config->header;
    send_data->detect_color = send_config->detect_color;
    send_data->tail         = send_config->tail;
    return send_data;
}

#ifdef VISION_USE_UART

static void RadarDecode(void)
{
    uint16_t judge_length;                         // 统计一帧数据长度
    if (vision_instance->usart->recv_buff == NULL) // 空数据包，则不作任何处理
        return;
}

/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision()
{
    if (vision_instance->usart->recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        RecvProcess(vision_instance->recv_data, vision_instance->usart->recv_buff);
    }
}

/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size  = VISION_RECV_SIZE;
    conf.usart_handle    = video_usart_handle;

    vision_instance->usart                = USARTRegister(&conf);
    Vision_Recv_Init_Config_s recv_config = {
        .header = VISION_RECV_HEADER,
    };

    vision_instance->recv_data            = VisionRecvRegister(&recv_config);
    Vision_Send_Init_Config_s send_config = {
        .header        = VISION_SEND_HEADER,
        .detect_color  = VISION_DETECT_COLOR_RED,
        .reset_tracker = VISION_RESET_TRACKER_NO,
        .is_shoot      = VISION_SHOOTING,
        .tail          = VISION_SEND_TAIL,
    };
    vision_instance->send_data = VisionSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USARTSend(vision_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}
#endif

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
/**
 * @brief 回调函数，确认帧头后用于解析视觉数据
 *
 */
static void DecodeVision(uint16_t var)
{
    UNUSED(var); // 仅为了消除警告
    if (vis_recv_buff[0] == vision_instance->recv_data->header) {
        // 读取视觉数据
        /* 接收校验位 */
        memcpy(&vision_instance->recv_data->checksum, &vis_recv_buff[VISION_RECV_SIZE - 2], 2);
        if (vision_instance->recv_data->checksum == Get_CRC16_Check_Sum(vis_recv_buff, VISION_RECV_SIZE - 2, CRC_INIT)) {
            DaemonReload(vision_daemon_instance);
            RecvProcess(vision_instance->recv_data, vis_recv_buff);
        } else {
            memset(vision_instance->recv_data, 0, sizeof(Vision_Recv_s));
        }
    }
}
/**
 * @brief 用于注册一个视觉通信模块实例,返回一个视觉接收数据结构体指针
 *
 * @param init_config
 * @return Vision_Recv_s*
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *video_usart_handle)
{
    UNUSED(video_usart_handle); // 仅为了消除警告
    vision_instance = (Vision_Instance *)malloc(sizeof(Vision_Instance));
    memset(vision_instance, 0, sizeof(Vision_Instance));
    Vision_Recv_Init_Config_s recv_config = {
        .header = VISION_RECV_HEADER,
    };

    USB_Init_Config_s conf     = {.rx_cbk = DecodeVision};
    vis_recv_buff              = USBInit(conf);
    recv_config.header         = VISION_RECV_HEADER;
    vision_instance->recv_data = VisionRecvRegister(&recv_config);

    Vision_Send_Init_Config_s send_config = {
        .header       = VISION_SEND_HEADER,
        .detect_color = VISION_DETECT_COLOR_RED,
        .tail         = VISION_SEND_TAIL,
    };
    vision_instance->send_data = VisionSendRegister(&send_config);
    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback     = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id     = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);
    return vision_instance->recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];
    SendProcess(vision_instance->send_data, send_buff);
    USBTransmit(send_buff, VISION_SEND_SIZE);
}

#endif