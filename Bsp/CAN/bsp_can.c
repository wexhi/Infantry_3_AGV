#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"

// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CAN_Instance *can_instances[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* ----------------------------------- 以下为私有函数 ----------------------------------------------- */

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note  此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
static void CANServiceInit(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *
 * @param _instance can instance owned by specific module
 */
static void CANAddFilter(CAN_Instance *_instance)
{
    CAN_FilterTypeDef can_filter_conf;
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

    can_filter_conf.FilterMode           = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
    can_filter_conf.FilterScale          = CAN_FILTERSCALE_16BIT;                                                       // 使用16位id模式,即只有低16位有效
    can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;                        // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
    can_filter_conf.SlaveStartFilterBank = 14;                                                                          // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
    can_filter_conf.FilterIdLow          = _instance->rx_id << 5;                                                       // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
    can_filter_conf.FilterBank           = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++); // 根据can_handle判断是CAN1还是CAN2,然后自增
    can_filter_conf.FilterActivation     = CAN_FILTER_ENABLE;                                                           // 启用过滤器

    HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf);
}

/* -------------------- 以下为公有函数 ---------------------- */

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CAN_Instance *CANRegister(CAN_Init_Config_s *config)
{
    if (!idx) {
        CANServiceInit(); // 初始化CAN总线
    }
    if (idx >= CAN_MX_REGISTER_CNT) {
        // 超出最大注册数量
        return NULL;
    }
    for (uint8_t i = 0; i < idx; ++i) {
        if (can_instances[i]->can_handle == config->can_handle && can_instances[i]->rx_id == config->rx_id) {
            // 重复注册
            return NULL;
        }
    }

    // 初始化实例
    CAN_Instance *can = (CAN_Instance *)malloc(sizeof(CAN_Instance));
    memset(can, 0, sizeof(CAN_Instance));

    // 配置发送报文
    can->txconf.StdId = config->tx_id; // 标准发送ID
    can->txconf.IDE   = CAN_ID_STD;    // 标准帧,使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    can->txconf.RTR   = CAN_RTR_DATA;  // 数据帧
    can->txconf.DLC   = 0x08;          // 数据长度,最大为8

    // 设置回调函数和接收发送id
    can->can_handle          = config->can_handle;          // can句柄
    can->tx_id               = config->tx_id;               // 发送id,(好像没用到)
    can->rx_id               = config->rx_id;               // 接收id
    can->can_module_callback = config->can_module_callback; // 回调函数
    can->id                  = config->id;                  // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入

    CANAddFilter(can); // 添加CAN的过滤规则

    can_instances[idx++] = can; // 添加到实例数组

    return can; // 返回实例指针
}

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 *
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CAN_Instance *_instance, float timeout)
{
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused)); // for cancel warning
    float dwt_start = DWT_GetTimeline_ms();

    while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
        {
            busy_count++;
            return 0;
        }
    }

    wait_time = DWT_GetTimeline_ms() - dwt_start;

    if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox)) {
        busy_count++;
        return 0;
    }

    return 1; // 发送成功
}

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CAN_Instance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        return;                    // 不执行
    _instance->txconf.DLC = length;
}

/* ----------------------- 回调函数定义 --------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx; ++i) {                        // 两者相等说明这是要找的实例
            if (_hcan == can_instances[i]->can_handle && rxconf.StdId == can_instances[i]->rx_id) {
                if (can_instances[i]->can_module_callback != NULL) // 回调函数不为空就调用
                {
                    can_instances[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                    memcpy(can_instances[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_instances[i]->can_module_callback(can_instances[i]);    // 触发回调进行数据解析和处理
                }
                return;
            }
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 */
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}