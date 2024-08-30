# 与视觉通信协议

可能后续会有更新，以视觉那边为准，下为24赛季复活赛视觉通信协议

## 注意，代码中有`#pragma pack(1) // 1字节对齐`否则无法与视觉数据对齐，结尾还应加上`#pragma pack() // 取消1字节对齐`

## 1. 接收视觉数据的结构体

``` c
    typedef struct
    {
        uint8_t header;      // 帧头，0xA5u
        uint8_t is_tracking; // 是否追踪到目标
        uint8_t is_shooting; // 是否发射
        float yaw;           // 目标Yaw角度
        float pitch;         // 目标Pitch角度
        float distance;      // 目标距离
        uint16_t checksum;
    } Vision_Recv_s;
```

## 2. 发送给视觉的数据结构体

``` c
    typedef struct
    {
        uint8_t header;
        uint8_t is_energy_mode; // 0-瞄准装甲板，1-瞄准能量机关
        uint8_t detect_color;   // 5-red 6-blue 发1
        uint8_t is_reset;       // 是否重置目标
        float roll;             // rad
        float yaw;              // rad
        float pitch;            //
        float bullet_speed;     // 弹速
        float yaw_speed;        // yaw速度
        uint16_t checksum;      // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
        uint8_t tail;           // 尾帧校验位
    } Vision_Send_s;
```

## 3. 如何使用？

1. 初始化与上位机通信的串口(当前代码为虚拟串口，后续如若使用串口，还需修改`robot_def.h`中的相关宏定义)

    ``` c
    #include "miniPC_process.h"
    static Vision_Recv_s *vision_ctrl; // 视觉控制信息
    vision_ctrl     = VisionInit(&huart1); // 初始化视觉控制
    ```

2. 设置给视觉发送的数据

    ``` c
    // 设置视觉发送数据,还需增加加速度和角速度数据
    static float yaw, pitch, roll, bullet_speed, yaw_speed;
    yaw          = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
    pitch        = gimbal_fetch_data.gimbal_imu_data.Roll;
    roll         = gimbal_fetch_data.gimbal_imu_data.Pitch;
    bullet_speed = chassis_fetch_data.bullet_speed;
    yaw_speed    = gimbal_fetch_data.gimbal_imu_data.Gyro[2];

    VisionSetDetectColor(chassis_fetch_data.self_color);
    VisionSetAltitude(yaw, pitch, roll, bullet_speed, yaw_speed);数据
    ```

3. 在控制命令中处理视觉的数据(以遥控器键鼠控制为例：)

    ``` c
    if (vision_ctrl->is_tracking) {     // 当识别到目标
        if (vision_ctrl->is_shooting) { // 如果可以开火，设置瞄准目标的标志位，应用于UI
            chassis_cmd_send.vision_mode = LOCK;
            gimbal_cmd_send.vision_mode  = LOCK;
        } else {
            chassis_cmd_send.vision_mode = UNLOCK;
            gimbal_cmd_send.vision_mode  = UNLOCK;
        }
        if (rc_data[TEMP].mouse.press_r) // 右键开启自瞄
        {
            gimbal_cmd_send.yaw   = (vision_ctrl->yaw == 0 ? gimbal_cmd_send.yaw : vision_ctrl->yaw);
            gimbal_cmd_send.pitch = (vision_ctrl->pitch == 0 ? gimbal_cmd_send.pitch : vision_ctrl->pitch);
        }
    } else {
        chassis_cmd_send.vision_mode = UNLOCK;
        gimbal_cmd_send.vision_mode  = UNLOCK;
    }
    ```
