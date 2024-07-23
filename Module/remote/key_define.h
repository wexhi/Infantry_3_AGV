#ifndef KEY_DEFINE_H
#define KEY_DEFINE_H

#include "stdint.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS            0
#define KEY_STATE            1
#define KEY_PRESS_WITH_CTRL  1
#define KEY_PRESS_WITH_SHIFT 2

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W     0
#define Key_S     1
#define Key_A     2
#define Key_D     3
#define Key_Shift 4
#define Key_Ctrl  5
#define Key_Q     6
#define Key_E     7
#define Key_R     8
#define Key_F     9
#define Key_G     10
#define Key_Z     11
#define Key_X     12
#define Key_C     13
#define Key_V     14
#define Key_B     15

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度
#pragma pack(1)
typedef union {
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t a : 1;
        uint16_t d : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;
#endif // !KEY_DEFINE_H