//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"
#include "Mdrv.h"
#include "math.h"
#include "motion_control.h"
#include "usart.h"
#include "string.h" // strlen

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET

uint8_t GetLineWalking() {
    uint8_t state = 0;
    if (HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Track_L2X1_Pin) == High) state |= (1 << 3); // L2
    if (HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Track_L1X2_Pin) == High) state |= (1 << 2); // L1
    if (HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Track_R1X3_Pin) == High) state |= (1 << 1); // R1
    if (HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Track_R2X4_Pin) == High) state |= (1 << 0); // R2
    return state;
}

/* 车体几何参数（单位 mm） */
#define TRACK_WIDTH   135.0f   // 左右轮中心距离
#define WHEEL_BASE    145.0f   // 前后轴中心距离
int8_t before_state = -1; // 上一次状态


void LineWalking(uint8_t state, int16_t speed) {
    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2
    // Notice: 已知1.7mm的间距黑胶不会有 0011 和 1100 的情况 行进路线放置
    // 为什么这里逆时针为正

    // 拆位（为清晰起见）
    uint8_t L2 = (state >> 3) & 0x01;  // 左外
    uint8_t L1 = (state >> 2) & 0x01;  // 左内
    uint8_t R1 = (state >> 1) & 0x01;  // 右内
    uint8_t R2 = (state >> 0) & 0x01;  // 右外


    // 严格匹配原逻辑的优先级顺序（从强烈转向到微调，再到直行）
    // bug 聚集地
    if (L2 == Low && R2 == Low && L1 == Low && R1 == Low) {                     // 0000
        Motion_SpinTaskStart(-45.0f,150);
    }else if (L2 == High && R2 == High && L1 == High && R1 == High) {           // 1111
        // 四个传感器都没看到线（
        if (before_state == -1)
            Motion_Forward(speed); // 初见白色向前走
        else if (before_state == 9) {
            // 如果之前是直走状态，继续前进
            Motion_Forward(speed);
        } else if (before_state == 1) {
            // 如果之前是转弯状态，继续转弯
            Motion_SpinTaskStart(45.0f, speed); // 继续转弯
        } else {
            Motion_Backward(100); // 否则后退
        }
    }else if ((L1 == Low || R1 == Low ) && R2 == Low) {                         // x0x0 / xx00
        // 右直角（强右偏）：右大弯
        Motion_SpinTaskStart(-40.0f, speed);
    }else if (L2 == Low && (R1 == Low || L1 == Low)) {                          // 0x0x / 00xx
        // 左直角（强左偏）：左大弯
        Motion_SpinTaskStart(40.0f, speed);
    }else if (L2 == Low) {                                                      // 0xxx
        // 左最外侧感应器触发
        Motion_SpinTaskStart(10.0f,speed);
    }else if (R2 == Low) {                                                      // xxx0
        // 右最外侧感应器触发
        Motion_SpinTaskStart(-10.0f, speed);
    }else if (L1 == Low && R1 == Low) {
        // 正常行进：直走
        Motion_Forward(speed);
    }
    else if (L1 == Low && R1 == High) {
        // 中央偏左（微调）
        // MDrv_SetSpeed(0, 0, 75, 75); // 左轮减速，右轮加速
        // Motion_SpinTaskStart(-5.0f, 85);
        Motion_LeftTurnByAngdps(speed, 2);
    }
    else if (L1 == High && R1 == Low) {
        // 中央偏右（微调）
        // MDrv_SetSpeed(75, 75, 0, 0); // 左轮加速，右轮减速
        // Motion_SpinTaskStart(5.0f, 85);
        Motion_LeftTurnByAngdps(speed, 2);
    }
    else {
        // 上面已经全覆盖了,所有16种情况
        Motion_Backward(speed);
    }

    if (before_state != state) {
        // 如果状态变化了，发送状态变化信息
        before_state = state;
    }
}
