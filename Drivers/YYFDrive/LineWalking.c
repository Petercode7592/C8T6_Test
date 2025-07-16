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
uint8_t before_state = -1; // 上一次状态


void LineWalking(uint8_t state, int16_t speed) {
    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2
    // Notice: 已知1.7mm的间距黑胶不会有 0011 和 1100 的情况 行进路线放置
    char txBuffer[64];

    // 拆位（为清晰起见）
    uint8_t L2 = (state >> 3) & 0x01;  // 左外
    uint8_t L1 = (state >> 2) & 0x01;  // 左内
    uint8_t R1 = (state >> 1) & 0x01;  // 右内
    uint8_t R2 = (state >> 0) & 0x01;  // 右外

    // 拼接字符串（可用 snprintf 更安全）
    sprintf(txBuffer, "$LWK:%d%d %d%d#\r\n", L2, L1, R1, R2);
    // 通过蓝牙发送
    HAL_UART_Transmit(&Bluetooth_Cfg, (uint8_t *)txBuffer, strlen(txBuffer), HAL_MAX_DELAY);

    // 严格匹配原逻辑的优先级顺序（从强烈转向到微调，再到直行）
    // L1 = 0 R2 = 0
    if (L2 == Low && R2 == Low && L1 == Low && R1 == Low) {
        // 四个传感器都看到线（后退）
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Backward#\r\n", 18, HAL_MAX_DELAY);
        Motion_SpinTaskStart(20.0f,100); // 这里的速度可以根据实际情况调整 bug
    }
    else if (L2 == High && R2 == High && L1 == High && R1 == High) {
        // 四个传感器都没看到线（
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Stop#\r\n", 14, HAL_MAX_DELAY);
        Motion_Backward(speed); // bug
    }
    else if ((L1 == Low || R1 == Low ) && R2 == Low) {
        // 右直角（强右偏）：右大弯
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Right Turn#\r\n", 20, HAL_MAX_DELAY);
        Motion_SpinTaskStart(40.0f, speed);
    }
    else if (L2 == Low && (R1 == Low || L1 == Low)) {
        // 左直角（强左偏）：左大弯
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Left Turn#\r\n", 19, HAL_MAX_DELAY);
        Motion_SpinTaskStart(-40.0f, speed);
    }
    else if (L2 == Low) {
        // 左最外侧感应器触发（轻微左偏）
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Left Slight#\r\n", 20, HAL_MAX_DELAY);
        Motion_SpinTaskStart(-10.0f, speed);
    }
    else if (R2 == Low) {
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Right Slight#\r\n", 21, HAL_MAX_DELAY);
        // 右最外侧感应器触发（轻微右偏）
        Motion_SpinTaskStart(10.0f, speed);
    }
    else if (L1 == Low && R1 == Low) {
        // 正常行进：直走
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Forward#\r\n", 16, HAL_MAX_DELAY);
        Motion_Forward(speed);
    }
    else if (L1 == Low && R1 == High) {
        // 中央偏左（微调左）
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Left Adjust#\r\n", 20, HAL_MAX_DELAY);
        Motion_SpinTaskStart(-5.0f, speed);
    }
    else if (L1 == High && R1 == Low) {
        // 中央偏右（微调右）
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:Right Adjust#\r\n", 21, HAL_MAX_DELAY);
        Motion_SpinTaskStart(5.0f, speed);
    }
    else {
        // HAL_UART_Transmit(&Bluetooth_Cfg, "$LWK:OTHER/Backward#\r\n", 22, HAL_MAX_DELAY);
        // 上面已经全覆盖了,所有16种情况
        Motion_Backward(speed);
    }

    if (before_state != state) {
        // 如果状态变化了，发送状态变化信息
        before_state = state;
    }
}
