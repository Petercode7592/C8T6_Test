//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"
#include "motion_control.h"

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET

/* 车体几何参数（单位 mm） */
#define TRACK_WIDTH 135.0f // 左右轮中心距离
#define WHEEL_BASE 145.0f // 前后轴中心距离

#define RotSpeed 120 // 旋转速度
#define TURN_BASE_SPEED 100 // 差速转弯时的基础前进速度
static int8_t last_state = -1; // 使用 static 变量来保存上一次的状态

// -1: 上一次是向左偏 (车身在黑线右侧)
//  0: 上一次是居中
//  1: 上一次是向右偏 (车身在黑线左侧)
int8_t last_deviation = 0;

void LineWalking(int16_t speed) {
    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2
    // Notice: 已知1.7mm的间距黑胶不会有 0011 和 1100 的情况 行进路线放置

    // 改为直接在这里面读取
    uint8_t L2 = (HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Track_L2X1_Pin) == High);  // 左外
    uint8_t L1 = (HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Track_L1X2_Pin) == High);  // 左内
    uint8_t R1 = (HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Track_R1X3_Pin) == High);  // 右内
    uint8_t R2 = (HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Track_R2X4_Pin) == High);  // 右外

    int8_t current_state = (L2 << 3) | (L1 << 2) | (R1 << 1) | (R2 << 0);

    if (L2 && !L1 && !R1 && R2) { // 状态: [1, 0, 0, 1] -> 最理想状态，车身居中
        Motion_Forward(speed);
        last_deviation = 0;
    }
    else if (L2 && L1 && !R1 && R2) { // 状态: [1, 1, 0, 1] -> 车身偏左，向右微调
        Motion_RightTurnByAngdps(speed, 2.0f);
        last_deviation = 1; // 记录向右偏
    }
    else if (L2 && !L1 && R1 && R2) { // 状态: [1, 0, 1, 1] -> 车身偏右，向左微调
        Motion_LeftTurnByAngdps(speed, 2.0f);
        last_deviation = -1; // 记录向左偏
    }
    else if (!L2 && !L1 && R1 && R2) { // 状态: [0, 0, 1, 1] -> 车身严重偏右，向左大角度转弯
        Motion_LeftTurnByAngdps(TURN_BASE_SPEED, 20.0f);
        last_deviation = -1;
    }
    else if (L1 && L2 && !R1 && !R2) { // 状态: [1, 1, 0, 0] -> 车身严重偏左，向右大角度转弯
        Motion_RightTurnByAngdps(TURN_BASE_SPEED, 20.0f);
        last_deviation = 1;
    }
    else if (L2 && L1 && R1 && R2) { // 状态: [1, 1, 1, 1] -> 全部在白线，完全脱轨
        if (last_state == -1) {
            // 第一次启动
            Motion_Forward(speed); // 向前行驶
            last_deviation = 0; // 重置偏移状态
        }else if (last_deviation == 0) {
            // 如果上次是居中
            Motion_Spin(-100); // 向左转找回黑线
        }else if (last_deviation == -1) {
            // 如果上次是向左偏
            Motion_Spin(100); // 向右转找回黑线
        }else {
            // 如果上次是向右偏
            Motion_Spin(-100); // 向左转找回黑线
        }
    }
    else if (!L2 && !L1 && !R1 && !R2) { // 状态: [0, 0, 0, 0]
        // 假设这是一个未知的急转弯或终点
        if (last_deviation == 0 || last_deviation == -1) { // 如果最后是向左偏或居中
            // 很可能是遇到了右侧的直角弯，或需要向右找线
            Motion_Spin(-RotSpeed); // 向右转找回黑线
        } else if (last_deviation == 1){ // 如果最后是向右偏
            // 很可能是遇到了左侧的直角弯，或需要向左找线
            Motion_Spin(RotSpeed); // 向左转找回黑线
        } else {
            Motion_Backward(speed/2); // 如果状态异常，后退一段距离
        }
    }
    else if (!L2) { // 状态: [0, x, x, x] -> 最左侧传感器压线，左急转
        Motion_Spin(RotSpeed); // 正值左转
        last_deviation = -1;
    }
    else if (!R2) { // 状态: [x, x, x, 0] -> 最右侧传感器压线，右急转
        Motion_Spin(-RotSpeed); // 负值右转
        last_deviation = 1;
    }

    if (current_state == last_state) {
        return; // 如果状态没有变化，直接返回，避免重复发送电机指令
    }
    last_state = current_state; // 更新状态
}