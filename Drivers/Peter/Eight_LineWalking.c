// EIGHT_LINEWALKING.c
#include "Eight_LineWalking.h"
#include "PID.h"
#include "motion_control.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"

// 权重表：-35 ~ +35（代表偏离中心的程度）
static const int8_t sensor_weight[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
static PID_Controller line_pid;

/**
 * @brief 初始化PID控制器（用于循迹）
 */
void LineWalking_Init(void) {
    PID_Init(&line_pid, 2.0f, 0.0f, 4.5f); // 使用PD控制：Ki=0

    // 启用积分限幅（防止积分过大）
    line_pid.enable_limit = 1;
    line_pid.I_max = 300.0f;
    line_pid.I_min = -300.0f;

    // 启用输出限幅（限制速度差）
    line_pid.enable_output_limit = 1;
    line_pid.out_max = 400.0f;
    line_pid.out_min = -400.0f;
}

/**
 * @brief 计算循迹误差
 */
static int16_t Calculate_Track_Error(uint8_t track_digital) {
    int sum = 0;
    int count = 0;
    for (int i = 0; i < 8; i++) {
        if (((track_digital >> i) & 0x01) == 0) {     // 0 表示黑线
            sum += sensor_weight[i];
            count++;
        }
    }
    if (count == 0) return 0; // 掉线处理（可自定义）
    return sum / count;
}

/**
 * @brief 进行一次循迹控制（建议周期调用）
 * @param track_digital 8路黑白状态数据（bit0~bit7：S1~S8）
 * @param speed 直行速度
 */
void LineWalking_Track(uint8_t track_digital, int16_t speed) {
    int16_t error = Calculate_Track_Error(track_digital);
    int16_t correction = PID_Compute(&line_pid, error);
    // HAL_UART_Transmit(&huart1, (uint8_t*)&error, sizeof(error), 100); // 调试输出误差
    int16_t base_speed = speed; // mm/s，基础速度可调
    int16_t left_speed = base_speed - correction;
    int16_t right_speed = base_speed + correction;

    Motion_SetLR_Speed(left_speed, right_speed);
}
