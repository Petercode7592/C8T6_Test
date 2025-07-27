// EIGHT_LINEWALKING.c
#include "Eight_LineWalking.h"
#include "PID.h"
#include "motion_control.h"
#include "Display/OLED/Oled.h"

// 权重表：中线为负，越靠外权重越大（正负对称）
static const int8_t sensor_weight[8] = {35, 25, 15, 5, -5, -15, -25, -35};
static PID_Controller line_pid;
static int16_t last_valid_error = 0; // 用于掉线方向记忆
static int16_t last_digital = -1;     // 上一帧有效数据（用于掉线恢复）

/**
 * @brief 初始化PID控制器（用于循迹）
 */
void LineWalking_Init(void) {
    // 12.0 5.8
    PID_Init(&line_pid, 12.0f, 0.0f, 5.8f); // 使用PD控制

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
 * @note 传入 191 = 1011 1111b ((track >> 6) & 0x01) 为 0 意味黑线
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
    // 掉线检测1：
    // if (count == 0)
    //     return last_valid_error; // 如果没有黑线，返回上次有效误差
    return sum / count; // 返回平均误差
}

/**
 * @brief 进行一次循迹控制（建议周期调用）
 * @param track_digital 8路黑白状态数据（bit0~bit7：S1~S8）
 * @param speed 直行速度
 * @note 400mm/s 是稳定的最大速度
 */
void LineWalking8_Track(uint8_t track_digital, int16_t speed) {
    // 掉线检测2：启动掉线恢复任务（全为1表示所有都是白线）
    // 掉线处理：全白时根据 last_digital 判断是否刚刚发生跳变
    if (track_digital == 0xFF) {
        if (last_digital != 0xFF && last_digital != -1) {
            // 掉线前有信息，触发带方向的找线逻辑
            Motion_Spin((last_valid_error >= 0) ? 150 : -150);
        }
        if (last_digital != -1) // 掉线后不再处理，等待下一次有效数据 刚启动让其继续走
            return;
    }
    else if (track_digital == last_digital) {
        return; // 没有变化，直接返回
    }

    // 更新上一帧有效数据
    last_digital = track_digital;

    int16_t error = Calculate_Track_Error(track_digital);
    last_valid_error = error;
    int16_t correction = PID_Compute(&line_pid, error);


    // OLED_ShowSignedNum(1,1, error, 4, 8); // 显示误差值
    // OLED_ShowSignedNum(1,4, correction, 4, 8); // 显示修正值
    // 根据误差计算速度修正
    Motion_SetLR_Speed(speed - correction, speed + correction);
}
