// PID.h
#ifndef PID_H
#define PID_H

#include <stdint.h>

// 完整 PID 控制结构体
typedef struct {
    float Kp;  // 比例系数
    float Ki;  // 积分系数
    float Kd;  // 微分系数

    int16_t last_error;  // 上一次误差，用于计算微分
    float integral;      // 累计积分项

    // 积分限幅功能开关与范围
    uint8_t enable_limit;
    float I_max;
    float I_min;

    // 输出限幅功能开关与范围
    uint8_t enable_output_limit;
    float out_max;
    float out_min;

} PID_Controller;

// 初始化 PID 控制器
void PID_Init(PID_Controller* pid, float Kp, float Ki, float Kd);

// 输入当前误差值，返回控制输出值
int16_t PID_Compute(PID_Controller* pid, int16_t error);

#endif // PID_H
