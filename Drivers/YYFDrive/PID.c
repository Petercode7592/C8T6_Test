#include "pid.h"

// --- 模块内部私有定义 ---

// PID控制器的数据结构定义
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    float output_min;
    float output_max;
} PID_Controller;

// PID控制器实例数组，定义为static，意味着它只在本文件内部可见
static PID_Controller motor_pids[4];

// --- 模块内部私有函数 ---

/**
 * @brief 内部PID更新函数，用于单个电机
 */
static float PID_Update_Single(PID_Controller *pid, float setpoint, float feedback) {
    pid->setpoint = setpoint;
    float error = pid->setpoint - feedback; // 计算误差
    pid->integral += error; // 积分项累加
    float derivative = error - pid->last_error; // 计算微分项
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative); // PID公式计算输出

    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    pid->last_error = error;
    return output;
}

// --- 对外公开函数的实现 ---

/**
 * @brief 初始化所有电机的PID控制器
 */
void PID_Init_All_Motors(void) {
    // Kp, Ki, Kd 的值需要您通过实验来整定
    float Kp = 1.5f;
    float Ki = 0.5f;
    float Kd = 0.2f;

    // 这个输出范围是PID调整量的范围，而不是最终速度的范围
    // 可以设得大一些，最终会在MDrv_SetSpeed函数中进行限幅
    float min_adj = -1000.0f;
    float max_adj = 1000.0f;

    for (int i = 0; i < 4; i++) {
        motor_pids[i].Kp = Kp;
        motor_pids[i].Ki = Ki;
        motor_pids[i].Kd = Kd;
        motor_pids[i].output_min = min_adj;
        motor_pids[i].output_max = max_adj;
        motor_pids[i].integral = 0.0f;
        motor_pids[i].last_error = 0.0f;
    }
}

/**
 * @brief 对所有电机运行一次PID闭环计算
 */
void PID_Run_All_Motors(const volatile int32_t* Vstd, const volatile float* V, int32_t* NewSpeed) {
    for (int i = 0; i < 4; i++) {
        float setpoint = (float)Vstd[i];
        float feedback = V[i];

        // PID_Update_Single 计算出的是一个“调整量”
        float adjustment = PID_Update_Single(&motor_pids[i], setpoint, feedback);

        // 新的速度指令 = 原始期望值 + PID调整量
        NewSpeed[i] = Vstd[i] + (int32_t)adjustment;
    }
}