#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief 初始化所有电机的PID控制器
 * 此函数应在系统启动时，在main函数中调用一次。
 */
void PID_Init_All_Motors(void);

/**
 * @brief 对所有电机运行一次PID闭环计算
 * @param Vstd      [in] 指向包含4个电机期望速度的数组的指针
 * @param V         [in] 指向包含4个电机实际速度的数组的指针
 * @param NewSpeed [out] 指向用于存储计算出的4个新速度指令的数组的指针
 */
void PID_Run_All_Motors(const volatile int32_t* Vstd, const volatile float* V, int32_t* NewSpeed);

#endif // PID_H