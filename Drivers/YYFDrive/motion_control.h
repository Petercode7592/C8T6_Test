// motion_control.h
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

/**
`` * @brief 小车前进
 * @param speed 线速度，单位 mm/s，为 0 时使用上一次速度
 */
void Motion_Forward(int16_t speed);

/**
 * @brief 小车后退
 * @param speed 线速度，单位 mm/s，为 0 时使用上一次速度
 */
void Motion_Backward(int16_t speed);

void Motion_right(int16_t speed);
void Motion_left(int16_t speed);

/**
 * @brief 小车原地旋转
 * @param speed 旋转速度，单位 mm/s，<0 为右转，>0 为左转，为 0 使用上次速度
 */
void Motion_Spin(int16_t speed); // <0: right, >0: left

/**
 * @brief 小车弧形转弯前进（非原地）
 * @param angle_deg 转弯方向角，正右负左（单位 °）
 * @param forward_speed 前进速度，单位 mm/s，为 0 使用上次速度
 */
void Motion_ArcTurn(float angle_deg, int16_t forward_speed);

/**
 * @brief 小车立即停止运动
 */
void Motion_Stop(void);

/**
 * @brief 启动基于角度的原地旋转任务
 * @param angle_deg 目标旋转角，正左转，负右转（单位 °）
 * @param speed 旋转速度，单位 mm/s，自动取绝对值
 */
void Motion_SpinTaskStart(float angle_deg, int16_t speed);

/**
 * @brief 旋转任务更新函数，应周期调用以检查角度完成状态
 * @param now_angle 当前实时航向角（单位 °）
 */
void Motion_SpinTaskUpdate(float now_angle);

/**
 * @brief 强制中止原地旋转任务
 */
void Motion_AbortSpin(void);

#endif