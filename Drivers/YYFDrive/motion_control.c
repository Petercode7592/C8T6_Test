#include "motion_control.h"
#include "Mdrv.h"  // MDrv_SetSpeed(m1, m2, m3, m4)
#include <math.h>
#include <stdlib.h>
#include "main.h"
#include "Global.h"  // 假设有一个MPU6050模块来获取当前角度

#define PI 3.1415926f
#define GAIN 0.4f
#define ANGLE_THRESHOLD 2.0f

/*
 * M1 -> 左上电机（小车的左前轮）
 * M2 -> 左下电机（小车的左后轮）
 * M3 -> 右上电机（小车的右前轮）
 * M4 -> 右下电机（小车的右后轮）
*/

static int16_t spd_fwd = 0;
static int16_t spd_spin = 0;

static float spin_target_angle = 0.0f;
static int16_t spin_speed = 0;
static uint8_t spin_active = 0;

/**
 * @brief 小车前进
 * @param speed 线速度，单位 mm/s，为 0 时使用上一次速度
 */
void Motion_Forward(int16_t speed)
{
    if (speed != 0)
        spd_fwd = speed;
    MDrv_SetSpeed(spd_fwd, spd_fwd, spd_fwd, spd_fwd);
}

/**
 * @brief 小车后退(失速)
 * @param speed 线速度，单位 mm/s，为 0 时使用上一次速度
 * @note 友情提醒：小车后退速度过高会车轮失速
 */
void Motion_Backward(int16_t speed)
{
    if (speed != 0)
        spd_fwd = speed;
    MDrv_SetSpeed(-spd_fwd, -spd_fwd, -spd_fwd, -spd_fwd);
}

void Motion_left(int16_t speed)
{
    if (speed != 0)
        spd_spin = speed;
    MDrv_SetSpeed(spd_spin, spd_spin, 0, 0);
}

void Motion_right(int16_t speed)
{
    if (speed != 0)
        spd_spin = speed;
    MDrv_SetSpeed(0, 0, spd_spin, spd_spin);
}

/**
 * @brief 小车原地旋转
 * @param speed 旋转速度，单位 mm/s，<0 为右转，>0 为左转，为 0 使用上次速度(bug)
 */
void Motion_Spin(int16_t speed)
{
    if (speed != 0)
        spd_spin = speed;
    MDrv_SetSpeed(-spd_spin, -spd_spin, spd_spin, spd_spin);
}

/**
 * @brief 小车弧形转弯前进（非原地）
 * @param angle_deg 转弯方向角，正右负左（单位 °）
 * @param forward_speed 前进速度，单位 mm/s，为 0 使用上次速度
 */
void Motion_ArcTurn(float angle_deg, int16_t forward_speed)
{
    if (forward_speed != 0)
        spd_fwd = forward_speed;
    float angle_rad = angle_deg * (PI / 180.0f);
    float ratio = -sinf(angle_rad);               // 正值左转，负值右转
    float delta = spd_fwd * ratio * GAIN;

    int16_t Lspd = (int16_t)(spd_fwd - delta);
    int16_t Rspd = (int16_t)(spd_fwd + delta);

    MDrv_SetSpeed(Lspd, Lspd, Rspd, Rspd);
}

/**
 * @brief 小车立即停止运动
 */
void Motion_Stop(void)
{
    MDrv_SetSpeed(0, 0, 0, 0);
    MDrv_SetPWM(0, 0, 0, 0);  // 停止电机输出
}

/**
 * @brief 启动基于角度的原地旋转任务
 * @param angle_deg 目标旋转角，正左转，负右转（单位 °）
 * @param speed 旋转速度，单位 mm/s，自动取绝对值
 */
void Motion_SpinTaskStart(float angle_deg, int16_t speed)
{
    float start_deg = MPU6050_Data.YawAngle;  // 假设起始角度为0度
    spin_target_angle = start_deg + angle_deg;  // 不限制角度范围，允许跨360度
    spin_speed = abs(speed);
    spin_active = 1;
}

/**
 * @brief 旋转任务更新函数，应周期调用以检查旋转角度完成状态
 * @param now_angle 当前实时航向角（单位 °）
 * @note 该函数会根据当前角度和目标角度计算差值，并控制电机速度
 */
void Motion_SpinTaskUpdate(float now_angle)
{
    if (!spin_active)
        return;

    float diff = spin_target_angle - now_angle;
    while (diff > 180.0f) diff -= 180.0f;
    while (diff < -180.0f) diff += 180.0f;

    if (fabsf(diff) < ANGLE_THRESHOLD || spin_active == 0) {
        spin_active = 0;
        return;
    }

    if (diff > 0) {
        MDrv_SetSpeed(spin_speed, spin_speed, -spin_speed, -spin_speed);
    } else {
        MDrv_SetSpeed(-spin_speed, -spin_speed, spin_speed, spin_speed);
    }
}

/**
 * @brief 强制中止原地旋转任务
 */
void Motion_AbortSpin(void)
{
    spin_active = 0;
}
