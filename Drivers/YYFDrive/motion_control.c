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

void Motion_Spin(int16_t speed)
{
    if (speed != 0)
        spd_spin = speed;
    MDrv_SetSpeed(-spd_spin, -spd_spin, spd_spin, spd_spin);
}

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

void Motion_Stop(void)
{
    MDrv_SetSpeed(0, 0, 0, 0);
}

void Motion_SpinTaskStart(float angle_deg, int16_t speed)
{
    float start_deg = MPU6050_Data.YawAngle;  // 假设起始角度为0度
    float target = start_deg + angle_deg;  // 不限制角度范围，允许跨360度
    spin_target_angle = target;
    spin_speed = abs(speed);
    spin_active = 1;
}

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

void Motion_AbortSpin(void)
{
    spin_active = 0;
}
