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

#define CAR_LENGTH 150.0f
#define CAR_WIDTH  133.3f
// 运动学约束半径 (Kinematic Constraint Radius)
static float KRadius = CAR_LENGTH / 2.0f + CAR_WIDTH / 2.0f; // 小车前后轮中心距离
static  const float DEG_TO_RAD = PI / 180.0f; // 角度转弧度的常数

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

/**
 * @brief 指定转弯半径控制小车左转
 * @param speed     小车中心前进的线速度 (mm/s)，负值为后退
 * @param radius    转弯轨迹的半径 (mm)，必须为正值
 * @note  此函数适用于“路径规划”，让小车走出一条精确的圆弧。
 */
void Motion_LeftTurnByRadius(int16_t speed, float radius)
{
    // Vinner = V
    // 根据运动学公式 V_inner = V * (1 - W/2R), V_outer = V * (1 + W/2R)
    // 计算左右轮的速度差。 W 是车宽 CAR_WIDTH。

    // 对于左转，左轮是内侧轮，右轮是外侧轮。
    float half_width = CAR_WIDTH / 2.0f;

    int16_t Lspd = (int16_t)(speed * (1.0f - half_width / radius)); // 内轮减速
    int16_t Rspd = (int16_t)(speed * (1.0f + half_width / radius)); // 外轮加速

    MDrv_SetSpeed(Lspd, Lspd, Rspd, Rspd);
}


/**
 * @brief 按指定的“角速度(度/秒)”控制小车向右转弯
 * @param speed     小车前进的基础线速度 (mm/s)
 * @param angspeed_dps 小车自转的角速度，单位就是【度/秒】
 */
void Motion_LeftTurnByAngdps(int16_t speed, float angspeed_dps)
{
    // 1. 将您输入的“度/秒”转换为物理公式需要的“弧度/秒”
    float angspeed_radps = angspeed_dps * DEG_TO_RAD;

    // 2. 使用“弧度/秒”进行后续计算
    float rotational_speed_component = angspeed_radps * KRadius;

    int16_t Lspd = speed - (int16_t)rotational_speed_component;
    int16_t Rspd = speed + (int16_t)rotational_speed_component;
    MDrv_SetSpeed(Lspd, Lspd, Rspd, Rspd);
}


/**
 * @brief 指定转弯半径控制小车右转
 * @param speed     小车中心前进的线速度 (mm/s)，负值为后退
 * @param radius    转弯轨迹的半径 (mm)，必须为正值
 * @note  此函数适用于“路径规划”，让小车走出一条精确的圆弧。
 */
void Motion_RightTurnByRadius(int16_t speed, float radius)
{
    // 根据运动学公式 V_inner = V * (1 - W/2R), V_outer = V * (1 + W/2R)
    // 计算左右轮的速度差。 W 是车宽 CAR_WIDTH。

    // 对于右转，右轮是内侧轮，左轮是外侧轮。
    float half_width = CAR_WIDTH / 2.0f;

    int16_t Rspd = (int16_t)(speed * (1.0f - half_width / radius)); // 内轮(右轮)减速
    int16_t Lspd = (int16_t)(speed * (1.0f + half_width / radius)); // 外轮(左轮)加速

    MDrv_SetSpeed(Lspd, Lspd, Rspd, Rspd);
}

/**
 * @brief 按指定的“角速度(度/秒)”控制小车向左转弯
 * @param speed     小车前进的基础线速度 (mm/s)
 * @param angspeed_dps 小车自转的角速度，单位就是【度/秒】
 */
void Motion_RightTurnByAngdps(int16_t speed, float angspeed_dps)
{
    float angspeed_radps = angspeed_dps * DEG_TO_RAD;
    // 计算由旋转产生的线速度分量，V_rot = V-z * (A+B)
    float rotational_speed_component = angspeed_radps * KRadius;

    // 左轮速度 = 前进速度 + 旋转分量 (左轮需要加速来实现左转)
    int16_t Lspd = speed + (int16_t)rotational_speed_component;
    // 右轮速度 = 前进速度 - 旋转分量 (右轮需要减速或反转来实现左转)
    int16_t Rspd = speed - (int16_t)rotational_speed_component;

    // 发送最终速度指令给电机驱动板
    MDrv_SetSpeed(Lspd, Lspd, Rspd, Rspd);
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
void Motion_RotationTurn(float angle_deg, int16_t forward_speed)
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
 * @brief 启动基于角度的原地旋转任务[MPU6050]
 * @param angle_deg 目标旋转角
 * @param speed 旋转速度，单位 mm/s，自动取绝对值
 */
void Motion_SpinTaskStart(float angle_deg, int16_t speed)
{
    float start_deg = MPU6050_Data.YawAngle;    // 假设起始角度为传感器的当前航向角
    spin_target_angle = start_deg + angle_deg;  // 不限制角度范围，允许跨360度
    spin_speed = speed < 0 ? -speed : speed;    // 确保速度为正值
    spin_active = 1;
}

/**
 * @brief 旋转任务更新函数，应周期调用以检查旋转角度完成状态
 * @param now_angle 当前实时航向角（单位 °）
 * @note 该函数会根据当前角度和目标角度计算差值，并控制电机速度
 * @note 如果转的超超快 达到更新任务周期能转一圈
 */
void Motion_SpinTaskUpdate(float now_angle)
{
    if (!spin_active)
        return;

    float diff = spin_target_angle - now_angle; // 125-90 = 35.0f 已经逆时针转了 90 还差35要转 逆时针为正
    while (diff > 180.0f) diff -= 180.0f;
    while (diff < -180.0f) diff += 180.0f;

    if (fabsf(diff) < ANGLE_THRESHOLD || spin_active == 0) {
        spin_active = 0;
        return;
    }

    if (diff < 0) {
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
