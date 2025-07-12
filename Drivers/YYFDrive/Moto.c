/**
 ******************************************************************************
 * @file    moto.c
 * @brief   4-轮差速底盘 —— 全功能电机控制实现
 ******************************************************************************
 */

#include "moto.h"
#include <math.h>    /* fabsf, fmaxf */
#include "Mdrv.h"

/*================ 内部辅助 ================================*/
static inline float _limit(float val, float limit_abs)
{
    if (val >  limit_abs) return  limit_abs;
    if (val < -limit_abs) return -limit_abs;
    return val;
}

/*================ Twist 主接口 ============================*/
void MOTO_ControlTwist(float v_mm_s, float omega_rad_s)
{
    /* 1. 差速模型：左右轮线速度 */
    float v_left  = v_mm_s - omega_rad_s * TRACK_WIDTH_MM * 0.5f;
    float v_right = v_mm_s + omega_rad_s * TRACK_WIDTH_MM * 0.5f;

    /* 2. 限幅 & 比例缩放 */
    float vmax = fmaxf(fabsf(v_left), fabsf(v_right));
    if (vmax > MAX_WHEEL_V_MM_S) {
        float scale = MAX_WHEEL_V_MM_S / vmax;
        v_left  *= scale;
        v_right *= scale;
    }

    /* 3. 下发 (四轮同侧同速) */
    int32_t vL_i = (int32_t)v_left;
    int32_t vR_i = (int32_t)v_right;
    MDrv_SetSpeed(vL_i, vL_i, vR_i, vR_i);
}

/*================ Twist-Angle 封装 ========================*/
void MOTO_ControlTwistAngle(float v_mm_s,
                            float delta_deg,
                            float duration_s)
{
    if (duration_s <= 0.0f) return;            /* 防止除 0 */
    float delta_rad  = delta_deg * PI_F / 180.0f;
    float omega_rad  = delta_rad / duration_s; /* rad/s */
    MOTO_ControlTwist(v_mm_s, omega_rad);
}

/*================ 弧线转弯封装 ============================*/
void MOTO_ArcTurn(float v_mm_s, float R_mm)
{
    if (fabsf(R_mm) < 1e-3f) {                 /* 半径过小 → 当作原地旋转 */
        float omega = (R_mm >= 0.0f) ? 1.0f : -1.0f;
        MOTO_ControlTwist(0.0f, omega);
        return;
    }
    float omega = v_mm_s / R_mm;               /* rad/s   正左负右 */
    MOTO_ControlTwist(v_mm_s, omega);
}
