/**
 ******************************************************************************
 * @file    moto.h
 * @brief   4-轮差速底盘 —— 全功能电机控制头文件
 * @author  (your-name)
 * @version 1.0
 * @date    2025-07-12
 *
 * ───────────────────────────────────────────────────────────────
 * 1) 核心接口：     MOTO_ControlTwist(v, ω)      // 连续量
 * 2) 转角封装：     MOTO_ControlTwistAngle(v, Δθ, t)
 * 3) 常用动作宏：   Stop / Forward / Backward / Spin / ArcTurn
 * ───────────────────────────────────────────────────────────────
 */

#ifndef __MOTO_H
#define __MOTO_H

#include <stdint.h>
#include "stm32f1xx_hal.h"            /* 若芯片不同请替换 */

/*—————————————— 机体参数 & 限制 —————————————*/
#define TRACK_WIDTH_MM        (135.0f)      /* 左右轮中心距 (mm) */
#define MAX_WHEEL_V_MM_S      (1000.0f)     /* 单轮最大线速 (mm/s) */
#define PI_F                  (3.1415926f)

/*——————————————— 核心 API ———————————————*/
/**
 * @brief  Twist 控制
 * @param  v_mm_s       线速度 (+前 / −后)  mm/s
 * @param  omega_rad_s  角速度 (+左 / −右)  rad/s
 */
void MOTO_ControlTwist(float v_mm_s, float omega_rad_s);

/**
 * @brief  让机器人在 duration_s 时间内转 delta_deg 角
 * @param  v_mm_s      线速度 mm/s
 * @param  delta_deg   左转 +, 右转 −  (°)
 * @param  duration_s  秒
 */
void MOTO_ControlTwistAngle(float v_mm_s,
                            float delta_deg,
                            float duration_s);

/*——————————————— 封装动作 API ———————————————*/
/* 刹车 / 原地不动 */
static inline void MOTO_Stop(void) { MOTO_ControlTwist(0.0f, 0.0f); }

/* 直线行驶（±speed_mm_s） */
static inline void MOTO_Forward (float speed_mm_s) { MOTO_ControlTwist(+speed_mm_s, 0.0f); }
static inline void MOTO_Backward(float speed_mm_s) { MOTO_ControlTwist(-speed_mm_s, 0.0f); }

/* 原地旋转（rad_s > 0 左转，<0 右转） */
static inline void MOTO_SpinLeft (float omega_rad_s) { MOTO_ControlTwist(0.0f, +omega_rad_s); }
static inline void MOTO_SpinRight(float omega_rad_s) { MOTO_ControlTwist(0.0f, -omega_rad_s); }

/**
 * @brief  弧线转弯：给定前进速度 v_mm_s 与转弯半径 R_mm
 *         R_mm > 0 → 左弯,  R_mm < 0 → 右弯
 */
void MOTO_ArcTurn(float v_mm_s, float R_mm);

#endif  /* __MOTO_H */
