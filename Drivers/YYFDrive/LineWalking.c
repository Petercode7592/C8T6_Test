//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"
#include "Mdrv.h"
#include "math.h"

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET


void GetLineWalking(LineWalkingState_t *state) {
    state->Left1 = HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Track_L1X2_Pin);
    state->Left2 = HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Track_L2X1_Pin);
    state->Right1 = HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Track_R1X3_Pin);
    state->Right2 = HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Track_R2X4_Pin);
}

/* 车体几何参数（单位 mm） */
#define TRACK_WIDTH   135.0f   // 左右轮中心距离
#define WHEEL_BASE    145.0f   // 前后轴中心距离

/**
 * @brief  对四轮差动驱动做“曲线转弯”速度分配
 * @param  Vc   车体中心线速度（mm/s）
 * @param  R    转弯半径，相对于车体中心（mm），须 > WHEEL_BASE/2 = 72.5mm
 */
static void setCurveSpeed(float R)
{
    float halfTrack = TRACK_WIDTH * 0.5f;   // 半跨距
    float halfBase  = WHEEL_BASE  * 0.5f;   // 半轴距

    // 1. 从全局 Vstd 数组读取并计算中心线速度
    float Vc = (Vstd[0] + Vstd[1] + Vstd[2] + Vstd[3]) / 4.0f;

    // 2. 角速度 ω = Vc / R
    float omega = Vc / R;

    // 3. 计算四个车轮到 ICC 的距离
    float R_fl = sqrtf((R - halfTrack)*(R - halfTrack) + halfBase*halfBase);  // 左前
    float R_fr = sqrtf((R + halfTrack)*(R + halfTrack) + halfBase*halfBase);  // 右前
    float R_rl = R_fl;  // 左后对称
    float R_rr = R_fr;  // 右后对称

    // 4. 线速度 vi = ω × Ri
    int32_t v_fl = (int32_t)(omega * R_fl);
    int32_t v_fr = (int32_t)(omega * R_fr);
    int32_t v_rl = v_fl;
    int32_t v_rr = v_fr;


    /* 限幅到驱动板支持范围 */
    #define CLAMP(x)  do { if ((x)>1000) (x)=1000; if ((x)<-1000) (x)=-1000; } while(0)
    CLAMP(v_fl); CLAMP(v_fr); CLAMP(v_rl); CLAMP(v_rr);

    /* 下发给闭环驱动 */
    MDrv_SetSpeed(v_fl, v_rl, v_fr, v_rr);
}


void LineWalking(LineWalkingState_t *state) {
    GPIO_PinState L1 = state->Left1;
    GPIO_PinState L2 = state->Left2;
    GPIO_PinState R1 = state->Right1;
    GPIO_PinState R2 = state->Right2;

    uint8_t delay = 10; // 默认延时

    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2

    // —— 左大弯 —— (0111)
    if (L2 == Low && L1 == High && R1 == High && R2 == High) {
        // 让左边反转、右边前进 → 用大偏移量
        setCurveSpeed(150.0f);
        delay = 80;
    }
    // —— 左直角 —— (0011)
    else if (L2 == Low && L1 == Low && R1 == High && R2 == High) {
        setCurveSpeed(100.0f); // 左直角
        delay = 40;
    }
    // —— 右直角 —— (1100)
    else if (L2 == High && L1 == High && R1 == Low && R2 == Low) {
        setCurveSpeed(-100.0f); // 右直角
        delay = 40;
    }
    // —— 右大弯 —— (1110)
    else if (L2 == High && L1 == High && R1 == High && R2 == Low) {
        setCurveSpeed(-150.0f); // 右大弯
        delay = 80;
    }
    // —— 左最外侧微调（小弯）—— (1011) 中间微调左
    else if (L2 == High && L1 == Low && R1 == High && R2 == High) {
        setCurveSpeed(300.0f); // 中等偏左
        delay = 10;
    }
    // —— 右最外侧微调（小弯）—— (1101) 中间微调右
    else if (L2 == High && L1 == High && R1 == Low && R2 == High) {
        setCurveSpeed(-300.0f); // 中等偏右
        delay = 10;
    }
    // —— 居中直行 —— (L1==Low && R1==Low)
    else if (L1 == Low && R1 == Low) {
        ;
    }
    // —— 其它断线 —— 停车
    else {
        MDrv_SetSpeed(0, 0, 0, 0);
        return;
    }
    HAL_Delay(delay);
}
