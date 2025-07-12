//
// Created by Petercode on 25-7-10.
//
#include "LineWalking.h"
#include "main.h"
#include "Mdrv.h"

#define High GPIO_PIN_SET
#define Low GPIO_PIN_RESET

// 每次偏移步进和偏移限幅
#define OFFSET_STEP   50
#define MAX_OFFSET   400


void GetLineWalking(LineWalkingState_t *state) {
    state->Left1 = HAL_GPIO_ReadPin(Track_L1X2_GPIO_Port, Track_L1X2_Pin);
    state->Left2 = HAL_GPIO_ReadPin(Track_L2X1_GPIO_Port, Track_L2X1_Pin);
    state->Right1 = HAL_GPIO_ReadPin(Track_R1X3_GPIO_Port, Track_R1X3_Pin);
    state->Right2 = HAL_GPIO_ReadPin(Track_R2X4_GPIO_Port, Track_R2X4_Pin);
}


void LineWalking(LineWalkingState_t *state) {
    GPIO_PinState L1 = state->Left1;
    GPIO_PinState L2 = state->Left2;
    GPIO_PinState R1 = state->Right1;
    GPIO_PinState R2 = state->Right2;

    // 基准速度，直接从 Vstd 数组里取
    int32_t baseL = Vstd[0];
    int32_t baseL2 = Vstd[1];
    int32_t baseR = Vstd[2];
    int32_t baseR2 = Vstd[3];
    int32_t offset = 0; // 偏移量
    uint8_t delay = 10; // 默认延时

    // 首先 Low 代表黑线 High 代表白线
    // L2   L1   R1   R2

    // —— 左大弯 —— (0111)
    if (L2 == Low && L1 == High && R1 == High && R2 == High) {
        // 让左边反转、右边前进 → 用大偏移量
        offset = baseL + baseR;
        delay = 80;
    }
    // —— 左直角 —— (0011)
    else if (L2 == Low && L1 == Low && R1 == High && R2 == High) {
        MDrv_SetSpeed(Vstd[0], Vstd[1], -Vstd[2], -Vstd[3]); // 左直角
        delay = 40;
    }
    // —— 右直角 —— (1100)
    else if (L2 == High && L1 == High && R1 == Low && R2 == Low) {
        MDrv_SetSpeed(-Vstd[0], -Vstd[1], Vstd[2], Vstd[3]); // 右直角
        delay = 40;
    }
    // —— 右大弯 —— (1110)
    else if (L2 == High && L1 == High && R1 == High && R2 == Low) {
        offset = -(baseL + baseR);
        delay = 80;
    }
    // —— 左最外侧微调（小弯）—— (1011)
    else if (L2 == Low && L1 == High && R1 == High && R2 == High) {
        offset = baseL; // 中等偏左
        delay = 10;
    }
    // —— 右最外侧微调（小弯）—— (1101)
    else if (L2 == High && L1 == High && R1 == Low && R2 == High) {
        offset = -baseL; // 中等偏右
        delay = 10;
    }
    // —— 中间微调左 —— (中间左传感器有线，右中没线)
    else if (L1 == Low && R1 == High) {
        offset = 50; // 小步进左
    }
    // —— 中间微调右 —— (右中有线，左中没线)
    else if (L1 == High && R1 == Low) {
        offset = -50; // 小步进右
    }
    // —— 居中加速 —— (L1==Low && R1==Low)
    else if (L1 == Low && R1 == Low) {
        offset = 0;
    }
    // —— 其它断线 —— 停车
    else {
        MDrv_SetSpeed(0, 0, 0, 0);
        return;
    }

    // 保留原速，只在“中间”情况做微调
    int32_t spL = Vstd[0] - offset;
    int32_t spL2 = Vstd[1] - offset;
    int32_t spR = Vstd[2] + offset;
    int32_t spR2 = Vstd[3] + offset;

    // 限幅到 [-1000,1000]
    if (spL > 1000) spL = 1000;
    if (spL < -1000) spL = -1000;
    if (spL2 > 1000) spL2 = 1000;
    if (spL2 < -1000) spL2 = -1000;
    if (spR > 1000) spR = 1000;
    if (spR < -1000) spR = -1000;
    if (spR2 > 1000) spR2 = 1000;
    if (spR2 < -1000) spR2 = -1000;

    MDrv_SetSpeed(spL, spL2, spR, spR2);
    HAL_Delay(delay);
}
