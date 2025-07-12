//
// Created by Petercode on 25-7-10.
//

#ifndef TRACKING_H
#define TRACKING_H

#include "main.h"

// 按照传感器位置应该是 L2 L1 R1 R2
typedef struct {
    GPIO_PinState Left1;
    GPIO_PinState Left2;
    GPIO_PinState Right1;
    GPIO_PinState Right2;
} LineWalkingState_t;

void GetLineWalking(LineWalkingState_t *state);
void LineWalking(LineWalkingState_t *state);

#endif //TRACKING_H
