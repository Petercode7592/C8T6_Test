//
// Created by Petercode on 25-7-10.
//

#ifndef TRACKING_H
#define TRACKING_H

#include "main.h"

// 按照传感器位置应该是 L2 L1 R1 R2

uint8_t GetLineWalking();
void LineWalking(uint8_t state, int16_t speed);

#endif //TRACKING_H
