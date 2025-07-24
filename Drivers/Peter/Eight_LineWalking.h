#ifndef EIGHT_LINEWALKING_H
#define EIGHT_LINEWALKING_H

#include <stdint.h>

// 初始化循迹PID控制器
void LineWalking_Init(void);

// 调用此函数进行循迹控制（输入8路黑白数据）
void LineWalking_Track(uint8_t track_digital, int16_t speed);

#endif // EIGHT_LINEWALKING_H
