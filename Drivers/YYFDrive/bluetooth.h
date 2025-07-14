#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f1xx_hal.h"

// 小车状态枚举
typedef enum {
    CAR_STOP = 0,
    CAR_FORWARD = 1,
    CAR_BACKWARD = 2,
    CAR_TURN_LEFT = 3,
    CAR_TURN_RIGHT = 4,
    CAR_SPIN_LEFT = 5,
    CAR_SPIN_RIGHT = 6
} CarState_e;

// 外部变量
extern volatile uint8_t  Bluetooth_NewCmdFlag;
extern volatile CarState_e Bluetooth_CarState;

// 函数声明
void Bluetooth_Init(void);
void Bluetooth_ParseData(void);

#endif /* __BLUETOOTH_H */
