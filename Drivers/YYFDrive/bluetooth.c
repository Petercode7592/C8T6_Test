#include "bluetooth.h"
#include <string.h>
#include "usart.h"


// ⚠️ 宏转换成地址
#define Bluetooth_huart (&Bluetooth_Cfg)

// 接收缓冲区
#define RX_BUFFER_SIZE 60
static uint8_t  RxBuffer[RX_BUFFER_SIZE];
static uint8_t  RxIndex = 0;
static uint8_t  Receiving = 0;
static uint8_t  tempRxByte = 0;

volatile uint8_t  Bluetooth_NewCmdFlag = 0;
volatile CarState_e Bluetooth_CarState = CAR_STOP;

void Bluetooth_Init(void) {
    RxIndex = 0;
    Receiving = 0;
    Bluetooth_NewCmdFlag = 0;
    Bluetooth_CarState = CAR_STOP;

    // 开启串口中断接收（每次1字节）
    HAL_UART_Receive_IT(Bluetooth_huart, &tempRxByte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == Bluetooth_huart) {
        uint8_t byte = tempRxByte;

        if (!Receiving) {
            if (byte == '$') {
                Receiving = 1;
                RxIndex = 0;
                Bluetooth_NewCmdFlag = 0;
            }
        } else {
            if (byte == '#') {
                Receiving = 0;
                RxBuffer[RxIndex] = '\0';
                Bluetooth_NewCmdFlag = 1;
            } else {
                if (RxIndex < RX_BUFFER_SIZE - 1) {
                    RxBuffer[RxIndex++] = byte;  // 存储接收到的字节
                }
            }
        }

        // 继续接收
        HAL_UART_Receive_IT(Bluetooth_huart, &tempRxByte, 1);
    }
}

void Bluetooth_ParseData(void) {
    if (Bluetooth_NewCmdFlag) {
        switch (RxBuffer[0]) {
            case '0': Bluetooth_CarState = CAR_STOP; break;
            case '1': Bluetooth_CarState = CAR_FORWARD; break;
            case '2': Bluetooth_CarState = CAR_BACKWARD; break;
            case '3': Bluetooth_CarState = CAR_TURN_LEFT; break;
            case '4': Bluetooth_CarState = CAR_TURN_RIGHT; break;
            default: break;
        }

        if (RxBuffer[2] == '1') {
            Bluetooth_CarState = CAR_SPIN_LEFT;
        } else if (RxBuffer[2] == '2') {
            Bluetooth_CarState = CAR_SPIN_RIGHT;
        }

        Bluetooth_NewCmdFlag = 0;
    }
}

HAL_StatusTypeDef Bluetooth_SendData(uint8_t *pData, uint16_t length) {
    return HAL_UART_Transmit(Bluetooth_huart, pData, length, Uart_MaxWait);
}

HAL_StatusTypeDef Bluetooth_SendString(const char *str) {
    return HAL_UART_Transmit(Bluetooth_huart, (uint8_t*)str, strlen(str), Uart_MaxWait);
}


