#ifndef OLED_H
#define OLED_H

// OLED 驱动头文件


#include "stm32f1xx_hal.h" // 修改为你具体的 HAL 头文件
#include <stdint.h>

#define OLED_ADDRESS (0x3C << 1) // OLED I2C 地址（左移1位）

uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr, uint8_t fontSize);
void OLED_ShowString(uint8_t x, uint8_t y, char *str, uint8_t fontSize);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize);
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize);
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *BMP);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t fontSize);

#endif // OLED_H