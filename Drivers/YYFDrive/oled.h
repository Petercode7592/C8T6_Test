#ifndef OLED_H
#define OLED_H

// 包含 STM32 HAL 库的主头文件，以获取标准类型和HAL函数原型
#include "main.h"

// 宏定义OLED的I2C地址（请根据您的模块修改，通常是0x3C或0x3D）
// HAL库的I2C地址需要左移一位
#define OLED_ADDRESS (0x3C << 1)

/**
 * @brief 初始化OLED屏幕
 * @param hi2c 指向您在CubeMX中配置的I2C句柄的指针 (例如 &hi2c1)
 * @return uint8_t 状态码 (0: 成功, 非0: 失败)
 */
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief 清空整个屏幕
 */
void OLED_Clear(void);

/**
 * @brief 设置光标位置
 * @param x X坐标 (0-127)
 * @param y Y坐标 (0-7)
 */
void OLED_SetCursor(uint8_t x, uint8_t y);

/**
 * @brief 在指定位置显示一个ASCII字符
 * @param x X坐标
 * @param y Y坐标
 * @param ch 要显示的字符
 * @param fontSize 字体大小 (可选 16 或 8)
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize);

/**
 * @brief 在指定位置显示一个字符串
 * @param x X坐标
 * @param y Y坐标
 * @param str 要显示的字符串
 * @param fontSize 字体大小 (可选 16 或 8)
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *str, uint8_t fontSize);

/**
 * @brief 在指定位置显示一个有符号数字
 * @param x X坐标
 * @param y Y坐标
 * @param num 要显示的数字
 * @param len 数字的长度
 * @param fontSize 字体大小
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize);

/**
 * @brief 在指定位置显示一个无符号数字
 * @param x X坐标
 * @param y Y坐标
 * @param num 要显示的数字
 * @param len 数字的长度
 * @param fontSize 字体大小
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize);


/**
 * @brief 在指定位置显示一个汉字
 * @param x X坐标
 * @param y Y坐标
 * @param index 汉字在内置字库中的索引
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index);


/**
 * @brief 绘制位图
 * @param x0 起始x坐标
 * @param y0 起始y坐标
 * @param x1 结束x坐标
 * @param y1 结束y坐标
 * @param BMP 位图数据指针
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *BMP);

#endif // OLED_H