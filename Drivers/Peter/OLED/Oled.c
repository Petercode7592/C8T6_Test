#include "Oled.h"
#include <math.h>
#include "Oledfont.h"

static I2C_HandleTypeDef *g_hi2c_oled = NULL;

/**
 * @brief  向 OLED 写入一个字节（命令或数据）
 * @param  data: 要发送的数据
 * @param  is_data: 1 表示数据，0 表示命令
 */
static void OLED_WR_Byte(uint8_t data, uint8_t is_data) {
    uint8_t buffer[2];
    buffer[0] = is_data ? 0x40 : 0x00;
    buffer[1] = data;
    HAL_I2C_Master_Transmit(g_hi2c_oled, OLED_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

/**
 * @brief  初始化 OLED 显示屏
 * @param  hi2c: 指向已初始化的 I2C 句柄
 * @retval 0 初始化成功，1 失败
 */
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c) {
    g_hi2c_oled = hi2c;

    if (HAL_I2C_IsDeviceReady(g_hi2c_oled, OLED_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1;
    }

    HAL_Delay(100);
    OLED_WR_Byte(0xAE, 0);
    OLED_WR_Byte(0x00, 0);
    OLED_WR_Byte(0x10, 0);
    OLED_WR_Byte(0x40, 0);
    OLED_WR_Byte(0xB0, 0);
    OLED_WR_Byte(0x81, 0);
    OLED_WR_Byte(0xFF, 0);
    OLED_WR_Byte(0xA1, 0);
    OLED_WR_Byte(0xA6, 0);
    OLED_WR_Byte(0xA8, 0);
    OLED_WR_Byte(0x3F, 0);
    OLED_WR_Byte(0xC8, 0);
    OLED_WR_Byte(0xD3, 0);
    OLED_WR_Byte(0x00, 0);
    OLED_WR_Byte(0xD5, 0);
    OLED_WR_Byte(0x80, 0);
    OLED_WR_Byte(0xD9, 0);
    OLED_WR_Byte(0xF1, 0);
    OLED_WR_Byte(0xDA, 0);
    OLED_WR_Byte(0x12, 0);
    OLED_WR_Byte(0xDB, 0);
    OLED_WR_Byte(0x40, 0);
    OLED_WR_Byte(0x20, 0);
    OLED_WR_Byte(0x02, 0);
    OLED_WR_Byte(0x8D, 0);
    OLED_WR_Byte(0x14, 0);
    OLED_WR_Byte(0xA4, 0);
    OLED_WR_Byte(0xAF, 0);

    OLED_Clear();
    return 0;
}

/**
 * @brief 清屏（将所有像素清零）
 */
void OLED_Clear(void) {
    for (uint8_t page = 0; page < 8; page++) {
        OLED_WR_Byte(0xB0 + page, 0);
        OLED_WR_Byte(0x00, 0);
        OLED_WR_Byte(0x10, 0);
        for (uint8_t col = 0; col < 128; col++) {
            OLED_WR_Byte(0x00, 1);
        }
    }
}

/**
 * @brief 设置光标位置
 * @param x: 列坐标（0~127）
 * @param y: 页地址（0~7）
 */
void OLED_SetCursor(uint8_t x, uint8_t y) {
    OLED_WR_Byte(0xB0 + y, 0);
    OLED_WR_Byte(x & 0x0F, 0);
    OLED_WR_Byte(((x >> 4) & 0x0F) | 0x10, 0);
}

/**
 * @brief 显示一个 ASCII 字符
 * @param x: 起始列位置
 * @param y: 页地址
 * @param ch: 字符
 * @param fontSize: 字体大小（8或16）
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize) {
    uint8_t c = ch - ' ';
    if (x > 127) {
        x = 0;
        y += (fontSize == 16) ? 2 : 1;
    }

    if (fontSize == 16) {
        OLED_SetCursor(x, y);
        for (uint8_t i = 0; i < 8; i++) OLED_WR_Byte(F8x16[c * 16 + i], 1);
        OLED_SetCursor(x, y + 1);
        for (uint8_t i = 0; i < 8; i++) OLED_WR_Byte(F8x16[c * 16 + 8 + i], 1);
    } else {
        OLED_SetCursor(x, y);
        for (uint8_t i = 0; i < 6; i++) OLED_WR_Byte(F6x8[c][i], 1);
    }
}

/**
 * @brief 显示字符串
 * @param x: 起始列
 * @param y: 页地址
 * @param str: 字符串指针
 * @param fontSize: 字体大小
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *str, uint8_t fontSize) {
    while (*str) {
        OLED_ShowChar(x, y, *str, fontSize);
        x += (fontSize == 16) ? 8 : 6;
        if (x > 120) {
            x = 0;
            y += (fontSize == 16) ? 2 : 1;
        }
        str++;
    }
}

/**
 * @brief 计算幂 m^n
 * @param m: 底数
 * @param n: 指数
 */
static uint32_t oled_pow(uint8_t m, uint8_t n) {
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief 显示无符号整数
 * @param x: 起始列
 * @param y: 页地址
 * @param num: 要显示的数字
 * @param len: 位数
 * @param fontSize: 字体大小
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize) {
    for (uint8_t i = 0; i < len; i++) {
        uint8_t digit = num / oled_pow(10, len - i - 1) % 10;
        OLED_ShowChar(x + i * ((fontSize == 16) ? 8 : 6), y, digit + '0', fontSize);
    }
}

/**
 * @brief 显示有符号整数
 * @param x: 起始列
 * @param y: 页地址
 * @param num: 有符号数
 * @param len: 显示位数
 * @param fontSize: 字体大小
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize) {
    uint32_t abs_num = (num < 0) ? -num : num;
    OLED_ShowChar(x, y, (num < 0) ? '-' : '+', fontSize);
    for (uint8_t i = 0; i < len; i++) {
        uint8_t digit = abs_num / oled_pow(10, len - i - 1) % 10;
        OLED_ShowChar(x + (i + 1) * ((fontSize == 16) ? 8 : 6), y, digit + '0', fontSize);
    }
}

/**
 * @brief 显示一个汉字（16x16 点阵）
 * @param x: 起始列
 * @param y: 起始页
 * @param index: 汉字索引（在HZK表中）
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index) {
    const uint8_t *data = HZK[index];
    OLED_SetCursor(x, y);
    for (uint8_t i = 0; i < 16; i++) OLED_WR_Byte(data[i], 1);
    OLED_SetCursor(x, y + 1);
    for (uint8_t i = 16; i < 32; i++) OLED_WR_Byte(data[i], 1);
}

/**
 * @brief 显示 BMP 位图图像
 * @param x0: 起始列
 * @param y0: 起始页
 * @param x1: 结束列
 * @param y1: 结束页
 * @param BMP: 位图数组（列优先）
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *BMP) {
    uint16_t j = 0;
    for (uint8_t y = y0; y < y1; y++) {
        OLED_SetCursor(x0, y);
        for (uint8_t x = x0; x < x1; x++) {
            OLED_WR_Byte(BMP[j++], 1);
        }
    }
}


/**
 * @brief 显示浮点数（不带前导0，整数自动对齐）
 * @param x 起始列
 * @param y 页地址（0-7）
 * @param num 要显示的浮点数
 * @param int_len 整数部分最大位数（用于占位计算，不补0）
 * @param dec_len 小数位数
 * @param fontSize 字体大小（8 或 16）
 */
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t int_len, uint8_t dec_len, uint8_t fontSize) {
    uint8_t offset = 0;
    if (num < 0) {
        OLED_ShowChar(x, y, '-', fontSize);
        num = -num;
    } else {
        OLED_ShowChar(x, y, ' ', fontSize);  // 可选显示空格或 '+'
    }
    offset += 1;

    uint32_t int_part = (uint32_t)num;
    float fractional = num - int_part;
    uint32_t frac_part = (uint32_t)(fractional * oled_pow(10, dec_len) + 0.5f);  // 四舍五入

    // 1. 计算整数部分的真实位数（用于去前导0）
    uint8_t actual_digits = 0;
    uint32_t temp = int_part;
    do {
        actual_digits++;
        temp /= 10;
    } while (temp > 0);

    // 2. 逐位显示整数部分（从高位到低位）
    for (int i = 0; i < actual_digits; i++) {
        uint8_t digit = int_part / oled_pow(10, actual_digits - i - 1) % 10;
        OLED_ShowChar(x + (offset + i) * ((fontSize == 16) ? 8 : 6), y, digit + '0', fontSize);
    }

    // 3. 小数点
    OLED_ShowChar(x + (offset + actual_digits) * ((fontSize == 16) ? 8 : 6), y, '.', fontSize);

    // 4. 小数部分（用 ShowNum 固定位数）
    OLED_ShowNum(x + (offset + actual_digits + 1) * ((fontSize == 16) ? 8 : 6), y, frac_part, dec_len, fontSize);
}