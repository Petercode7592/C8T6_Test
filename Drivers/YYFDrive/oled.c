#include "oled.h"
#include <math.h> // 需要包含 math.h 以使用 pow 函数

// 包含字库数据
#include "oledfont.h"

// --- 模块内部全局变量 ---
// 使用 static 关键字，表示这个变量只在当前文件内可见
static I2C_HandleTypeDef *g_hi2c_oled;


// --- 模块内部辅助函数 ---

/**
 * @brief 向OLED写入一个字节
 * @param data 要写入的数据
 * @param is_data 标志位 (1:数据, 0:命令)
 */
static void OLED_WR_Byte(uint8_t data, uint8_t is_data) {
    uint8_t tx_buf[2];

    // I2C通信时，第一个字节是控制字节
    // 如果is_data为1，发送数据；否则，发送命令
    tx_buf[0] = is_data ? 0x40 : 0x00;
    tx_buf[1] = data;

    // 使用HAL库函数通过硬件I2C发送数据
    HAL_I2C_Master_Transmit(g_hi2c_oled, OLED_ADDRESS, tx_buf, 2, HAL_MAX_DELAY);
}


// --- 对外公开函数的实现 ---

/**
 * @brief 初始化OLED屏幕
 */
uint8_t OLED_Init(I2C_HandleTypeDef *hi2c) {
    // 保存用户传入的I2C句柄
    g_hi2c_oled = hi2c;

    // 检查I2C设备是否就绪
    if (HAL_I2C_IsDeviceReady(g_hi2c_oled, OLED_ADDRESS, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 1; // 设备未就绪
    }

    // --- OLED初始化命令序列 ---
    HAL_Delay(100); // 初始化前先延时

    OLED_WR_Byte(0xAE, 0); // 关闭显示
    OLED_WR_Byte(0x00, 0); // 设置低列地址
    OLED_WR_Byte(0x10, 0); // 设置高列地址
    OLED_WR_Byte(0x40, 0); // 设置显示起始行
    OLED_WR_Byte(0xB0, 0); // 设置页地址
    OLED_WR_Byte(0x81, 0); // 设置对比度控制
    OLED_WR_Byte(0xFF, 0); // 亮度 0x00~0xFF
    OLED_WR_Byte(0xA1, 0); // 设置段重映射 (左右反置) 0xA0/A1
    OLED_WR_Byte(0xA6, 0); // 设置正常/反向显示 0xA6/A7
    OLED_WR_Byte(0xA8, 0); // 设置多路复用率
    OLED_WR_Byte(0x3F, 0); // 1/64 duty
    OLED_WR_Byte(0xC8, 0); // 设置COM扫描方向 (上下反置) 0xC0/C8
    OLED_WR_Byte(0xD3, 0); // 设置显示偏移
    OLED_WR_Byte(0x00, 0);
    OLED_WR_Byte(0xD5, 0); // 设置显示时钟分频比/振荡器频率
    OLED_WR_Byte(0x80, 0);
    OLED_WR_Byte(0xD9, 0); // 设置预充电周期
    OLED_WR_Byte(0xF1, 0);
    OLED_WR_Byte(0xDA, 0); // 设置COM引脚硬件配置
    OLED_WR_Byte(0x12, 0);
    OLED_WR_Byte(0xDB, 0); // 设置VCOMH取消选择级别
    OLED_WR_Byte(0x40, 0);
    OLED_WR_Byte(0x20, 0); // 设置内存寻址模式
    OLED_WR_Byte(0x02, 0); // 页寻址模式
    OLED_WR_Byte(0x8D, 0); // 设置电荷泵
    OLED_WR_Byte(0x14, 0); // 使能电荷泵
    OLED_WR_Byte(0xA4, 0); // 全局显示开启; 0xA5-忽略RAM
    OLED_WR_Byte(0xAF, 0); // 打开显示

    OLED_Clear(); // 清屏
    return 0; // 初始化成功
}

/**
 * @brief 清空整个屏幕
 */
void OLED_Clear(void) {
    uint8_t i, n;
    for (i = 0; i < 8; i++) {
        OLED_WR_Byte(0xb0 + i, 0); // 设置页地址 (0~7)
        OLED_WR_Byte(0x00, 0);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, 0);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++) {
            OLED_WR_Byte(0, 1);
        }
    }
}

/**
 * @brief 设置光标位置
 */
void OLED_SetCursor(uint8_t x, uint8_t y) {
    OLED_WR_Byte(0xb0 + y, 0);
    OLED_WR_Byte((x & 0x0F), 0);
    OLED_WR_Byte(((x & 0xF0) >> 4) | 0x10, 0);
}

/**
 * @brief 在指定位置显示一个ASCII字符
 */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch, uint8_t fontSize) {
    uint8_t c = ch - ' '; // 得到偏移后的值
    uint8_t i = 0;
    if (x > 127) { x = 0; y += (fontSize == 16) ? 2 : 1; }

    if (fontSize == 16) {
        OLED_SetCursor(x, y);
        for (i = 0; i < 8; i++) OLED_WR_Byte(F8x16[c * 16 + i], 1);
        OLED_SetCursor(x, y + 1);
        for (i = 0; i < 8; i++) OLED_WR_Byte(F8x16[c * 16 + i + 8], 1);
    } else { // 默认6x8字体
        OLED_SetCursor(x, y);
        for (i = 0; i < 6; i++) OLED_WR_Byte(F6x8[c][i], 1);
    }
}

/**
 * @brief 在指定位置显示一个字符串
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *str, uint8_t fontSize) {
    uint8_t j = 0;
    while (str[j] != '\0') {
        OLED_ShowChar(x, y, str[j], fontSize);
        x += (fontSize == 16) ? 8 : 6;
        if (x > 120) { x = 0; y += (fontSize == 16) ? 2 : 1; }
        j++;
    }
}

// 幂函数
static uint32_t oled_pow(uint8_t m, uint8_t n) {
    uint32_t result = 1;
    while (n--) result *= m;
    return result;
}

/**
 * @brief 在指定位置显示一个无符号数字
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t fontSize) {
    uint8_t i;
    for (i = 0; i < len; i++) {
        OLED_ShowChar(x + i * ((fontSize == 16) ? 8 : 6), y, num / oled_pow(10, len - i - 1) % 10 + '0', fontSize);
    }
}

/**
 * @brief 在指定位置显示一个有符号数字
 */
void OLED_ShowSignedNum(uint8_t x, uint8_t y, int32_t num, uint8_t len, uint8_t fontSize) {
    uint8_t i;
    uint32_t num_abs;
    if (num < 0) {
        OLED_ShowChar(x, y, '-', fontSize);
        num_abs = -num;
    } else {
        OLED_ShowChar(x, y, '+', fontSize);
        num_abs = num;
    }
    for (i = 0; i < len; i++) {
        OLED_ShowChar(x + (i + 1) * ((fontSize == 16) ? 8 : 6), y, num_abs / oled_pow(10, len - i - 1) % 10 + '0', fontSize);
    }
}

/**
 * @brief 在指定位置显示一个汉字
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t index) {
    uint8_t i;
    const uint8_t *p_data = &HZK[index][0];

    // 汉字是16x16的，需要占用两个页
    OLED_SetCursor(x, y);
    for (i = 0; i < 16; i++) {
        OLED_WR_Byte(p_data[i], 1);
    }
    OLED_SetCursor(x, y + 1);
    for (i = 16; i < 32; i++) {
        OLED_WR_Byte(p_data[i], 1);
    }
}

/**
 * @brief 绘制位图
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t *BMP) {
    uint16_t j = 0;
    uint8_t x, y;

    if (y1 % 8 == 0) y = y1 / 8;
    else y = y1 / 8 + 1;

    for (y = y0; y < y1; y++) {
        OLED_SetCursor(x0, y);
        for (x = x0; x < x1; x++) {
            OLED_WR_Byte(BMP[j++], 1);
        }
    }
}