#include "HW290B.h"
#include "stm32f1xx_hal.h"
#include <math.h>

/* —— MPU6050 定义 —— */
#define MPU_ADDR        (0x68 << 1)    // 7-bit 地址 0x68，HAL 要求左移 1 位
#define PWR_MGMT_1      0x6B           // 电源管理寄存器
#define GYRO_CONFIG     0x1B           // 陀螺量程配置寄存器
#define ACCEL_CONFIG    0x1C           // 加速度量程配置寄存器
#define ACCEL_XOUT_H    0x3B           // 加速度输出起始寄存器

/* —— BMP085 定义 —— */
#define BMP_ADDR        (0x77 << 1)    // 7-bit 地址 0x77 左移
#define BMP_CALIB_START 0xAA           // 校准参数起始寄存器

/* —— QMC5883L 定义 —— */
#define QMC_ADDR        (0x0D << 1)    // QMC5883L 7-bit 地址 0x0D 左移
#define QMC_REG_OUT_X_L 0x00           // 输出寄存器：X LSB
#define QMC_REG_OUT_X_H 0x01           // 输出寄存器：X MSB
#define QMC_REG_OUT_Y_L 0x02           // 输出寄存器：Y LSB
#define QMC_REG_OUT_Y_H 0x03           // 输出寄存器：Y MSB
#define QMC_REG_OUT_Z_L 0x04           // 输出寄存器：Z LSB
#define QMC_REG_OUT_Z_H 0x05           // 输出寄存器：Z MSB
#define QMC_REG_STATUS  0x06           // 状态寄存器
#define QMC_REG_CTRL1   0x09           // 控制寄存器 1：OSR, RNG, ODR, MODE :contentReference[oaicite:1]{index=1}
#define QMC_REG_CTRL2   0x0A           // 控制寄存器 2：软复位等

/* 本地缓存 BMP085 校准参数 */
static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;

/* 私有函数声明 */
static void HW_MPU6000_Init(I2C_HandleTypeDef *hi2c);

static void HW_MPU6000_Read(I2C_HandleTypeDef *hi2c,
                            float *ax, float *ay, float *az,
                            float *gx, float *gy, float *gz);

static void HW_BMP085_Init(I2C_HandleTypeDef *hi2c);

static void HW_BMP085_Read(I2C_HandleTypeDef *hi2c,
                           float *temperature, float *pressure);

static void HW_QMC5883L_Init(I2C_HandleTypeDef *hi2c);

static void HW_QMC5883L_Read(I2C_HandleTypeDef *hi2c,
                             float *mx, float *my, float *mz);

/* —— 公共 API —— */

void HW290B_InitHandle(I2C_HandleTypeDef *hi2c) {
    /* 唤醒并配置 MPU6050 */
    HW_MPU6000_Init(hi2c);
    /* 读取 BMP085 校准参数 */
    HW_BMP085_Init(hi2c);
    /* 配置 QMC5883L 连续测量、±8G、200Hz、OSR=512 */
    HW_QMC5883L_Init(hi2c);
}

void HW290B_ReadAccelGyro(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data) {
    HW_MPU6000_Read(hi2c,
                    &data->ax, &data->ay, &data->az,
                    &data->gx, &data->gy, &data->gz);
}

void HW290B_ReadTempPress(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data) {
    HW_BMP085_Read(hi2c,
                   &data->temperature,
                   &data->pressure);
}

void HW290B_ReadMag(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data) {
    HW_QMC5883L_Read(hi2c,
                     &data->mx, &data->my, &data->mz);
}

void HW290B_ReadAll(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data) {
    HW290B_ReadAccelGyro(hi2c, data);
    // HW290B_ReadTempPress(hi2c, data);
    HW290B_ReadMag(hi2c, data);
}

/* —— 私有实现 —— */

static void HW_MPU6000_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t cfg;

    /* 退出睡眠 */
    cfg = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, PWR_MGMT_1, 1, &cfg, 1, 100);

    /* 陀螺量程 FS_SEL=0 → ±250°/s, 131 LSB/°/s, ±3%  */
    cfg = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, GYRO_CONFIG, 1, &cfg, 1, 100);

    /* 加速度量程 AFS_SEL=2 → ±8g, 4096 LSB/g, ±3%  */
    cfg = (2 << 3);
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &cfg, 1, 100);
}

static void HW_MPU6000_Read(I2C_HandleTypeDef *hi2c,
                            float *ax, float *ay, float *az,
                            float *gx, float *gy, float *gz) {
    uint8_t buf[14];
    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, ACCEL_XOUT_H, 1, buf, 14, 200);

    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    /* 按量程换算到物理单位 */
    *ax = raw_ax / 4096.0f;
    *ay = raw_ay / 4096.0f;
    *az = raw_az / 4096.0f;
    *gx = raw_gx / 131.0f;
    *gy = raw_gy / 131.0f;
    *gz = raw_gz / 131.0f;
}

static void HW_BMP085_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t buf[22];
    HAL_I2C_Mem_Read(hi2c, BMP_ADDR, BMP_CALIB_START, 1, buf, 22, 200);

    AC1 = (buf[0] << 8) | buf[1];
    AC2 = (buf[2] << 8) | buf[3];
    AC3 = (buf[4] << 8) | buf[5];
    AC4 = (buf[6] << 8) | buf[7];
    AC5 = (buf[8] << 8) | buf[9];
    AC6 = (buf[10] << 8) | buf[11];
    B1 = (buf[12] << 8) | buf[13];
    B2 = (buf[14] << 8) | buf[15];
    MB = (buf[16] << 8) | buf[17];
    MC = (buf[18] << 8) | buf[19];
    MD = (buf[20] << 8) | buf[21];
}

static void HW_BMP085_Read(I2C_HandleTypeDef *hi2c,
                           float *temperature, float *pressure) {
    uint8_t cmd, buf[3];
    int32_t UT, UP, X1, X2, B5;

    /* 温度测量 */
    cmd = 0x2E;
    HAL_I2C_Mem_Write(hi2c, BMP_ADDR, 0xF4, 1, &cmd, 1, 100);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(hi2c, BMP_ADDR, 0xF6, 1, buf, 2, 100);
    UT = (buf[0] << 8) | buf[1];

    /* 气压测量 (OSS=0) */
    cmd = 0x34;
    HAL_I2C_Mem_Write(hi2c, BMP_ADDR, 0xF4, 1, &cmd, 1, 100);
    HAL_Delay(8);
    HAL_I2C_Mem_Read(hi2c, BMP_ADDR, 0xF6, 1, buf, 3, 100);
    UP = (((int32_t) buf[0] << 16) | ((int32_t) buf[1] << 8) | buf[2]) >> 8;

    /* 温度换算 */
    X1 = ((UT - AC6) * AC5) >> 15;
    X2 = (MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    *temperature = (B5 + 8) >> 4;

    /* 气压换算 */
    {
        int32_t B6 = B5 - 4000;
        X1 = (B2 * (B6 * B6 >> 12)) >> 11;
        X2 = (AC2 * B6) >> 11;
        int32_t X3 = X1 + X2;
        int32_t B3 = (((int32_t) AC1 * 4 + X3) + 2) >> 2;
        X1 = (AC3 * B6) >> 13;
        X2 = (B1 * (B6 * B6 >> 12)) >> 16;
        X3 = ((X1 + X2) + 2) >> 2;
        uint32_t B4 = (AC4 * (uint32_t) (X3 + 32768)) >> 15;
        uint32_t B7 = ((uint32_t) UP - B3) * 50000;
        int32_t p = (B7 < 0x80000000U)
                        ? (B7 << 1) / B4
                        : (B7 / B4) << 1;
        X1 = (p >> 8) * (p >> 8);
        X1 = (X1 * 3038) >> 16;
        X2 = (-7357 * p) >> 16;
        *pressure = p + ((X1 + X2 + 3791) >> 4);
    }
}

static void HW_QMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t cfg1;

    /* OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous → 0b00011101 = 0x1D :contentReference[oaicite:4]{index=4} */
    cfg1 = 0x1D;
    HAL_I2C_Mem_Write(hi2c, QMC_ADDR, QMC_REG_CTRL1, 1, &cfg1, 1, 100);

    /* Control2 软复位、指针回绕、INT 引脚等保持默认即可 */
}

static void HW_QMC5883L_Read(I2C_HandleTypeDef *hi2c,
                             float *mx, float *my, float *mz) {
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    /* 读 6 字节：LSB/MSB 顺序 */
    HAL_I2C_Mem_Read(hi2c, QMC_ADDR, QMC_REG_OUT_X_L, 1, buf, 6, 100);

    raw_x = (int16_t) ((buf[1] << 8) | buf[0]);
    raw_y = (int16_t) ((buf[3] << 8) | buf[2]);
    raw_z = (int16_t) ((buf[5] << 8) | buf[4]);

    /* ±8G 对应 4096 LSB/Gauss → 转换为 Gauss */
    *mx = raw_x / 4096.0f;
    *my = raw_y / 4096.0f;
    *mz = raw_z / 4096.0f;
}
