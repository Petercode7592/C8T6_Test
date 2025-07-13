#ifndef HW290B_H
#define HW290B_H

#include "stm32f1xx_hal.h"

/**
 * @brief  HW-290B 10 轴传感器数据结构
 *         包含 MPU6050 三轴加速度／角速度、BMP085 温度／气压、
 *         QMC5883L 三轴磁强计的物理量（单位：g, °/s, °C, Pa, Gauss）
 */
typedef struct {
    /* MPU6050 加速度 (g) */
    float ax;    // X 轴加速度
    float ay;    // Y 轴加速度
    float az;    // Z 轴加速度

    /* MPU6050 角速度 (°/s) */
    float gx;    // X 轴角速度
    float gy;    // Y 轴角速度
    float gz;    // Z 轴角速度

    /* BMP085 温度 (°C) */
    float temperature;  // 温度

    /* BMP085 气压 (Pa) */
    float pressure;     // 气压

    /* QMC5883L 磁力计 (Gauss) */
    float mx;    // X 轴磁力
    float my;    // Y 轴磁力
    float mz;    // Z 轴磁力
} HW290B_Data_t;

/**
 * @brief  初始化所有子模块（MPU6050 / BMP085 / QMC5883L）
 * @param  hi2c  指向已初始化 I2C 句柄
 *
 * 仅需在系统启动时调用一次
 */
void HW290B_InitHandle(I2C_HandleTypeDef *hi2c);

/**
 * @brief  读取 MPU6050 的加速度与角速度到 data
 * @param  hi2c  指向已初始化 I2C 句柄
 * @param  data  指向 HW290B_Data_t 结构体
 */
void HW290B_ReadAccelGyro(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data);

/**
 * @brief  读取 BMP085 的温度与气压到 data
 * @param  hi2c  指向已初始化 I2C 句柄
 * @param  data  指向 HW290B_Data_t 结构体
 */
void HW290B_ReadTempPress(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data);

/**
 * @brief  读取 QMC5883L 的三轴磁力到 data
 * @param  hi2c  指向已初始化 I2C 句柄
 * @param  data  指向 HW290B_Data_t 结构体
 */
void HW290B_ReadMag(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data);

/**
 * @brief  一次性读取所有传感器数据到 data
 * @param  hi2c  指向已初始化 I2C 句柄
 * @param  data  指向 HW290B_Data_t 结构体
 */
void HW290B_ReadAll(I2C_HandleTypeDef *hi2c, HW290B_Data_t *data);

#endif  /* __HW290B_H */
