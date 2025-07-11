//
// Created by Petercode on 25-7-11.
//

#include "MPU6050.h"


// MPU6050 I2C地址 (AD0接地时为0x68)
#define MPU6050_ADDR (0x68 << 1) // HAL库地址格式为左移一位

// MPU6050 内部寄存器地址
#define SMPLRT_DIV      0x19
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_XOUT_H    0x3B
#define PWR_MGMT_1      0x6B
#define WHO_AM_I        0x75

// 全局变量，用于保存I2C句柄
static I2C_HandleTypeDef *g_hi2c_mpu6050;

/**
 * @brief 初始化MPU6050传感器
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c_mpu6050) {
    uint8_t check;
    uint8_t data;

    g_hi2c_mpu6050 = hi2c_mpu6050;

    // 检查设备是否在线 (读取WHO_AM_I寄存器)
    if (HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 1000) != HAL_OK)
        return 1;
    if (check != 0x68)
        return 2;

    // 唤醒MPU6050
    data = 0;
    if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 1000) != HAL_OK)
        return 3;

    // 设置陀螺仪采样率
    data = 0x07;
    if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, 1000) != HAL_OK)
        return 4;

    // 设置陀螺仪配置 (±2000deg/s)
    data = 0x18;
    if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 1000) != HAL_OK)
        return 5;

    // 设置加速度计配置 (±2g)
    data = 0x00;
    if (HAL_I2C_Mem_Write(g_hi2c_mpu6050, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000) != HAL_OK)
        return 6;

    return 0; // 初始化成功
}

/**
 * @brief 从MPU6050读取所有传感器原始数据
 */
void MPU6050_Read_All(MPU6050_Data_t *MPU6050_Data) {
    uint8_t Rec_Data[14];
    int16_t temp_raw;

    // 从加速度计的第一个寄存器 ACCEL_XOUT_H (0x3B) 开始，连续读取14个字节
    HAL_I2C_Mem_Read(g_hi2c_mpu6050, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, 1000);

    // 组合数据
    MPU6050_Data->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    MPU6050_Data->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    MPU6050_Data->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    temp_raw = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    MPU6050_Data->Temperature = (float)temp_raw / 340.0f + 36.53f;

    MPU6050_Data->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    MPU6050_Data->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    MPU6050_Data->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
}