//
// Created by Petercode on 25-7-11.
//

#ifndef MPU6050_H
#define MPU6050_H


#include "main.h" // 包含STM32CubeMX生成的主头文件

// MPU6050 数据结构体，用于存放一次读取的所有数据
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Temperature;

} MPU6050_Data_t;

/**
 * @brief 初始化MPU6050传感器
 * @param hi2c_mpu6050 指向您在CubeMX中配置的I2C句柄的指针 (例如 &hi2c1)
 * @return uint8_t 状态码 (0: 成功, 非0: 失败)
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c_mpu6050);

/**
 * @brief 从MPU6050读取所有传感器原始数据
 * @param MPU6050_Data 指向用于存储数据的结构体的指针
 */
void MPU6050_Read_All(MPU6050_Data_t *MPU6050_Data);


#endif //MPU6050_H
