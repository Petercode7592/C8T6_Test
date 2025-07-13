#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"

// 根据您在CubeMX中的配置，修改为正确的I2C句柄，例如 hi2c2
extern I2C_HandleTypeDef hi2c1;

// MPU6050 的 I2C 地址 (如果 AD0 接地，地址为 0x68)
#define MPU6050_ADDR 0xD0

// MPU6050 内部寄存器地址
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_XOUT_H_REG    0x3B
#define TEMP_OUT_H_REG      0x41
#define GYRO_CONFIG_REG     0x1B
#define GYRO_XOUT_H_REG     0x43

// MPU6050 数据结构体，用于存储所有相关数据
typedef struct
{
    // --- 原始数据 (16位，有符号) ---
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // --- 经过转换的实际物理量 ---
    float Ax, Ay, Az; // 加速度，单位 g
    float Gx, Gy, Gz; // 角速度，单位 °/s

    float Temperature; // 温度，单位 °C

    // --- 滤波/计算后的角度 ---
    double KalmanAngleX; // X轴（Roll）卡尔曼滤波后的角度
    double KalmanAngleY; // Y轴（Pitch）卡尔曼滤波后的角度
    double YawAngle;     // Z轴（Yaw）积分得到的相对航向角

} MPU6050_Data_t;

// 卡尔曼滤波器结构体
typedef struct {
    double Q_angle;     // 过程噪声协方差 (角度)
    double Q_bias;      // 过程噪声协方差 (陀螺仪偏置)
    double R_measure;   // 测量噪声协方差
    double angle;       // 估计的角度
    double bias;        // 陀螺仪偏置
    double P[2][2];     // 误差协方差矩阵
} Kalman_t;


// --- 函数声明 ---

/**
 * @brief 初始化 MPU6050 传感器
 * @retval 0: 初始化成功
 * @retval 1: 初始化失败 (设备未找到)
 */
uint8_t MPU6050_Init(void);

/**
 * @brief 一次性读取 MPU6050 所有传感器数据，并进行卡尔曼滤波和航向角计算
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct);


#endif /* MPU6050_H_ */