#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"

// 根据您在CubeMX中的配置，修改为正确的I2C句柄
extern I2C_HandleTypeDef hi2c1;

// MPU6050 的 I2C 地址
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

// MPU6050 数据结构体
typedef struct
{
    // --- 原始数据 ---
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // --- 经过转换的实际物理量 ---
    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float Temperature;

    // --- 滤波/计算后的角度 ---
    double KalmanAngleX;
    double KalmanAngleY;
    double YawAngle;

} MPU6050_Data_t;

// 卡尔曼滤波器结构体
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;


// --- 函数声明 ---

/**
 * @brief 初始化 MPU6050 传感器
 * @retval 0: 初始化成功, 1: 失败
 */
uint8_t MPU6050_Init(void);

/**
 * @brief 校准陀螺仪。请在设备静止时调用此函数！
 */
void MPU6050_Calibrate_Gyro(void);

/**
 * @brief 读取所有传感器数据，并进行滤波和角度计算
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针
 */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct);


#endif /* MPU6050_H_ */