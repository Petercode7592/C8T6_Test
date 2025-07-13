#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"

// 根据您在CubeMX中的配置，修改为正确的I2C句柄
extern I2C_HandleTypeDef hi2c1;

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
    float Ax, Ay, Az; // 加速度，单位 g
    float Gx, Gy, Gz; // 角速度，单位 °/s

    float Temperature; // 温度，单位 °C

    // --- 卡尔曼滤波后的角度 ---
    double KalmanAngleX;
    double KalmanAngleY;

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
 * @brief 从 MPU6050 读取加速度计数据
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_Accel(MPU6050_Data_t *DataStruct);

/**
 * @brief 从 MPU6050 读取陀螺仪数据
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_Gyro(MPU6050_Data_t *DataStruct);

/**
 * @brief 从 MPU6050 读取温度数据
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_Temp(MPU6050_Data_t *DataStruct);

/**
 * @brief 一次性读取 MPU6050 所有传感器数据并进行卡尔曼滤波
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct);

/**
 * @brief 卡尔曼滤波计算
 * @param Kalman: 指向卡尔曼结构体的指针
 * @param newAngle: 新的角度测量值 (来自加速度计)
 * @param newRate: 新的角速度测量值 (来自陀螺仪)
 * @param dt: 时间间隔 (s)
 * @retval 滤波后的角度
 */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


#endif /* MPU6050_H_ */