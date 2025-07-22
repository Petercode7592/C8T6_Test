#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h"                     /* 包含 hi2c1 定义 */
extern I2C_HandleTypeDef hi2c1;

/* ───── I²C 地址 (AD0=LOW) ───── */
#define MPU6050_ADDR        0xD0      /* 0x68 << 1 */

/* ───── 寄存器地址 ───── */
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define CONFIG_REG          0x1A
#define ACCEL_CONFIG_REG    0x1C
#define GYRO_CONFIG_REG     0x1B
#define INT_PIN_CFG_REG     0x37
#define INT_ENABLE_REG      0x38
#define GYRO_ZOUT_H_REG     0x47  // Z轴陀螺仪数据的高位寄存器地址
#define INT_STATUS_REG      0x3A
#define ACCEL_XOUT_H_REG    0x3B
#define GYRO_XOUT_H_REG     0x43

/* ───── 物理比例常数 (±2 g, ±250 °/s) ───── */
#define ACCEL_SENSITIVITY   16384.0f
#define GYRO_SENSITIVITY    131.0f
#define RAD_TO_DEG          57.295779513

/* ───── 外部标志：PB9 INT 置 1 ───── */
extern volatile uint8_t mpu_data_ready;

/* ───── 数据结构 ───── */
typedef struct {
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    int16_t Gyro_X_RAW , Gyro_Y_RAW , Gyro_Z_RAW;
    float   Ax, Ay, Az;
    float   Gx, Gy, Gz;
    float   Temperature;
    double  KalmanAngleX, KalmanAngleY;
    double  YawAngle;
} MPU6050_Data_t;

/* 卡尔曼结构 */
typedef struct {
    double Q_angle, Q_bias, R_measure;
    double angle, bias;
    double P[2][2];
} Kalman_t;


uint8_t MPU6050_Init(I2C_HandleTypeDef *hc);
void    MPU6050_Calibrate_Gyro(void);
void    MPU6050_Read_All(MPU6050_Data_t *data);
void    MPU6050_Get_Angle_yaw(MPU6050_Data_t *DataStruct);
#endif
