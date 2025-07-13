#include "MPU6050.h"
#include <math.h>

// --- 常量定义 ---
#define RAD_TO_DEG 57.295779513
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY  131.0f

// --- 静态全局变量 ---
static Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };
static Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

static uint32_t timer;
static double currentYaw = 0.0;

// 新增：用于存储陀螺仪XYZ三轴偏置的变量
static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;


// --- 私有函数声明 ---
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


// --- 公共函数定义 ---

uint8_t MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
        timer = HAL_GetTick();
        return 0;
    }
    return 1;
}

/**
 * @brief 新增的校准函数
 */
void MPU6050_Calibrate_Gyro(void)
{
    uint8_t Rec_Data[6];
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;
    int i;

    // 读取2000次数据进行平均
    const int num_samples = 2000;
    for (i = 0; i < num_samples; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
        gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;

        HAL_Delay(2); // 每次读取之间短暂延时
    }

    // 计算平均值并转换为物理量偏置
    gx_bias = (float)gx_sum / num_samples / GYRO_SENSITIVITY;
    gy_bias = (float)gy_sum / num_samples / GYRO_SENSITIVITY;
    gz_bias = (float)gz_sum / num_samples / GYRO_SENSITIVITY;
}


void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp_raw;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    // temp_raw = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
    // DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;

    double dt = (double)(HAL_GetTick() - timer) / 1000.0;
    timer = HAL_GetTick();

    double roll  = atan2(DataStruct->Accel_Y_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    double pitch = atan(-DataStruct->Accel_X_RAW / sqrt(DataStruct->Accel_Y_RAW * DataStruct->Accel_Y_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW)) * RAD_TO_DEG;

    // 在卡尔曼滤波前，也减去偏置，可以使Roll和Pitch更稳定
    double gx_calibrated = DataStruct->Gx - gx_bias;
    double gy_calibrated = DataStruct->Gy - gy_bias;
    double gz_calibrated = DataStruct->Gz - gz_bias;

    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, gx_calibrated, dt);
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, gy_calibrated, dt);

    // 使用校准后的Gz值进行积分
    currentYaw += gz_calibrated * dt;
    DataStruct->YawAngle = currentYaw;
}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}