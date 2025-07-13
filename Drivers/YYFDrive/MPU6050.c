#include "MPU6050.h"
#include <math.h> // 引入数学库

// 常量定义
#define RAD_TO_DEG 57.295779513082320876798154814105 // 弧度转角度
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY  131.0f

// 卡尔曼滤波器实例
static Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

static Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};

// 计时器变量
static uint32_t timer;


uint8_t MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // 1. 检查设备ID
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        // 2. 唤醒传感器
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // 3. 设置采样率
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // 4. 配置加速度计 (±2g)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // 5. 配置陀螺仪 (±250°/s)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        // 初始化计时器
        timer = HAL_GetTick();
        return 0; // 初始化成功
    }
    return 1; // 初始化失败
}

void MPU6050_Read_Accel(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
}

void MPU6050_Read_Gyro(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;
}

void MPU6050_Read_Temp(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp_raw;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);
    temp_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
}


void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[14];

    // 1. 一次性读取14字节数据
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    // 2. 解析原始数据
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    int16_t temp_raw = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 3. 转换为物理量
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;

    // 4. 卡尔曼滤波计算
    // 计算dt
    double dt = (double)(HAL_GetTick() - timer) / 1000.0;
    timer = HAL_GetTick();

    // 从加速度计原始数据计算Roll和Pitch
    double roll  = atan2(DataStruct->Accel_Y_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    double pitch = atan(-DataStruct->Accel_X_RAW / sqrt(DataStruct->Accel_Y_RAW * DataStruct->Accel_Y_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW)) * RAD_TO_DEG;

    // 调用卡尔曼函数进行滤波
    // Gyro_X 用于计算Pitch, Gyro_Y 用于计算Roll (这取决于传感器安装方向)
    // 根据参考代码，我们使用 Gx 作为 roll 的速率，Gy 作为 pitch 的速率
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    // 预测
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    // 更新误差协方差矩阵
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // 更新
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