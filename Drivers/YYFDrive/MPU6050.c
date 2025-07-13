#include "MPU6050.h"
#include <math.h> // 引入数学库以使用 atan2, sqrt 等函数

// --- 常量定义 ---
#define RAD_TO_DEG 57.295779513082320876798154814105 // 弧度转角度的系数
#define ACCEL_SENSITIVITY 16384.0f // 加速度计灵敏度 (FS_SEL=0, ±2g)
#define GYRO_SENSITIVITY  131.0f   // 陀螺仪灵敏度 (FS_SEL=0, ±250°/s)

// --- 静态全局变量 ---

// X轴和Y轴的卡尔曼滤波器实例
// 这些参数(Q, R)可以根据实际情况进行微调以达到最佳滤波效果
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

// 用于计算时间间隔(dt)的计时器变量
static uint32_t timer;
// 用于积分计算Z轴相对航向角的变量
static double currentYaw = 0.0;


// --- 私有函数声明 ---
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


// --- 公共函数定义 ---

uint8_t MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // 1. 检查设备ID (WHO_AM_I 寄存器)
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68) // 0x68 是 MPU6050 的正确设备ID
    {
        // 2. 唤醒传感器 (写入 0 到电源管理寄存器1)
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // 3. 设置采样率 (写入 0x07 到采样率分频寄存器，使采样率=陀螺仪输出速率/8 = 1KHz)
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // 4. 配置加速度计 (写入 0x00 到加速度计配置寄存器，设置为 ±2g 量程)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // 5. 配置陀螺仪 (写入 0x00 到陀螺仪配置寄存器，设置为 ±250°/s 量程)
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        // 初始化计时器，用于第一次dt计算
        timer = HAL_GetTick();
        return 0; // 初始化成功
    }
    return 1; // 初始化失败
}


void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp_raw;

    // 1. 一次性读取14字节数据 (加速度计3轴 + 温度 + 陀螺仪3轴)
    // 这样做比分次读取效率更高
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    // 2. 解析原始数据 (高8位和低8位合并为16位有符号整数)
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp_raw = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 3. 将原始数据转换为实际物理量
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53; // 根据数据手册公式转换
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;

    // 4. 计算和滤波
    // 计算两次测量之间的时间差 (dt)，单位为秒
    double dt = (double)(HAL_GetTick() - timer) / 1000.0;
    timer = HAL_GetTick();

    // 根据加速度计数据计算Roll和Pitch角
    double roll  = atan2(DataStruct->Accel_Y_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    double pitch = atan(-DataStruct->Accel_X_RAW / sqrt(DataStruct->Accel_Y_RAW * DataStruct->Accel_Y_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW)) * RAD_TO_DEG;

    // 调用卡尔曼函数，对Roll和Pitch进行滤波
    // 注意：Gx 用于计算 roll 角, Gy 用于计算 pitch 角。这取决于传感器的物理安装方向。
    // 这里我们遵循您提供的参考代码的逻辑。
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
    DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);

    // 积分Z轴角速度，计算相对航向角
    // 注意：这个角度会因为陀螺仪的漂移而不断累积误差
    currentYaw += DataStruct->Gz * dt;
    DataStruct->YawAngle = currentYaw;
}


// --- 私有函数定义 ---

/**
 * @brief 卡尔曼滤波核心算法
 * @param Kalman: 指向卡尔曼结构体的指针
 * @param newAngle: 新的角度测量值 (来自加速度计)
 * @param newRate: 新的角速度测量值 (来自陀螺仪)
 * @param dt: 时间间隔 (s)
 * @retval 滤波后的角度
 */
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    // 预测阶段
    // 1. 状态预测：角度 = 上一次的角度 + (角速度 - 偏置) * dt
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    // 2. 预测误差协方差
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    // 更新阶段
    // 1. 计算卡尔曼增益
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    // 2. 更新状态估计：角度 = 预测角度 + K[0] * (测量角度 - 预测角度)
    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    // 更新偏置：偏置 = 上一次偏置 + K[1] * (测量角度 - 预测角度)
    Kalman->bias += K[1] * y;

    // 3. 更新误差协方差
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}