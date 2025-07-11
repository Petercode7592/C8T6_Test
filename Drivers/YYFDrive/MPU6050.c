#include "MPU6050.h"

// 加速度计和陀螺仪的灵敏度因子
// 这些值取决于你在初始化时设置的量程
// 当前配置下 (加速度计±2g, 陀螺仪±250°/s) 的灵敏度
#define ACCEL_SENSITIVITY 16384.0f
#define GYRO_SENSITIVITY  131.0f

uint8_t MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // 1. 检查设备ID (WHO_AM_I)
    // 尝试从 WHO_AM_I_REG (0x75) 读取一个字节
    // MPU6050 的 ID 应该是 0x68
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)  // 如果设备ID正确
    {
        // 2. 唤醒传感器
        // 向电源管理寄存器1 (PWR_MGMT_1_REG) 写入 0
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // 3. 设置采样率
        // 向采样率分频寄存器 (SMPLRT_DIV_REG) 写入 0x07，设置采样率为 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // 4. 配置加速度计
        // 向加速度计配置寄存器 (ACCEL_CONFIG_REG) 写入 0x00，设置为 ±2g 量程
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // 5. 配置陀螺仪
        // 向陀螺仪配置寄存器 (GYRO_CONFIG_REG) 写入 0x00，设置为 ±250°/s 量程
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

        return 0; // 初始化成功
    }
    return 1; // 初始化失败
}

void MPU6050_Read_Accel(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从 ACCEL_XOUT_H_REG (0x3B) 开始，连续读取6个字节
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // 组合高低字节，得到16位的原始数据
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 将原始数据转换为实际物理量 (g)
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
}

void MPU6050_Read_Gyro(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从 GYRO_XOUT_H_REG (0x43) 开始，连续读取6个字节
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

    // 组合高低字节，得到16位的原始数据
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    // 将原始数据转换为实际物理量 (°/s)
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;
}

void MPU6050_Read_Temp(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp_raw;

    // 从 TEMP_OUT_H_REG (0x41) 开始，连续读取2个字节
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, 1000);

    // 组合高低字节
    temp_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    // 根据数据手册的公式，将原始数据转换为摄氏度
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
}


void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[14];

    // 从 ACCEL_XOUT_H_REG (0x3B) 开始，连续读取14个字节
    // 这样可以一次性读出加速度计、温度、陀螺仪的所有数据，效率更高
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    // 分别解析数据
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    int16_t temp_raw = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 转换
    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_SENSITIVITY;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_SENSITIVITY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;
}