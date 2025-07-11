#ifndef MPU6050_H_
#define MPU6050_H_

#include "main.h" // 包含您的主头文件，通常由CubeMX生成，里面包含了HAL库的引用

// 定义一个外部的I2C句柄，您需要在main.c中定义它
// 注意：请根据您在CubeMX中的配置，修改为正确的I2C句柄，例如 hi2c2
extern I2C_HandleTypeDef hi2c1;

// MPU6050 的 I2C 地址
// 如果 AD0 引脚接地，地址为 0x68，左移一位后为 0xD0
#define MPU6050_ADDR 0xD0

// MPU6050 的内部寄存器地址
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define SMPLRT_DIV_REG      0x19
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_XOUT_H_REG    0x3B
#define TEMP_OUT_H_REG      0x41
#define GYRO_CONFIG_REG     0x1B
#define GYRO_XOUT_H_REG     0x43

// 定义一个结构体来存储从 MPU6050 读取的所有数据
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

} MPU6050_Data_t;


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
 * @brief 一次性读取 MPU6050 所有传感器数据
 * @param DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储数据
 */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct);


#endif /* MPU6050_H_ */