#include "MPU6050.h"
#include <math.h>


// --- 静态全局变量 ---
static Kalman_t KalmanX = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };
static Kalman_t KalmanY = { .Q_angle = 0.001f, .Q_bias = 0.003f, .R_measure = 0.03f };

static uint32_t timer;
static double currentYaw = 0.0;

// 新增：用于存储陀螺仪XYZ三轴偏置的变量
static float gx_bias=0, gy_bias=0, gz_bias=0;

I2C_HandleTypeDef *LWKhIIC;

// --- 私有函数声明 ---
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

static inline void wr(uint8_t reg, uint8_t val)
{
    uint8_t d[2] = {reg,val};
    HAL_I2C_Master_Transmit(LWKhIIC, MPU6050_ADDR, d, 2, 100);
}


/**
 * @brief 初始化 MPU6050 传感器
 * @retval 0 成功，1 失败
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hc)
{
    uint8_t id;
    HAL_I2C_Mem_Read(hc, MPU6050_ADDR, WHO_AM_I_REG,1,&id,1,100);
    if (id != 0x68) return 1;
    LWKhIIC = hc; // 保存 I2C 句柄
    wr(PWR_MGMT_1_REG, 0x01);    /* PLL with X-gyro */
    wr(SMPLRT_DIV_REG, 0x09);    /* 100 Hz */
    wr(CONFIG_REG,      0x03);   /* DLPF 42 Hz  */
    wr(ACCEL_CONFIG_REG,0x00);   /* ±2 g        */
    wr(GYRO_CONFIG_REG ,0x00);   /* ±250°/s     */
    wr(INT_PIN_CFG_REG ,0x10);   /* INT_RD_CLEAR=1 推挽高电平 */
    wr(INT_ENABLE_REG  ,0x01);   /* DATA_RDY_INT  */

    timer = HAL_GetTick();
    return 0;
}

/**
 * @brief MPU6050 陀螺仪校准
 * @note 读取2000次陀螺仪数据进行平均，计算偏置值
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
        HAL_I2C_Mem_Read(LWKhIIC, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
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


/**
 * @brief 读取 MPU6050 的所有传感器数据
 * @param DataStruct 指向 MPU6050_Data_t 结构体的指针
 * @note 此函数会读取加速度计、陀螺仪和温度传感器的数据，并进行转换
 */
void MPU6050_Read_All(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp_raw;

    HAL_I2C_Mem_Read(LWKhIIC, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp_raw = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW  = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_SENSITIVITY;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_SENSITIVITY;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_SENSITIVITY;
    DataStruct->Temperature = (temp_raw / 340.0) + 36.53;
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
    uint8_t dummy;
    HAL_I2C_Mem_Read(LWKhIIC, MPU6050_ADDR, INT_STATUS_REG, 1, &dummy, 1, 10); /* <<< 保证 INT 拉低 */
}

/**
 * @brief  通过积分Z轴陀螺仪数据来计算偏航角(Yaw)
 * @note   这个函数必须被周期性地、高频率地调用 (例如每 5-20ms 一次) 才能保证精度。
 * 这个方法存在固有的积分漂移，长时间运行后角度会不准确。
 * 为了校正漂移，需要结合加速度计或磁力计进行数据融合（如互补滤波或卡尔曼滤波）。
 * @param  DataStruct: 指向 MPU6050_Data_t 结构体的指针，用于存储结果
 */
void MPU6050_Get_Angle_yaw(MPU6050_Data_t *DataStruct)
{
    uint8_t Rec_Data[2]; // 优化点1: 只需要2个字节来存放Z轴陀螺仪数据

    // --- 1. 高效读取数据 ---
    // 只读取GYRO_Z的数据，而不是读取从加速度计开始的所有14个字节，大大减少I2C总线负担
    HAL_I2C_Mem_Read(LWKhIIC, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 100);

    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

    // --- 2. 使用 float 进行浮点运算 ---
    // 在没有硬件双精度FPU的单片机上(如STM32F1)，float比double快得多
    static uint32_t timer = 0; // 优化点2: 设为static，保证在函数调用间保持其值
    float dt = (float)(HAL_GetTick() - timer) / 1000.0f;
    timer = HAL_GetTick();

    // 如果dt为0（调用间隔小于1ms），则不进行计算，防止除零或无效积分
    if (dt <= 0) {
        return;
    }

    // --- 3. 积分计算 Yaw 角 ---
    // 将原始值转换为角速度 (单位: 度/秒)
    DataStruct->Gz = (float)DataStruct->Gyro_Z_RAW / GYRO_SENSITIVITY;

    // 减去零点漂移（偏置），这个gz_bias需要在设备静止时提前计算并标定
    float gz_calibrated = DataStruct->Gz - gz_bias;

    // 使用静态变量来累加角度值
    static float currentYaw = 0.0f; // 优化点3: 设为static，保存积分结果
    currentYaw += gz_calibrated * dt; // 欧拉积分

    // 可以在这里将角度限制在 0-360 度或 -180~180 度范围内，防止无限增大
    // fmodf 是 float 版本的求余函数
    currentYaw = fmodf(currentYaw, 360.0f);

    DataStruct->YawAngle = currentYaw;

    // --- 4. 清除数据就绪中断标志位 ---
    // 读取中断状态寄存器可以清除中断标志，为下一次中断做准备
    uint8_t dummy;
    HAL_I2C_Mem_Read(LWKhIIC, MPU6050_ADDR, INT_STATUS_REG, 1, &dummy, 1, 10);
}

/**
 * @brief 卡尔曼滤波算法
 * @param Kalman 指向 Kalman_t 结构体的指针
 * @param newAngle 新的角度值
 * @param newRate 新的角速度值
 * @param dt 时间间隔
 * @retval 返回滤波后的角度值
 * @note 内部定义算法
 */
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