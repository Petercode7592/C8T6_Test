/* vl53l1x.c - VL53L1X 激光测距传感器驱动源文件 */
#include "VL5311H.h"

/* 内部静态变量保存 I2C 接口句柄和传感器状态 */
static I2C_HandleTypeDef *vl53l1x_hi2c = NULL;
static uint8_t vl53l1x_running = 0;                     // 标记传感器是否在连续测量中
static const uint16_t vl53l1x_dev = VL53L1X_I2C_ADDR;   // 设备I2C地址 (dev参数)

/**
  * @brief  传感器初始化实现
  */
uint8_t VL53L1X_Init(I2C_HandleTypeDef *hi2c) {
    vl53l1x_hi2c = hi2c;  // 保存I2C句柄供后续低层I2C读写使用

    /* 1. 等待传感器上电后完成Boot */
    uint8_t bootState = 0;
    uint32_t timeout = 0;
    while (bootState == 0 && timeout < 500) {
        VL53L1X_BootState(vl53l1x_dev, &bootState);
        HAL_Delay(2);
        timeout += 2;
    }
    // 检查传感器是否启动成功
    if (bootState == 0) {
        return 1;  // 超时错误: 传感器未在预期时间内启动
    }

    /* 2. 调用VL53L1X库的初始化，将传感器配置为默认模式 */
    VL53L1X_SensorInit(vl53l1x_dev);
    // 默认配置：10Hz测量频率，长距离模式，Timing Budget=100ms:contentReference[oaicite:8]{index=8}:contentReference[oaicite:9]{index=9}

    /* 3. 启动连续测距 */
    VL53L1X_StartRanging(vl53l1x_dev);  // 开始连续测距（传感器将按默认10Hz输出距离）
    vl53l1x_running = 1;               // 标记传感器测距已启动

    return 0; // 初始化成功
}

/**
  * @brief  读取当前测量距离
  */
uint16_t VL53L1X_ReadDistance(void) {
    uint8_t dataReady = 0;
    uint16_t distance = 0;

    /* 若采用轮询方式，则等待数据准备好 */
    if (!vl53l1x_running) {
        // 如未开始测距则无法读取，直接返回0
        return 0;
    }
    while (dataReady == 0) {
        VL53L1X_CheckForDataReady(vl53l1x_dev, &dataReady);
        HAL_Delay(5);  // 小延时避免占用过多CPU
    }

    /* 数据准备好后，读取距离数据 */
    VL53L1X_GetDistance(vl53l1x_dev, &distance);
    /* 清除中断标志，准备接收下一次测量数据:contentReference[oaicite:10]{index=10} */
    VL53L1X_ClearInterrupt(vl53l1x_dev);

    return distance;  // 返回测得的距离（毫米）
}

/**
  * @brief  配置测量模式（距离模式/TimingBudget/测量间隔）
  */
uint8_t VL53L1X_SetMeasurementMode(uint8_t distanceMode, uint32_t timingBudgetMs, uint32_t interMeasurementMs) {
    VL53L1X_ERROR status = 0;

    /* 若当前正在测量，则先停止测距，以便安全地更新配置 */
    if (vl53l1x_running) {
        VL53L1X_StopRanging(vl53l1x_dev);
        vl53l1x_running = 0;
    }

    /* 1. 设置距离模式: 1=短距离模式, 2=长距离模式 */
    status |= VL53L1X_SetDistanceMode(vl53l1x_dev, distanceMode);
    /* 确保Timing Budget满足模式要求: 短距离模式允许最短15ms，长距离模式最短33ms:contentReference[oaicite:11]{index=11} */
    if (distanceMode == 1 && timingBudgetMs < 15) {
        timingBudgetMs = 15;
    } else if (distanceMode == 2 && timingBudgetMs < 33) {
        timingBudgetMs = 33;
    }

    /* 2. 设置测量Timing Budget（单次测量周期） */
    status |= VL53L1X_SetTimingBudgetInMs(vl53l1x_dev, timingBudgetMs);
    /* 3. 设置测量间隔（两次测量启动之间的间隔），需>= Timing Budget:contentReference[oaicite:12]{index=12} */
    if (interMeasurementMs < timingBudgetMs) {
        interMeasurementMs = timingBudgetMs;
    }
    status |= VL53L1X_SetInterMeasurementInMs(vl53l1x_dev, interMeasurementMs);

    /* 4. 重新启动连续测距 */
    status |= VL53L1X_StartRanging(vl53l1x_dev);
    vl53l1x_running = 1;

    return (uint8_t)status;
}
