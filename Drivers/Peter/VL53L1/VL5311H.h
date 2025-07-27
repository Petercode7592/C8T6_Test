/* vl53l1x.h - VL53L1X 激光测距传感器驱动头文件 (基于 STM32 HAL) */
#ifndef __VL53L1X_H
#define __VL53L1X_H

#include "stm32f1xx_hal.h"      // HAL 库的头文件 (根据具体芯片修改)
#include "VL53L1X_api.h"

/**
  * @brief VL53L1X 传感器默认I2C地址 (7位地址0x29左移1位得到0x52)
  */
#define VL53L1X_I2C_ADDR      (0x52)

/**
  * @brief 传感器初始化函数。初始化I2C句柄，等待传感器启动并配置默认参数。
  * @param hi2c I2C 硬件句柄 (例如 &hi2c1)
  * @retval 0表示成功，非0表示错误代码
  * @note   默认配置下传感器测量频率为10Hz，模式为长距离模式:contentReference[oaicite:2]{index=2}。
  */
uint8_t VL53L1X_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief 读取距离的函数。可在轮询模式下调用，也可在中断触发时调用。
  * @retval 当前测得的距离，单位: 毫米(mm)
  * @note   如果使用轮询，此函数内部将阻塞等待测量数据准备好；如果使用中断，应确保数据就绪后再调用。
  */
uint16_t VL53L1X_ReadDistance(void);

/**
  * @brief 配置测量模式的函数。例如设置短距/长距模式，测量时序等。
  * @param distanceMode 距离模式: 1 表示短距模式，2 表示长距模式:contentReference[oaicite:3]{index=3}
  * @param timingBudgetMs 测量Timing Budget（单次测量的最大持续时间，单位ms）
  * @param interMeasurementMs 两次测量之间的间隔(Inter-Measurement Period，单位ms)
  * @retval 0表示配置成功，非0表示错误代码
  * @note   短距模式抗环境光干扰更强，但最大测量距离约1.3m；长距模式最远可测约4m（在暗环境下，TimingBudget需200ms):contentReference[oaicite:4]{index=4}。
  *         TimingBudget可用值一般为15,20,33,50,100,200,500(ms)，其中15ms仅支持短距模式:contentReference[oaicite:5]{index=5}。测量间隔必须大于或等于TimingBudget:contentReference[oaicite:6]{index=6}。
  */
uint8_t VL53L1X_SetMeasurementMode(uint8_t distanceMode, uint32_t timingBudgetMs, uint32_t interMeasurementMs);

#endif // __VL53L1X_H
