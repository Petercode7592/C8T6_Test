#ifndef __VL5311H_H
#define __VL5311H_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"    // 根据具体型号选择对应的 HAL 头文件
#include "VL53L1X_api.h"     // ST 提供的 VL53L1X API 接口头文件

/**
  * @brief 全局标志位，表示 VL53L1X 有新的测距数据可读。
  * 当传感器的中断引脚触发时（测距完成），应在 HAL_GPIO_EXTI_Callback 中将该标志位置1。
  * 主循环应定期检查该标志，若为1则调用 VL53L1X_ReadDistance() 获取距离数据。
  */
extern volatile uint8_t vl53l1x_data_ready;

/**
  * @brief VL53L1X 测距模式枚举（距离模式）。
  * 短距模式(Short)对环境光抗扰更好，但最大测距约1.3m。
  * 长距模式(Long)可测距最远约4m，但对环境光敏感一些（默认）。
  */
typedef enum {
    VL53L1X_DIST_SHORT = 1,  /**< 短距离模式 (距离较近，但抗干扰强) */
    VL53L1X_DIST_LONG  = 2   /**< 长距离模式 (距离较远，默认模式) */
} VL53L1X_DistanceMode;

/**
  * @brief 初始化 VL53L1X 传感器并进入连续测距模式。
  * @param hi2c 指向用于与 VL53L1X 通信的 I2C_HandleTypeDef 句柄
  * @param mode 初始测距模式（短距或长距），推荐使用 VL53L1X_DIST_LONG 长距离模式
  * @param timing_budget_ms 每次测距的时间预算(单位毫秒)。典型值15~500(ms)，较大值可提高精度但降低速率。
  * @return 返回VL53L1X错误状态：0表示成功，非0表示失败（可通过 VL53L1X_ERROR 枚举查看具体错误）。
  * @note 该函数会配置传感器的测距模式和测量时间，并设置中断引脚为低电平有效，然后启动连续测距。
  *       请确保在调用此初始化之前，VL53L1X 的 XSHUT 引脚已拉高，传感器完成启动。
  */
VL53L1X_ERROR VL53L1X_Init(I2C_HandleTypeDef *hi2c, VL53L1X_DistanceMode mode, uint16_t timing_budget_ms);

/**
  * @brief 读取当前测得的距离数据（毫米）。
  * @param pDistance 指向用于存储测量距离的变量的指针 (单位: mm)。
  * @return 返回VL53L1X错误状态：0表示成功读取，非0表示读取失败。
  * @note 该函数应在 vl53l1x_data_ready 标志置位后调用。调用后函数内部会清除传感器的测距中断，并自动清除标志位。
  *       若无需使用中断，也可轮询此函数读取距离，但仍需确保传感器有数据可读以避免读取旧数据。
  */
VL53L1X_ERROR VL53L1X_ReadDistance(uint16_t *pDistance);

/**
  * @brief 设置 VL53L1X 的测距参数（模式和时间设置），可在运行过程中调用修改测距配置。
  * @param mode 测距距离模式：VL53L1X_DIST_SHORT（短距离模式）或 VL53L1X_DIST_LONG（长距离模式）。
  * @param timing_budget_ms 测距时间预算，单位ms。可用值包括15,20,33,50,100,200,500等，时间越长测量越稳定。
  * @param inter_measurement_ms 连续测量间隔，单位ms。必须 >= timing_budget_ms。建议等于或略大于测距时间预算。
  * @return 返回VL53L1X错误状态：0表示成功，非0表示失败。
  * @note 若传感器正在连续测距，本函数内部会先停止测距，更新配置后再重新启动连续测距，使配置生效。
  *       调用此函数将重置数据准备标志（vl53l1x_data_ready），因此应避免在刚好有未读取数据时切换模式，以防丢失数据。
  */
VL53L1X_ERROR VL53L1X_SetMeasurementMode(VL53L1X_DistanceMode mode, uint16_t timing_budget_ms, uint32_t inter_measurement_ms);

#ifdef __cplusplus
}
#endif

#endif /* __VL5311H_H */
