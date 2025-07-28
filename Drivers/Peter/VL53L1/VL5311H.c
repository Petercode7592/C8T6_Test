#include "VL5311H.h"

// 存储用于 VL53L1X 通信的 I2C 句柄指针
I2C_HandleTypeDef *vl53l1x_hi2c = NULL;

// VL53L1X 默认I2C地址（7位地址0x29, 在代码中使用8位地址0x52）
#define VL53L1X_I2C_ADDR (0x52) /**< VL53L1X 默认I2C地址 (上电后默认0x52) */

// 定义并初始化全局数据就绪标志位
volatile uint8_t vl53l1x_data_ready = 0;

VL53L1X_ERROR VL53L1X_Init(I2C_HandleTypeDef *hi2c, VL53L1X_DistanceMode mode, uint16_t timing_budget_ms) {
	VL53L1X_ERROR status = 0;
	uint8_t bootState = 0;
	uint16_t dev = VL53L1X_I2C_ADDR; // 传感器 I2C 设备地址 (使用 ST 驱动时作为 dev 参数)

	// 保存 I2C 句柄用于后续通信 (假定 ST API 的底层实现需要此全局句柄)
	vl53l1x_hi2c = hi2c;

	// 1. 等待传感器启动完成（引导完成）
	// 读取传感器启动状态，等待 BootState = 1
	for (int retry = 0; retry < 100; ++retry) {
		status = VL53L1X_BootState(dev, &bootState);
		if (status != 0) {
			// 通信错误则直接返回错误状态
			return status;
		}
		if (bootState == 1) {
			break; // 传感器已经启动
		}
		HAL_Delay(2); // 等待2ms后重试
	}

	if (bootState == 0) {
		// 若循环结束仍未启动，则返回错误
		return 1; // 返回非0错误码，表示传感器未就绪
	}

	// 2. 初始化传感器（加载默认配置，默认长距离模式10Hz）
	status = VL53L1X_SensorInit(dev);
	if (status != 0) {
		return status;
	}

	// 3. 设置测距模式和测量时间参数
	// 距离模式：短距 or 长距
	status = VL53L1X_SetDistanceMode(dev, (uint16_t) mode);
	if (status != 0) {
		return status;
	}
	// 设置测距时间预算（Timing Budget）
	status = VL53L1X_SetTimingBudgetInMs(dev, timing_budget_ms);
	if (status != 0) {
		return status;
	}
	// 设置连续测量的间隔时间（确保间隔 >= 时间预算）
	uint32_t inter_measurement = timing_budget_ms; // 此处将间隔设为等于时间预算，以实现最大速度
	if (inter_measurement < timing_budget_ms) {
		inter_measurement = timing_budget_ms;
	}
	status = VL53L1X_SetInterMeasurementInMs(dev, inter_measurement);
	if (status != 0) {
		return status;
	}

	// 4. 配置中断引脚极性为低电平有效（默认为高电平有效，需要改为0）
	status = VL53L1X_SetInterruptPolarity(dev, 0);
	if (status != 0) {
		return status;
	}

	// 5. 启动连续测距
	status = VL53L1X_StartRanging(dev);
	// 如果启动测距返回成功，传感器将开始连续测量，此后每完成一次测距测量，
	// VL53L1X的GPIO中断引脚将被拉低(低电平有效)，需要用户在中断回调中设置标志位。

	return status;
}

VL53L1X_ERROR VL53L1X_ReadDistance(uint16_t *pDistance) {
	uint16_t dev = VL53L1X_I2C_ADDR;
	VL53L1X_ERROR status;

	// 读取测量的距离数据（毫米）
	status = VL53L1X_GetDistance(dev, pDistance);
	if (status != 0) {
		return status; // 若读取失败，返回错误状态
	}

	// 清除传感器内部中断，以便传感器进入下一次测距并再次触发中断引脚
	status = VL53L1X_ClearInterrupt(dev);
	if (status != 0) {
		return status;
	}

	// 重置数据就绪标志，表示本次数据已处理
	vl53l1x_data_ready = 0;

	return 0; // 成功读取则返回0
}

VL53L1X_ERROR VL53L1X_SetMeasurementMode(VL53L1X_DistanceMode mode, uint16_t timing_budget_ms,
										 uint32_t inter_measurement_ms) {
	uint16_t dev = VL53L1X_I2C_ADDR;
	VL53L1X_ERROR status = 0;

	// 如果需要修改配置，建议先停止连续测距再修改
	status = VL53L1X_StopRanging(dev);
	if (status != 0) {
		// 若停止测距失败，可能传感器未在测距，或者出现错误，尝试继续配置
		// 我们这里不强制返回，让后续配置继续进行
		status = 0;
	}

	// 设置新的距离模式（1=短距，2=长距）
	status = VL53L1X_SetDistanceMode(dev, (uint16_t) mode);
	if (status != 0) {
		return status;
	}
	// 设置测距时间预算
	status = VL53L1X_SetTimingBudgetInMs(dev, timing_budget_ms);
	if (status != 0) {
		return status;
	}
	// 确保间隔时间不小于测距时间预算
	if (inter_measurement_ms < timing_budget_ms) {
		inter_measurement_ms = timing_budget_ms;
	}
	status = VL53L1X_SetInterMeasurementInMs(dev, inter_measurement_ms);
	if (status != 0) {
		return status;
	}

	// 清除之前可能未处理的中断标志，避免误触发
	vl53l1x_data_ready = 0;
	// （可选）读取并清除一次传感器中断，以防上一次测距的中断遗留
	VL53L1X_ClearInterrupt(dev);

	// 重新启动连续测距，使新配置生效
	status = VL53L1X_StartRanging(dev);
	return status;
}
