// vl53l1_platform.c - 适用于 STM32F103 的 VL53L1 平台适配层

#include "vl53l1_platform.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include "vl53l1_error_codes.h"

extern I2C_HandleTypeDef hi2c1;  // 请确认这是你的 I2C 句柄

#define I2C_TIMEOUT_MS 100
static uint8_t _I2CBuffer[256];

VL53L1_Error VL53L1_WriteMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    if (HAL_I2C_Master_Transmit(&hi2c1, Dev, _I2CBuffer, count + 2, I2C_TIMEOUT_MS) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_ReadMulti(uint16_t Dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t reg[2] = { index >> 8, index & 0xFF };
    if (HAL_I2C_Master_Transmit(&hi2c1, Dev, reg, 2, I2C_TIMEOUT_MS) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;
    if (HAL_I2C_Master_Receive(&hi2c1, Dev, pdata, count, I2C_TIMEOUT_MS) != HAL_OK)
        return VL53L1_ERROR_CONTROL_INTERFACE;
    return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrByte(uint16_t Dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(Dev, index, &data, 1);
}

VL53L1_Error VL53L1_WrWord(uint16_t Dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L1_WriteMulti(Dev, index, buf, 2);
}

VL53L1_Error VL53L1_WrDWord(uint16_t Dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = {
        (uint8_t)(data >> 24), (uint8_t)(data >> 16),
        (uint8_t)(data >> 8), (uint8_t)data
    };
    return VL53L1_WriteMulti(Dev, index, buf, 4);
}

VL53L1_Error VL53L1_RdByte(uint16_t Dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(Dev, index, data, 1);
}

VL53L1_Error VL53L1_RdWord(uint16_t Dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 2);
    if (status == VL53L1_ERROR_NONE)
        *data = ((uint16_t)buf[0] << 8) | buf[1];
    return status;
}

VL53L1_Error VL53L1_RdDWord(uint16_t Dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    VL53L1_Error status = VL53L1_ReadMulti(Dev, index, buf, 4);
    if (status == VL53L1_ERROR_NONE)
        *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    return status;
}

VL53L1_Error VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    (void)dev;
    HAL_Delay(wait_ms);
    return VL53L1_ERROR_NONE;
}
