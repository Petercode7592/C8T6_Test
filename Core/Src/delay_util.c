#include "delay_util.h"
#include "stm32f1xx_hal.h"

void Delay_Start(DelayTimer_t *timer)
{
    timer->start = HAL_GetTick();
}

uint8_t Delay_Expired(DelayTimer_t *timer, uint32_t duration_ms)
{
    if (HAL_GetTick()-timer->start >= duration_ms) {
        return 1;
    }
    return 0;
}
