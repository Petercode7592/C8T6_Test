#ifndef __DELAY_UTIL_H
#define __DELAY_UTIL_H

#include "stdint.h"

typedef struct {
    uint32_t start;
} DelayTimer_t;

void Delay_Start(DelayTimer_t *timer);
uint8_t Delay_Expired(DelayTimer_t *timer, uint32_t duration_ms);

#endif
