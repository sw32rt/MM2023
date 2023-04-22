#ifndef DEBUG_H_
#define DEBUG_H_

#include "stm32g4xx_hal.h"
#include "limits.h"

#define clockDiff(start, end) ((start < end) ? (end - start) : (end + (UINT32_MAX - start))) 
#define getClock() (DWT->CYCCNT)
#define stopWatchInit() {CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;}

#define getTimeUs(start, stop) ((clockDiff(start, stop)) / (SystemCoreClock / 1000000))

#endif /* DEBUG_H_ */