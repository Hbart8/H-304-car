#ifndef APP_TICK_H
#define APP_TICK_H

#include <stdint.h>

void AppTick_Init(void);
void AppTick_IrqHandler(void);
uint32_t AppTick_GetMs(void);
uint8_t AppTick_IsExpired(uint32_t start_ms, uint32_t duration_ms);

#endif
