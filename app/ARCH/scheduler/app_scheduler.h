#ifndef APP_ARCH_SCHEDULER_H
#define APP_ARCH_SCHEDULER_H

#include <stdint.h>

typedef void (*AppScheduler_Callback)(void);

void AppScheduler_Init(void);
uint8_t AppScheduler_Register(AppScheduler_Callback callback, uint16_t period_ms, uint16_t start_delay_ms);
void AppScheduler_Run(void);

#endif
