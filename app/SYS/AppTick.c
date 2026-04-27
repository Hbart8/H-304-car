#include "AppTick.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t g_app_tick_ms = 0U;

void AppTick_Init(void)
{
    SysTick_Config(CPUCLK_FREQ / 1000U);
}

void AppTick_IrqHandler(void)
{
    g_app_tick_ms++;
}

uint32_t AppTick_GetMs(void)
{
    return g_app_tick_ms;
}

uint8_t AppTick_IsExpired(uint32_t start_ms, uint32_t duration_ms)
{
    return ((uint32_t)(AppTick_GetMs() - start_ms) >= duration_ms) ? 1U : 0U;
}
