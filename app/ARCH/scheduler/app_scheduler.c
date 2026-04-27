#include "ARCH/scheduler/app_scheduler.h"

#include "AppTick.h"
#include "ARCH/config/app_config.h"

typedef struct
{
    AppScheduler_Callback callback;
    uint32_t period_ms;
    uint32_t next_run_ms;
    uint8_t used;
} AppScheduler_Node_t;

static AppScheduler_Node_t g_scheduler_nodes[APP_SCHEDULER_MAX_TASKS];

void AppScheduler_Init(void)
{
    uint8_t i;

    for (i = 0U; i < APP_SCHEDULER_MAX_TASKS; i++) {
        g_scheduler_nodes[i].callback    = 0;
        g_scheduler_nodes[i].period_ms   = 0U;
        g_scheduler_nodes[i].next_run_ms = 0U;
        g_scheduler_nodes[i].used        = 0U;
    }
}

uint8_t AppScheduler_Register(AppScheduler_Callback callback, uint16_t period_ms, uint16_t start_delay_ms)
{
    uint8_t i;
    uint32_t now_ms = AppTick_GetMs();

    if ((callback == 0) || (period_ms == 0U)) {
        return 0U;
    }

    for (i = 0U; i < APP_SCHEDULER_MAX_TASKS; i++) {
        if (!g_scheduler_nodes[i].used) {
            g_scheduler_nodes[i].callback    = callback;
            g_scheduler_nodes[i].period_ms   = period_ms;
            g_scheduler_nodes[i].next_run_ms = now_ms + start_delay_ms;
            g_scheduler_nodes[i].used        = 1U;
            return 1U;
        }
    }

    return 0U;
}

void AppScheduler_Run(void)
{
    uint8_t i;
    uint32_t now_ms = AppTick_GetMs();

    for (i = 0U; i < APP_SCHEDULER_MAX_TASKS; i++) {
        if (!g_scheduler_nodes[i].used || (g_scheduler_nodes[i].callback == 0)) {
            continue;
        }

        if ((int32_t)(now_ms - g_scheduler_nodes[i].next_run_ms) >= 0) {
            g_scheduler_nodes[i].callback();
            do {
                g_scheduler_nodes[i].next_run_ms += g_scheduler_nodes[i].period_ms;
            } while ((int32_t)(now_ms - g_scheduler_nodes[i].next_run_ms) >= 0);
        }
    }
}
