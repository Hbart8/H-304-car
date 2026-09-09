#ifndef APP_ARCH_SCHEDULER_H
#define APP_ARCH_SCHEDULER_H

#include <stdint.h>

/**
 * @brief 调度器回调函数类型
 * @note 所有被注册的任务都必须是无参、无返回值的短周期函数。
 */
typedef void (*AppScheduler_Callback)(void);

/**
 * @brief 初始化调度器内部任务表
 */
void AppScheduler_Init(void);

/**
 * @brief 注册一个周期任务
 * @param callback 周期到达时执行的回调函数
 * @param period_ms 任务周期，单位 ms
 * @param start_delay_ms 首次执行前的延迟，单位 ms
 * @return 注册成功返回 1，任务表已满或参数无效返回 0
 */
uint8_t AppScheduler_Register(AppScheduler_Callback callback, uint16_t period_ms, uint16_t start_delay_ms);

/**
 * @brief 运行一次调度扫描
 * @note 主循环应尽可能高频调用该函数，由它判断哪些周期任务到期。
 */
void AppScheduler_Run(void);

#endif
