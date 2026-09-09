#ifndef APP_ARCH_APP_SYSTEM_H
#define APP_ARCH_APP_SYSTEM_H

/**
 * @brief 应用系统入口模块
 * @note 负责完成各业务模块初始化，并驱动主循环中的调度器运行。
 */
void AppSystem_Init(void);
void AppSystem_Run(void);

#endif
