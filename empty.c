#include "ti_msp_dl_config.h"
#include "AppTick.h"
#include "ARCH/app_system.h"
#include "ARCH/config/app_config.h"
#include "Buzzer.h"
#include "OLED.h"
#include "MENU/menu_app.h"
#include "MENU/menu_port.h"

void SysTick_Handler(void)
{
    AppTick_IrqHandler();
    Buzzer_Tick1ms();
}

int main(void)                                                                                       
{
    SYSCFG_DL_init();
    AppTick_Init();
    AppSystem_Init();
    OLED_Init();
#if APP_DEBUG_KEY_MONITOR == 0U
    MenuApp_Init();
    MenuPort_Init(&g_Menu, &main_menu_1); 
#else
    MenuPort_ResetKeyState();
#endif

    while (1) {
        AppSystem_Run();

#if APP_DEBUG_KEY_MONITOR == 1U
        MenuPort_DebugTick();
#elif APP_DEBUG_KEY_MONITOR == 2U
        MenuPort_DebugMenuTick();
#else
        if (MenuApp_IsPageActive()) {
            MenuApp_Tick();
        } else {
            MenuPort_Tick(&g_Menu);
        }
#endif 

        __WFI();
    }
}
