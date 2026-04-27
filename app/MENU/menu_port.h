#ifndef _MENU_PORT_H_
#define _MENU_PORT_H_

#include <stdint.h>
#include "menu.h"

typedef enum {
    KEY_NONE = 0,
    KEY_UP,
    KEY_DOWN,
    KEY_ENTER,
    KEY_BACK
} MenuKey_e;

void MenuPort_Init(MenuManager *nav, MenuItem *root);
void MenuPort_Tick(MenuManager *nav);
void MenuPort_DebugTick(void);
void MenuPort_DebugMenuTick(void);
void MenuPort_Render(MenuManager *nav);
void MenuPort_ResetKeyState(void);
void MenuPort_SuppressUntilRelease(void);

void OLED_DrawMenuString(uint8_t line, uint8_t cursor_flag, const char *str);
void OLED_ClearScreen(void);
MenuKey_e MenuPort_GetKey(void);

#endif
