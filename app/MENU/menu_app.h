#ifndef _MENU_APP_H_
#define _MENU_APP_H_

#include "menu.h"
#include <stdint.h>

extern MenuManager g_Menu;
extern MenuItem main_menu_1;

void MenuApp_Init(void);
void MenuApp_Tick(void);
uint8_t MenuApp_IsPageActive(void);

#endif
