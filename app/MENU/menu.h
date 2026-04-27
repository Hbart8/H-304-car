 #ifndef _MENU_H_
#define _MENU_H_

#include <stddef.h> 
#include <stdint.h>

typedef void (*MenuCallback)(void);

typedef struct MenuItem {
    const char *name;
    struct MenuItem *parent;
    struct MenuItem *child;
    struct MenuItem *next;
    struct MenuItem *prev;
    MenuCallback callback;
} MenuItem;

typedef struct {
    MenuItem *current_menu;
    MenuItem *cursor_item;
    uint8_t cursor_index;
    uint8_t show_start_idx;
    uint8_t max_show_items;
} MenuManager;

void Menu_Init(MenuManager *nav, MenuItem *root_menu, uint8_t max_items_on_screen);
void Menu_Up(MenuManager *nav);
void Menu_Down(MenuManager *nav);
void Menu_Enter(MenuManager *nav);
void Menu_Back(MenuManager *nav);

#endif
