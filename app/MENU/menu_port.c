#include "menu_port.h"

#include <stdio.h>

#include "OLED.h"
#include "menu_app.h"
#include "AppTick.h"
#include "ARCH/device/device_center.h"
#include "ti_msp_dl_config.h"

static uint8_t k_dn(GPIO_Regs *port, uint32_t pin) {
    return (DL_GPIO_readPins(port, pin) == 0U);
}

typedef struct
{
    MenuKey_e last_raw;
    MenuKey_e debounced;
    uint32_t change_ms;
    uint8_t press_latched;
    uint8_t wait_release;
} MenuPort_KeyState_t;

static MenuPort_KeyState_t g_key_state = {KEY_NONE, KEY_NONE, 0U, 0U, 0U};

static char MenuPort_KeyCode(MenuKey_e key)
{
    switch (key) {
    case KEY_UP:
        return 'U';
    case KEY_DOWN:
        return 'D';
    case KEY_ENTER:
        return 'E';
    case KEY_BACK:
        return 'B';
    case KEY_NONE:
    default:
        return '-';
    }
}

static uint8_t MenuPort_ReadRawKey(MenuKey_e *key)
{
    uint8_t key1_pressed = k_dn(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN);
    uint8_t key2_pressed = k_dn(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN);
    uint8_t key3_pressed = k_dn(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN);
    uint8_t key4_pressed = k_dn(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN);
    uint8_t pressed_count = (uint8_t)(key1_pressed + key2_pressed + key3_pressed + key4_pressed);

    if (key == 0) {
        return pressed_count;
    }

    if (pressed_count != 1U) {
        *key = KEY_NONE;
        return pressed_count;
    }

    if (key1_pressed != 0U) {
        *key = KEY_UP;
    } else if (key2_pressed != 0U) {
        *key = KEY_DOWN;
    } else if (key3_pressed != 0U) {
        *key = KEY_ENTER;
    } else {
        *key = KEY_BACK;
    }

    return pressed_count;
}

static MenuKey_e k_raw(void) {
    MenuKey_e key = KEY_NONE;

    MenuPort_ReadRawKey(&key);
    return key;
}

void MenuPort_DebugTick(void)
{
    static uint32_t last_render_ms = 0U;
    static uint32_t last_state = 0xFFFFFFFFUL;
    uint8_t level1;
    uint8_t level2;
    uint8_t level3;
    uint8_t level4;
    uint8_t press1;
    uint8_t press2;
    uint8_t press3;
    uint8_t press4;
    uint8_t pressed_count;
    MenuKey_e key = KEY_NONE;
    uint32_t state;
    char line1[17];
    char line2[17];
    char line3[17];
    char line4[17];

    level1 = (DL_GPIO_readPins(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN) != 0U) ? 1U : 0U;
    level2 = (DL_GPIO_readPins(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN) != 0U) ? 1U : 0U;
    level3 = (DL_GPIO_readPins(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN) != 0U) ? 1U : 0U;
    level4 = (DL_GPIO_readPins(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN) != 0U) ? 1U : 0U;
    press1 = k_dn(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN);
    press2 = k_dn(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN);
    press3 = k_dn(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN);
    press4 = k_dn(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN);
    pressed_count = MenuPort_ReadRawKey(&key);

    state = ((uint32_t)level1 << 0U) |
            ((uint32_t)level2 << 1U) |
            ((uint32_t)level3 << 2U) |
            ((uint32_t)level4 << 3U) |
            ((uint32_t)press1 << 4U) |
            ((uint32_t)press2 << 5U) |
            ((uint32_t)press3 << 6U) |
            ((uint32_t)press4 << 7U) |
            ((uint32_t)pressed_count << 8U) |
            ((uint32_t)key << 12U);

    if ((state == last_state) && !AppTick_IsExpired(last_render_ms, 100U)) {
        return;
    }

    last_state = state;
    last_render_ms = AppTick_GetMs();

    snprintf(line1, sizeof(line1), "KEY MONITOR");
    snprintf(line2, sizeof(line2), "LEV:%u%u%u%u",
        (unsigned)level1, (unsigned)level2, (unsigned)level3, (unsigned)level4);
    snprintf(line3, sizeof(line3), "PRS:%u%u%u%u",
        (unsigned)press1, (unsigned)press2, (unsigned)press3, (unsigned)press4);
    snprintf(line4, sizeof(line4), "KEY:%c CNT:%u",
        MenuPort_KeyCode(key), (unsigned)pressed_count);

    OLED_Clear();
    OLED_ShowString(0, 0, line1, 16);
    OLED_ShowString(0, 2, line2, 16);
    OLED_ShowString(0, 4, line3, 16);
    OLED_ShowString(0, 6, line4, 16);
    OLED_Update();
}

void MenuPort_DebugMenuTick(void)
{
    static uint32_t last_render_ms = 0U;
    static uint32_t event_count = 0U;
    static uint32_t last_state = 0xFFFFFFFFUL;
    static uint8_t cursor = 0U;
    static MenuKey_e last_key = KEY_NONE;
    uint8_t level1;
    uint8_t level2;
    uint8_t level3;
    uint8_t level4;
    uint8_t pressed_count;
    MenuKey_e raw_key = KEY_NONE;
    MenuKey_e key;
    uint32_t state;
    char line1[17];
    char line2[17];
    char line3[17];
    char line4[17];

    key = MenuPort_GetKey();
    if (key != KEY_NONE) {
        last_key = key;
        event_count++;

        if (key == KEY_UP) {
            cursor = (cursor == 0U) ? 4U : (uint8_t)(cursor - 1U);
        } else if (key == KEY_DOWN) {
            cursor = (cursor >= 4U) ? 0U : (uint8_t)(cursor + 1U);
        } else if (key == KEY_ENTER) {
            cursor = 0U;
        } else if (key == KEY_BACK) {
            cursor = 4U;
        }
    }

    level1 = (DL_GPIO_readPins(GPIO_KEY_KEY1_PORT, GPIO_KEY_KEY1_PIN) != 0U) ? 1U : 0U;
    level2 = (DL_GPIO_readPins(GPIO_KEY_KEY2_PORT, GPIO_KEY_KEY2_PIN) != 0U) ? 1U : 0U;
    level3 = (DL_GPIO_readPins(GPIO_KEY_KEY3_PORT, GPIO_KEY_KEY3_PIN) != 0U) ? 1U : 0U;
    level4 = (DL_GPIO_readPins(GPIO_KEY_KEY4_PORT, GPIO_KEY_KEY4_PIN) != 0U) ? 1U : 0U;
    pressed_count = MenuPort_ReadRawKey(&raw_key);

    state = ((uint32_t)cursor << 0U) |
            ((uint32_t)last_key << 4U) |
            ((uint32_t)level1 << 8U) |
            ((uint32_t)level2 << 9U) |
            ((uint32_t)level3 << 10U) |
            ((uint32_t)level4 << 11U) |
            ((uint32_t)pressed_count << 12U) |
            ((event_count & 0xFFFFUL) << 16U);

    if ((state == last_state) && !AppTick_IsExpired(last_render_ms, 100U)) {
        return;
    }

    last_state = state;
    last_render_ms = AppTick_GetMs();

    snprintf(line1, sizeof(line1), "MENU PROBE");
    snprintf(line2, sizeof(line2), "CUR:%u EVT:%u",
        (unsigned)cursor, (unsigned)(event_count & 0xFFFFUL));
    snprintf(line3, sizeof(line3), "KEY:%c RAW:%c",
        MenuPort_KeyCode(last_key), MenuPort_KeyCode(raw_key));
    snprintf(line4, sizeof(line4), "LEV:%u%u%u%u C:%u",
        (unsigned)level1, (unsigned)level2, (unsigned)level3, (unsigned)level4,
        (unsigned)pressed_count);

    OLED_Clear();
    OLED_ShowString(0, 0, line1, 16);
    OLED_ShowString(0, 2, line2, 16);
    OLED_ShowString(0, 4, line3, 16);
    OLED_ShowString(0, 6, line4, 16);
    OLED_Update();
}

void MenuPort_ResetKeyState(void)
{
    g_key_state.last_raw = KEY_NONE;
    g_key_state.debounced = KEY_NONE;
    g_key_state.change_ms = AppTick_GetMs();
    g_key_state.press_latched = 0U;
    g_key_state.wait_release = 0U;
}

void MenuPort_SuppressUntilRelease(void)
{
    g_key_state.last_raw = KEY_NONE;
    g_key_state.debounced = KEY_NONE;
    g_key_state.change_ms = AppTick_GetMs();
    g_key_state.press_latched = 0U;
    g_key_state.wait_release = 1U;
}

void OLED_DrawMenuString(uint8_t line, uint8_t cursor_flag, const char *str) {
    char buf[17] = "                ";
    uint8_t i = 0U;
    uint8_t dst = 0U;

    while ((str[i] != '\0') && (dst < 15U)) {
        buf[dst++] = str[i];
        i++;
    }
    buf[16] = '\0';

    if (cursor_flag != 0U) {
        OLED_ShowString_Reverse(0, line * 2, buf, 16);
    } else {
        OLED_ShowString(0, line * 2, buf, 16);
    }
}

void OLED_ClearScreen(void) {
    OLED_Clear();
}

MenuKey_e MenuPort_GetKey(void) {
    MenuKey_e raw = k_raw();
    uint8_t pressed_count = MenuPort_ReadRawKey(0);
    uint32_t now_ms = AppTick_GetMs();

    if (g_key_state.wait_release != 0U) {
        if (pressed_count != 0U) {
            g_key_state.change_ms = now_ms;
            return KEY_NONE;
        }

        if (!AppTick_IsExpired(g_key_state.change_ms, 20U)) {
            return KEY_NONE;
        }

        g_key_state.wait_release = 0U;
        g_key_state.last_raw = KEY_NONE;
        g_key_state.debounced = KEY_NONE;
        g_key_state.press_latched = 0U;
    }

    if (pressed_count > 1U) {
        g_key_state.last_raw = KEY_NONE;
        g_key_state.debounced = KEY_NONE;
        g_key_state.change_ms = now_ms;
        g_key_state.press_latched = 0U;
        return KEY_NONE;
    }

    if (raw != g_key_state.last_raw) {
        g_key_state.last_raw = raw;
        g_key_state.change_ms = now_ms;
    }

    if (!AppTick_IsExpired(g_key_state.change_ms, 20U)) {
        return KEY_NONE;
    }

    if (raw != g_key_state.debounced) {
        g_key_state.debounced = raw;
        if (raw == KEY_NONE) {
            g_key_state.press_latched = 0U;
        }
    }

    if ((g_key_state.debounced != KEY_NONE) && !g_key_state.press_latched) {
        g_key_state.press_latched = 1U;
        g_key_state.wait_release = 1U;
        g_key_state.change_ms = now_ms;
        return g_key_state.debounced;
    }

    return KEY_NONE;
}

void MenuPort_Init(MenuManager *nav, MenuItem *root) {
    MenuPort_ResetKeyState();
    Menu_Init(nav, root, 4);
    MenuPort_Render(nav);
}

void MenuPort_Render(MenuManager *nav) {
    MenuItem *temp;
    uint8_t i;

    if (nav == NULL || nav->current_menu == NULL) {
        return;
    }

    OLED_ClearScreen();

    temp = nav->current_menu;
    for (i = 0; i < nav->show_start_idx; i++) {
        if (temp != NULL) {
            temp = temp->next;
        }
    }

    for (i = 0; i < nav->max_show_items; i++) {
        if (temp == NULL) {
            break;
        }

        OLED_DrawMenuString(i, (uint8_t)(i == nav->cursor_index), temp->name);
        temp = temp->next;
    }

    OLED_Update();
}

void MenuPort_Tick(MenuManager *nav) {
    MenuKey_e key = MenuPort_GetKey();

    if (key != KEY_NONE) {
        switch (key) {
        case KEY_UP:
            Menu_Up(nav);
            break;
        case KEY_DOWN:
            Menu_Down(nav);
            break;
        case KEY_ENTER:
            Menu_Enter(nav);
            break;
        case KEY_BACK:
            Menu_Back(nav);
            break;
        default:
            break;
        }

        if (!MenuApp_IsPageActive()) {
            MenuPort_Render(nav);
        }
    }
}
