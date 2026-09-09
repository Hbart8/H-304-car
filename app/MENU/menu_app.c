#include "menu_app.h"

#include <string.h>

#include "OLED.h"
#include "menu_port.h"
#include "ARCH/device/device_center.h"
#include "ARCH/interaction/interaction_service.h"

MenuManager g_Menu;

MenuItem main_menu_1;
static MenuItem main_menu_2;
static MenuItem main_menu_3;
static MenuItem main_menu_4;
static MenuItem main_menu_5;

static MenuItem sub_1_1;
static MenuItem sub_1_2;
static MenuItem sub_1_3;
static MenuItem sub_1_4;
static MenuItem sub_1_5;
static MenuItem sub_1_6;
static MenuItem sub_1_7;
static MenuItem sub_1_8;
static MenuItem sub_1_9;
static MenuItem sub_1_10;
static MenuItem sub_1_11;

static MenuItem sub_2_1;
static MenuItem sub_2_2;
static MenuItem sub_2_3;
static MenuItem sub_2_4;
static MenuItem sub_2_5;

static MenuItem sub_4_1;
static MenuItem sub_4_2;
static MenuItem sub_4_3;
static MenuItem sub_4_4;

typedef struct {
    uint8_t active;
    InteractionPage_t page;
} MenuApp_PageState_t;

static MenuApp_PageState_t g_page_state = {0};

static void page(const char *a, const char *b, const char *c, const char *d) {
    OLED_Clear();
    if (a != 0) {
        OLED_ShowString(0, 0, (char *)a, 16);
    }
    if (b != 0) {
        OLED_ShowString(0, 2, (char *)b, 16);
    }
    if (c != 0) {
        OLED_ShowString(0, 4, (char *)c, 16);
    }
    if (d != 0) {
        OLED_ShowString(0, 6, (char *)d, 16);
    }
    OLED_Update();
}

static void MenuApp_RenderInteractionPage(void)
{
    page(g_page_state.page.lines[0],
         g_page_state.page.lines[1],
         g_page_state.page.lines[2],
         g_page_state.page.lines[3]);
}

static void MenuApp_OpenAction(InteractionAction_e action)
{
    memset(&g_page_state.page, 0, sizeof(g_page_state.page));
    MenuPort_SuppressUntilRelease();
    InteractionService_RequestAction(action);
    if (InteractionService_FetchPage(&g_page_state.page, 1U)) {
        g_page_state.active = 1U;
        MenuApp_RenderInteractionPage();
    }
}

static void act_run_a(void) { MenuApp_OpenAction(INTERACTION_ACTION_RUN_MODE_A); }
static void act_run_b(void) { MenuApp_OpenAction(INTERACTION_ACTION_RUN_MODE_B); }
static void act_straight(void) { MenuApp_OpenAction(INTERACTION_ACTION_STRAIGHT_TEST); }
static void act_arc(void) { MenuApp_OpenAction(INTERACTION_ACTION_ARC_TEST); }
static void act_go1000(void) { MenuApp_OpenAction(INTERACTION_ACTION_GO_1000MM); }
static void act_route250(void) { MenuApp_OpenAction(INTERACTION_ACTION_ROUTE_2500_L90_500_R90_500); }
static void act_q2l(void) { MenuApp_OpenAction(INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT); }
static void act_q2r(void) { MenuApp_OpenAction(INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT); }
static void act_q3(void) { MenuApp_OpenAction(INTERACTION_ACTION_Q3_1200_R270_500_R420_1800); }
/* 第四题入口，直接打开新加的直线+转弯路线页。 */
static void act_q4(void) { MenuApp_OpenAction(INTERACTION_ACTION_Q4); }
static void act_m1(void) { MenuApp_OpenAction(INTERACTION_ACTION_MOTOR_TEST_1); }
static void act_m2(void) { MenuApp_OpenAction(INTERACTION_ACTION_MOTOR_TEST_2); }
static void act_tl90(void) { MenuApp_OpenAction(INTERACTION_ACTION_TURN_LEFT_90); }
static void act_tr90(void) { MenuApp_OpenAction(INTERACTION_ACTION_TURN_RIGHT_90); }
static void act_sns(void) { MenuApp_OpenAction(INTERACTION_ACTION_SENSOR_PAGE); }
static void act_set(void) { MenuApp_OpenAction(INTERACTION_ACTION_SETTINGS_PAGE); }
static void act_arc_set(void) { MenuApp_OpenAction(INTERACTION_ACTION_ARC_SETTINGS_PAGE); }
static void act_abt(void) { MenuApp_OpenAction(INTERACTION_ACTION_ABOUT_PAGE); }
static void act_bak(void) { Menu_Back(&g_Menu); }

void MenuApp_Init(void) {
    main_menu_1 = (MenuItem){"1. System Run",  NULL, &sub_1_1, &main_menu_2, NULL,         NULL};
    main_menu_2 = (MenuItem){"2. Motor Test",  NULL, &sub_2_1, &main_menu_3, &main_menu_1, NULL};
    main_menu_3 = (MenuItem){"3. Sensor Data", NULL, NULL,     &main_menu_4, &main_menu_2, act_sns};
    main_menu_4 = (MenuItem){"4. Settings",    NULL, &sub_4_1, &main_menu_5, &main_menu_3, NULL};
    main_menu_5 = (MenuItem){"5. About",       NULL, NULL,     NULL,         &main_menu_4, act_abt};

    sub_1_1 = (MenuItem){"  Run Mode A",   &main_menu_1, NULL, &sub_1_2, NULL,     act_run_a};
    sub_1_2 = (MenuItem){"  Run Mode B",   &main_menu_1, NULL, &sub_1_3, &sub_1_1, act_run_b};
    sub_1_3 = (MenuItem){"  Straight Test",&main_menu_1, NULL, &sub_1_4, &sub_1_2, act_straight};
    sub_1_4 = (MenuItem){"  Arc Test",     &main_menu_1, NULL, &sub_1_5, &sub_1_3, act_arc};
    sub_1_5 = (MenuItem){"  Go 1000mm",    &main_menu_1, NULL, &sub_1_6, &sub_1_4, act_go1000};
    sub_1_6 = (MenuItem){"  Route 250LR",  &main_menu_1, NULL, &sub_1_7, &sub_1_5, act_route250};
    sub_1_7 = (MenuItem){"  Q2 L-R-L",   &main_menu_1, NULL, &sub_1_8,  &sub_1_6, act_q2l};
    sub_1_8 = (MenuItem){"  Q2 R-L-R",   &main_menu_1, NULL, &sub_1_9,  &sub_1_7, act_q2r};
    sub_1_9 = (MenuItem){"  Q3 Arc R100",&main_menu_1, NULL, &sub_1_10, &sub_1_8, act_q3};
    sub_1_10 = (MenuItem){"  Q4 Route",   &main_menu_1, NULL, &sub_1_11, &sub_1_9, act_q4};
    sub_1_11 = (MenuItem){"  Back",       &main_menu_1, NULL, NULL,      &sub_1_10, act_bak};

    sub_2_1 = (MenuItem){"  M1 Speed CL",&main_menu_2, NULL, &sub_2_2, NULL,     act_m1};
    sub_2_2 = (MenuItem){"  M2 Spin OL", &main_menu_2, NULL, &sub_2_3, &sub_2_1, act_m2};
    sub_2_3 = (MenuItem){"  Turn L90",   &main_menu_2, NULL, &sub_2_4, &sub_2_2, act_tl90};
    sub_2_4 = (MenuItem){"  Turn R90",   &main_menu_2, NULL, &sub_2_5, &sub_2_3, act_tr90};
    sub_2_5 = (MenuItem){"  Back",       &main_menu_2, NULL, NULL,     &sub_2_4, act_bak};

    sub_4_1 = (MenuItem){"  Q1 Tuning",  &main_menu_4, NULL, &sub_4_2, NULL,     act_set};
    sub_4_2 = (MenuItem){"  Arc Tuning", &main_menu_4, NULL, &sub_4_3, &sub_4_1, act_arc_set};
    sub_4_3 = (MenuItem){"  Common",     &main_menu_4, NULL, &sub_4_4, &sub_4_2, NULL};
    sub_4_4 = (MenuItem){"  Back",       &main_menu_4, NULL, NULL,     &sub_4_3, act_bak};
}

void MenuApp_Tick(void)
{
    MenuKey_e key;

    if (!g_page_state.active) {
        return;
    }

    if (InteractionService_FetchPage(&g_page_state.page, 0U)) {
        MenuApp_RenderInteractionPage();
    }

    key = MenuPort_GetKey();
    if ((InteractionService_GetCurrentAction() == INTERACTION_ACTION_SETTINGS_PAGE) ||
        (InteractionService_GetCurrentAction() == INTERACTION_ACTION_ARC_SETTINGS_PAGE)) {
        if (key == KEY_UP) {
            InteractionService_SettingsAdjust(1);
        } else if (key == KEY_DOWN) {
            InteractionService_SettingsAdjust(-1);
        } else if (key == KEY_ENTER) {
            InteractionService_SettingsNext();
        }

        if ((key == KEY_UP) || (key == KEY_DOWN) || (key == KEY_ENTER)) {
            if (InteractionService_FetchPage(&g_page_state.page, 1U)) {
                MenuApp_RenderInteractionPage();
            }
            return;
        }
    } else if ((key == KEY_UP) &&
        (InteractionService_GetCurrentAction() == INTERACTION_ACTION_SENSOR_PAGE)) {
        DeviceCenter_ResetOdometry();
        DeviceCenter_PlayConfirm();
        if (InteractionService_FetchPage(&g_page_state.page, 1U)) {
            MenuApp_RenderInteractionPage();
        }
        return;
    }

    if (key == KEY_ENTER || key == KEY_BACK) {
        g_page_state.active = 0U;
        InteractionService_ClosePage();
        MenuPort_SuppressUntilRelease();
        MenuPort_Render(&g_Menu);
    }
}

uint8_t MenuApp_IsPageActive(void)
{
    return g_page_state.active;
}
