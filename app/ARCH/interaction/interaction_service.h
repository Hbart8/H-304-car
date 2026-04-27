#ifndef APP_ARCH_INTERACTION_SERVICE_H
#define APP_ARCH_INTERACTION_SERVICE_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

typedef enum
{
    INTERACTION_ACTION_NONE = 0,
    INTERACTION_ACTION_RUN_MODE_A,
    INTERACTION_ACTION_RUN_MODE_B,
    INTERACTION_ACTION_STRAIGHT_TEST,
    INTERACTION_ACTION_ARC_TEST,
    INTERACTION_ACTION_GO_1000MM,
    INTERACTION_ACTION_ROUTE_2500_L90_500_R90_500,
    INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT,
    INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT,
    INTERACTION_ACTION_Q3_1200_R270_500_R420_1800,
    INTERACTION_ACTION_MOTOR_TEST_1,
    INTERACTION_ACTION_MOTOR_TEST_2,
    INTERACTION_ACTION_TURN_LEFT_90,
    INTERACTION_ACTION_TURN_RIGHT_90,
    INTERACTION_ACTION_SENSOR_PAGE,
    INTERACTION_ACTION_SETTINGS_PAGE,
    INTERACTION_ACTION_ARC_SETTINGS_PAGE,
    INTERACTION_ACTION_ABOUT_PAGE,
} InteractionAction_e;

typedef struct
{
    uint8_t visible;
    char lines[4][APP_UI_LINE_LEN + 1U];
} InteractionPage_t;

void InteractionService_Init(void);
void InteractionService_RequestAction(InteractionAction_e action);
void InteractionService_Tick50ms(void);
void InteractionService_ClosePage(void);
uint8_t InteractionService_FetchPage(InteractionPage_t *page, uint8_t force_copy);
InteractionAction_e InteractionService_GetCurrentAction(void);
void InteractionService_SettingsAdjust(int8_t delta);
void InteractionService_SettingsNext(void);

#endif
