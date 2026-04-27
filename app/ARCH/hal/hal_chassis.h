#ifndef APP_ARCH_HAL_CHASSIS_H
#define APP_ARCH_HAL_CHASSIS_H

#include <stdint.h>

#include "ARCH/config/board_profile.h"

typedef struct
{
    int16_t left_duty;
    int16_t right_duty;
    int16_t wheel_duty[APP_BOARD_WHEEL_COUNT];
    uint8_t pwm_online;
} HalChassis_MotorState_t;

typedef struct
{
    int32_t wheel_ticks[APP_BOARD_WHEEL_COUNT];
    uint32_t raw_edges[APP_BOARD_ENCODER_RAW_COUNT];
    uint8_t raw_level[APP_BOARD_ENCODER_RAW_COUNT];
} HalChassis_EncoderState_t;

void HalChassis_Init(void);
void HalChassis_Service1ms(void);
void HalChassis_Tick10ms(void);
void HalChassis_SetMotorDuty(int16_t left_duty, int16_t right_duty);
void HalChassis_SetWheelDuty(const int16_t *wheel_duty);
void HalChassis_GetMotorState(HalChassis_MotorState_t *state);
void HalChassis_GetEncoderState(HalChassis_EncoderState_t *state);
void HalChassis_SetRgb(uint8_t red, uint8_t green, uint8_t blue);

#endif
