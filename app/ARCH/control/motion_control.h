#ifndef APP_ARCH_CONTROL_MOTION_CONTROL_H
#define APP_ARCH_CONTROL_MOTION_CONTROL_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

typedef struct
{
    int16_t left_duty;
    int16_t right_duty;
    int16_t wheel_speed_target_mmps[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_measure_mmps[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_output_duty[APP_BOARD_WHEEL_COUNT];
    int16_t speed_target_left_mmps;
    int16_t speed_target_right_mmps;
    int16_t speed_measure_left_mmps;
    int16_t speed_measure_right_mmps;
    int32_t distance_target_mm;
    int32_t distance_measure_mm;
    int16_t distance_error_mm;
    int16_t heading_target_deg10;
    int16_t heading_measure_deg10;
    int16_t heading_error_deg10;
    int16_t yaw_rate_target_mdps;
    int16_t yaw_rate_measure_mdps;
    int16_t yaw_rate_error_mdps;
    int16_t line_target;
    int16_t line_measure;
    int16_t line_error;
    int16_t distance_output_speed_mmps;
    int16_t attitude_output_yaw_mdps;
    int16_t line_output_yaw_mdps;
    int16_t turn_output_diff_mmps;
    int16_t speed_output_left_duty;
    int16_t speed_output_right_duty;
    uint8_t distance_loop_ready;
    uint8_t attitude_loop_ready;
    uint8_t turn_loop_ready;
    uint8_t line_loop_ready;
    uint8_t speed_loop_ready;
} MotionControl_Status_t;

void MotionControl_Init(void);
void MotionControl_Tick10ms(void);
void MotionControl_GetStatus(MotionControl_Status_t *status);

#endif
