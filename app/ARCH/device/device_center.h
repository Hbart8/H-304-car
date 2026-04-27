#ifndef APP_ARCH_DEVICE_CENTER_H
#define APP_ARCH_DEVICE_CENTER_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

typedef struct
{
    uint32_t uptime_ms;
    int16_t imu_yaw_mdps;
    int16_t imu_heading_deg10;
    int16_t encoder_heading_deg10;
    int16_t track_line_position;
    int16_t motor_left_duty;
    int16_t motor_right_duty;
    int16_t motor_wheel_duty[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_window_counts[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_mmps[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_filtered_mmps[APP_BOARD_WHEEL_COUNT];
    int16_t wheel_speed_display_mmps[APP_BOARD_WHEEL_COUNT];
    int32_t wheel_distance_mm[APP_BOARD_WHEEL_COUNT];
    int16_t side_speed_mmps[APP_BOARD_SIDE_COUNT];
    int32_t side_distance_mm[APP_BOARD_SIDE_COUNT];
    int32_t chassis_distance_mm;
    uint16_t gray_raw[APP_TRACK_SENSOR_COUNT];
    uint8_t gray_binary[APP_TRACK_SENSOR_COUNT];
    int32_t wheel_ticks[APP_BOARD_WHEEL_COUNT];
    uint8_t imu_online;
    uint8_t imu_parser_locked;
    uint8_t track_online;
    uint8_t motor_pwm_online;
    uint8_t odom_ready;
    uint8_t rgb_r;
    uint8_t rgb_g;
    uint8_t rgb_b;
} DeviceCenter_Snapshot_t;

void DeviceCenter_Init(void);
void DeviceCenter_Tick10ms(void);
void DeviceCenter_GetSnapshot(DeviceCenter_Snapshot_t *snapshot);
void DeviceCenter_ResetOdometry(void);
void DeviceCenter_SetRgb(uint8_t red, uint8_t green, uint8_t blue);
void DeviceCenter_PlayClick(void);
void DeviceCenter_PlayConfirm(void);
void DeviceCenter_PlayStartup(void);

#endif
