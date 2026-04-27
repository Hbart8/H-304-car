#ifndef APP_ARCH_DEVICE_TRACK_SENSOR_H
#define APP_ARCH_DEVICE_TRACK_SENSOR_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

typedef struct
{
    uint16_t raw[APP_TRACK_SENSOR_COUNT];
    uint8_t binary[APP_TRACK_SENSOR_COUNT];
    int16_t line_position;
    uint8_t online;
    uint8_t current_channel;
} DeviceTrackSensor_Snapshot_t;

void DeviceTrackSensor_Init(void);
void DeviceTrackSensor_Tick10ms(void);
void DeviceTrackSensor_InjectRaw(uint8_t channel, uint16_t value);
void DeviceTrackSensor_GetSnapshot(DeviceTrackSensor_Snapshot_t *snapshot);

#endif
