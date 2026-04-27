#ifndef APP_ARCH_DEVICE_GYRO_H
#define APP_ARCH_DEVICE_GYRO_H

#include <stdint.h>

typedef struct
{
    int16_t yaw_mdps;
    int16_t heading_deg10;
    uint32_t rx_bytes;
    uint8_t online;
    uint8_t parser_locked;
} DeviceGyro_Snapshot_t;

typedef struct
{
    int16_t raw_z;
    int16_t yaw_mdps;
    int16_t heading_deg10;
    int16_t bias_raw;
    uint16_t init_config_offset;
    uint32_t rx_bytes;
    uint8_t online;
    uint8_t parser_locked;
    uint8_t state;
    uint8_t sensor;
    uint8_t address;
    uint8_t internal_status;
    uint8_t calibrate_count;
    uint8_t stream_error_count;
    uint8_t last_probe_value;
    uint8_t raw_byte_count;
    uint8_t raw_bytes[6];
    char last_line[32];
    char last_rx_hex[24];
} DeviceGyro_DebugInfo_t;

void DeviceGyro_Init(void);
void DeviceGyro_Tick10ms(void);
void DeviceGyro_GetSnapshot(DeviceGyro_Snapshot_t *snapshot);
void DeviceGyro_GetDebugInfo(DeviceGyro_DebugInfo_t *info);
void DeviceGyro_ResetHeading(void);

#endif
