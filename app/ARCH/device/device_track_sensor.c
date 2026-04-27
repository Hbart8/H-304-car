#include "ARCH/device/device_track_sensor.h"

#include "ARCH/config/board_profile.h"
#include "ARCH/hal/hal_board.h"

static DeviceTrackSensor_Snapshot_t g_track;

static const HalBoard_Pin_t g_sel0_pin = {APP_BOARD_TRACK_SEL0_PORT, APP_BOARD_TRACK_SEL0_PIN, APP_BOARD_TRACK_SEL0_IOMUX};
static const HalBoard_Pin_t g_sel1_pin = {APP_BOARD_TRACK_SEL1_PORT, APP_BOARD_TRACK_SEL1_PIN, APP_BOARD_TRACK_SEL1_IOMUX};
static const HalBoard_Pin_t g_sel2_pin = {APP_BOARD_TRACK_SEL2_PORT, APP_BOARD_TRACK_SEL2_PIN, APP_BOARD_TRACK_SEL2_IOMUX};

static void DeviceTrackSensor_UpdateDerived(void)
{
    int32_t weight_sum = 0;
    int32_t value_sum = 0;
    uint8_t i;

    for (i = 0U; i < APP_TRACK_SENSOR_COUNT; i++) {
        int32_t weight = ((int32_t)i * 1000) - 3500;

        g_track.binary[i] = (g_track.raw[i] > APP_TRACK_THRESHOLD_DEFAULT) ? 1U : 0U;
        weight_sum += weight * (int32_t)g_track.raw[i];
        value_sum += g_track.raw[i];
    }

    if (value_sum > 0) {
        g_track.line_position = (int16_t)(weight_sum / value_sum);
    } else {
        g_track.line_position = 0;
    }

    if (!APP_BOARD_ENABLE_TRACK_ADC_DIRECT) {
        g_track.online = 0U;
    }
}

static void DeviceTrackSensor_SelectChannel(uint8_t channel)
{
    HalBoard_WritePin(&g_sel0_pin, (uint8_t)(channel & 0x01U));
    HalBoard_WritePin(&g_sel1_pin, (uint8_t)((channel >> 1) & 0x01U));
    HalBoard_WritePin(&g_sel2_pin, (uint8_t)((channel >> 2) & 0x01U));
}

void DeviceTrackSensor_Init(void)
{
    uint8_t i;

    HalBoard_InitOutput(&g_sel0_pin, 0U);
    HalBoard_InitOutput(&g_sel1_pin, 0U);
    HalBoard_InitOutput(&g_sel2_pin, 0U);

    g_track.online = 0U;
    g_track.current_channel = 0U;
    g_track.line_position = 0;

    for (i = 0U; i < APP_TRACK_SENSOR_COUNT; i++) {
        g_track.raw[i] = 0U;
        g_track.binary[i] = 0U;
    }

    DeviceTrackSensor_SelectChannel(0U);
}

void DeviceTrackSensor_Tick10ms(void)
{
    DeviceTrackSensor_SelectChannel(g_track.current_channel);
    g_track.current_channel = (uint8_t)((g_track.current_channel + 1U) % APP_TRACK_SENSOR_COUNT);
    DeviceTrackSensor_UpdateDerived();
}

void DeviceTrackSensor_InjectRaw(uint8_t channel, uint16_t value)
{
    if (channel >= APP_TRACK_SENSOR_COUNT) {
        return;
    }

    g_track.raw[channel] = value;
    if (APP_BOARD_ENABLE_TRACK_ADC_DIRECT) {
        g_track.online = 1U;
    }
    DeviceTrackSensor_UpdateDerived();
}

void DeviceTrackSensor_GetSnapshot(DeviceTrackSensor_Snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = g_track;
}
