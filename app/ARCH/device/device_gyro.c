#include "ARCH/device/device_gyro.h"

#include <stdio.h>
#include <string.h>

#include "AppTick.h"
#include "ARCH/config/app_config.h"
#include "ARCH/device/device_gyro_imu660ra_fw.h"

#define IMU660RA_CONFIG_CHUNK_SIZE               (32U)

#define IMU660RA_ADDR                            (0x69U)
#define IMU660RA_WHO_AM_I_REG                    (0x00U)
#define IMU660RA_WHO_AM_I_VALUE                  (0x24U)
#define IMU660RA_GYRO_DATA_REG                   (0x12U)
#define IMU660RA_INTERNAL_STATUS_REG             (0x21U)
#define IMU660RA_PWR_CONF_REG                    (0x7CU)
#define IMU660RA_PWR_CTRL_REG                    (0x7DU)
#define IMU660RA_INIT_CTRL_REG                   (0x59U)
#define IMU660RA_INIT_ADDR_0_REG                 (0x5BU)
#define IMU660RA_INIT_ADDR_1_REG                 (0x5CU)
#define IMU660RA_INIT_DATA_REG                   (0x5EU)
#define IMU660RA_GYR_CONF_REG                    (0x42U)
#define IMU660RA_GYR_RANGE_REG                   (0x43U)
#define IMU660RA_INTERNAL_STATUS_READY           (0x01U)

#define MPU6050_ADDR0                            (0x68U)
#define MPU6050_ADDR1                            (0x69U)
#define MPU6050_WHO_AM_I_REG                     (0x75U)
#define MPU6050_WHO_AM_I_VALUE                   (0x68U)
#define MPU6050_PWR_MGMT_1_REG                   (0x6BU)
#define MPU6050_GYRO_CONFIG_REG                  (0x1BU)
#define MPU6050_GYRO_ZOUT_H_REG                  (0x47U)

typedef enum
{
    GYRO_SENSOR_NONE = 0,
    GYRO_SENSOR_IMU660RA,
    GYRO_SENSOR_MPU6050,
} DeviceGyro_SensorType_t;

typedef enum
{
    GYRO_STATE_DISABLED = 0,
    GYRO_STATE_PROBE_IMU660RA,
    GYRO_STATE_PROBE_MPU6050_68,
    GYRO_STATE_PROBE_MPU6050_69,
    GYRO_STATE_INIT_IMU660RA_PWR_CONF,
    GYRO_STATE_INIT_IMU660RA_INIT_CTRL,
    GYRO_STATE_INIT_IMU660RA_LOAD_CFG,
    GYRO_STATE_INIT_IMU660RA_ENABLE_CFG,
    GYRO_STATE_INIT_IMU660RA_CHECK_STATUS,
    GYRO_STATE_INIT_IMU660RA_PWR_CTRL,
    GYRO_STATE_INIT_IMU660RA_GYR_CONF,
    GYRO_STATE_INIT_IMU660RA_GYR_RANGE,
    GYRO_STATE_INIT_MPU6050_PWR,
    GYRO_STATE_INIT_MPU6050_GYRO_CFG,
    GYRO_STATE_CALIBRATE,
    GYRO_STATE_STREAM,
} DeviceGyro_State_t;

typedef struct
{
    DeviceGyro_State_t state;
    DeviceGyro_SensorType_t sensor;
    uint8_t address;
    uint8_t calibrate_count;
    int32_t calibrate_sum_raw;
    int16_t bias_raw;
    int32_t heading_accum_x100;
    int32_t yaw_filter_dps10;
    uint32_t next_action_ms;
    uint16_t init_config_offset;
    int16_t last_raw_z;
    uint8_t stream_error_count;
    uint8_t internal_status;
    uint8_t last_raw_count;
    uint8_t bias_track_count;
    uint8_t heading_active_count;
    uint8_t last_raw_bytes[6];
} DeviceGyro_Runtime_t;

static DeviceGyro_Snapshot_t g_gyro;
static DeviceGyro_Runtime_t g_runtime;
static char g_last_line[32];
static char g_last_rx_hex[24];
static uint8_t g_last_probe_value;

static void DeviceGyro_SetDebugText(const char *line_text, const char *hex_text)
{
    memset(g_last_line, 0, sizeof(g_last_line));
    memset(g_last_rx_hex, 0, sizeof(g_last_rx_hex));

    if (line_text != 0) {
        strncpy(g_last_line, line_text, sizeof(g_last_line) - 1U);
    }
    if (hex_text != 0) {
        strncpy(g_last_rx_hex, hex_text, sizeof(g_last_rx_hex) - 1U);
    }
}

static const char *DeviceGyro_GetSensorName(DeviceGyro_SensorType_t sensor)
{
    switch (sensor) {
    case GYRO_SENSOR_IMU660RA:
        return "IMU660RA";
    case GYRO_SENSOR_MPU6050:
        return "MPU6050";
    case GYRO_SENSOR_NONE:
    default:
        return "Gyro";
    }
}

static int16_t DeviceGyro_ClampInt16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t DeviceGyro_AbsInt16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

static uint8_t DeviceGyro_IsReady(void)
{
    return (uint8_t)((int32_t)(AppTick_GetMs() - g_runtime.next_action_ms) >= 0);
}

static void DeviceGyro_SetRetry(const char *line_text, const char *hex_text)
{
    g_runtime.sensor = GYRO_SENSOR_NONE;
    g_runtime.state = GYRO_STATE_PROBE_IMU660RA;
    g_runtime.address = 0U;
    g_runtime.calibrate_count = 0U;
    g_runtime.calibrate_sum_raw = 0;
    g_runtime.bias_raw = 0;
    g_runtime.heading_accum_x100 = 0;
    g_runtime.yaw_filter_dps10 = 0;
    g_runtime.init_config_offset = 0U;
    g_runtime.last_raw_z = 0;
    g_runtime.stream_error_count = 0U;
    g_runtime.internal_status = 0U;
    g_runtime.last_raw_count = 0U;
    g_runtime.bias_track_count = 0U;
    g_runtime.heading_active_count = 0U;
    memset(g_runtime.last_raw_bytes, 0, sizeof(g_runtime.last_raw_bytes));
    g_runtime.next_action_ms = AppTick_GetMs() + APP_BOARD_GYRO_RETRY_PERIOD_MS;

    g_gyro.rx_bytes = 0U;
    g_gyro.yaw_mdps = 0;
    g_gyro.heading_deg10 = 0;
    g_gyro.online = 0U;
    g_gyro.parser_locked = 0U;

    DeviceGyro_SetDebugText(line_text, hex_text);
}

static void GyroI2C_BitDelay(void)
{
    delay_cycles(APP_BOARD_GYRO_I2C_DELAY_CYCLES);
}

static void GyroI2C_SclRelease(void)
{
    DL_GPIO_initDigitalInputFeatures(APP_BOARD_GYRO_I2C_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(APP_BOARD_GYRO_I2C_SCL_PORT, APP_BOARD_GYRO_I2C_SCL_PIN);
}

static void GyroI2C_SdaRelease(void)
{
    DL_GPIO_initDigitalInputFeatures(APP_BOARD_GYRO_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(APP_BOARD_GYRO_I2C_SDA_PORT, APP_BOARD_GYRO_I2C_SDA_PIN);
}

static void GyroI2C_SclLow(void)
{
    DL_GPIO_initDigitalOutputFeatures(APP_BOARD_GYRO_I2C_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(APP_BOARD_GYRO_I2C_SCL_PORT, APP_BOARD_GYRO_I2C_SCL_PIN);
    DL_GPIO_enableOutput(APP_BOARD_GYRO_I2C_SCL_PORT, APP_BOARD_GYRO_I2C_SCL_PIN);
}

static void GyroI2C_SdaLow(void)
{
    DL_GPIO_initDigitalOutputFeatures(APP_BOARD_GYRO_I2C_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW,
        DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(APP_BOARD_GYRO_I2C_SDA_PORT, APP_BOARD_GYRO_I2C_SDA_PIN);
    DL_GPIO_enableOutput(APP_BOARD_GYRO_I2C_SDA_PORT, APP_BOARD_GYRO_I2C_SDA_PIN);
}

static uint8_t GyroI2C_ReadScl(void)
{
    return (DL_GPIO_readPins(APP_BOARD_GYRO_I2C_SCL_PORT, APP_BOARD_GYRO_I2C_SCL_PIN) != 0U) ?
        1U : 0U;
}

static uint8_t GyroI2C_ReadSda(void)
{
    return (DL_GPIO_readPins(APP_BOARD_GYRO_I2C_SDA_PORT, APP_BOARD_GYRO_I2C_SDA_PIN) != 0U) ?
        1U : 0U;
}

static uint8_t GyroI2C_WaitSclHigh(void)
{
    uint8_t retry = 16U;

    while (retry-- > 0U) {
        if (GyroI2C_ReadScl() != 0U) {
            return 1U;
        }
        GyroI2C_BitDelay();
    }

    return 0U;
}

static void GyroI2C_Stop(void)
{
    GyroI2C_SdaLow();
    GyroI2C_BitDelay();
    GyroI2C_SclRelease();
    GyroI2C_WaitSclHigh();
    GyroI2C_BitDelay();
    GyroI2C_SdaRelease();
    GyroI2C_BitDelay();
}

static uint8_t GyroI2C_Start(void)
{
    GyroI2C_SdaRelease();
    GyroI2C_SclRelease();
    GyroI2C_BitDelay();
    if (!GyroI2C_WaitSclHigh()) {
        return 0U;
    }
    GyroI2C_SdaLow();
    GyroI2C_BitDelay();
    GyroI2C_SclLow();
    GyroI2C_BitDelay();
    return 1U;
}

static uint8_t GyroI2C_WriteByte(uint8_t byte_data)
{
    uint8_t bit_mask;

    for (bit_mask = 0x80U; bit_mask != 0U; bit_mask >>= 1U) {
        if ((byte_data & bit_mask) != 0U) {
            GyroI2C_SdaRelease();
        } else {
            GyroI2C_SdaLow();
        }

        GyroI2C_BitDelay();
        GyroI2C_SclRelease();
        if (!GyroI2C_WaitSclHigh()) {
            GyroI2C_SclLow();
            return 0U;
        }
        GyroI2C_BitDelay();
        GyroI2C_SclLow();
    }

    GyroI2C_SdaRelease();
    GyroI2C_BitDelay();
    GyroI2C_SclRelease();
    if (!GyroI2C_WaitSclHigh()) {
        GyroI2C_SclLow();
        return 0U;
    }
    GyroI2C_BitDelay();
    byte_data = (GyroI2C_ReadSda() == 0U) ? 1U : 0U;
    GyroI2C_SclLow();
    GyroI2C_BitDelay();

    return byte_data;
}

static uint8_t GyroI2C_ReadByte(uint8_t ack_enable)
{
    uint8_t bit_index;
    uint8_t byte_data = 0U;

    GyroI2C_SdaRelease();
    for (bit_index = 0U; bit_index < 8U; bit_index++) {
        byte_data <<= 1U;
        GyroI2C_BitDelay();
        GyroI2C_SclRelease();
        if (!GyroI2C_WaitSclHigh()) {
            GyroI2C_SclLow();
            return 0xFFU;
        }
        GyroI2C_BitDelay();
        if (GyroI2C_ReadSda() != 0U) {
            byte_data |= 0x01U;
        }
        GyroI2C_SclLow();
    }

    if (ack_enable != 0U) {
        GyroI2C_SdaLow();
    } else {
        GyroI2C_SdaRelease();
    }
    GyroI2C_BitDelay();
    GyroI2C_SclRelease();
    GyroI2C_WaitSclHigh();
    GyroI2C_BitDelay();
    GyroI2C_SclLow();
    GyroI2C_SdaRelease();
    GyroI2C_BitDelay();

    return byte_data;
}

static void GyroI2C_Recovery(void)
{
    uint8_t index;

    GyroI2C_SdaRelease();
    for (index = 0U; index < 9U; index++) {
        GyroI2C_SclRelease();
        GyroI2C_BitDelay();
        GyroI2C_SclLow();
        GyroI2C_BitDelay();
    }
    GyroI2C_Stop();
}

static uint8_t GyroI2C_WriteReg(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t ok = 0U;

    if (!GyroI2C_Start()) {
        GyroI2C_Stop();
        return 0U;
    }

    if (!GyroI2C_WriteByte((uint8_t)(address << 1U))) {
        goto exit_i2c;
    }
    if (!GyroI2C_WriteByte(reg)) {
        goto exit_i2c;
    }
    if (!GyroI2C_WriteByte(value)) {
        goto exit_i2c;
    }

    ok = 1U;

exit_i2c:
    GyroI2C_Stop();
    return ok;
}

static uint8_t GyroI2C_WriteRegs(uint8_t address, uint8_t reg, const uint8_t *data, uint8_t length)
{
    uint8_t index;
    uint8_t ok = 0U;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }

    if (!GyroI2C_Start()) {
        GyroI2C_Stop();
        return 0U;
    }

    if (!GyroI2C_WriteByte((uint8_t)(address << 1U))) {
        goto exit_i2c;
    }
    if (!GyroI2C_WriteByte(reg)) {
        goto exit_i2c;
    }
    for (index = 0U; index < length; index++) {
        if (!GyroI2C_WriteByte(data[index])) {
            goto exit_i2c;
        }
    }

    ok = 1U;

exit_i2c:
    GyroI2C_Stop();
    return ok;
}

static uint8_t GyroI2C_ReadRegs(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t index;
    uint8_t ok = 0U;

    if ((data == 0) || (length == 0U)) {
        return 0U;
    }

    if (!GyroI2C_Start()) {
        GyroI2C_Stop();
        return 0U;
    }

    if (!GyroI2C_WriteByte((uint8_t)(address << 1U))) {
        goto exit_i2c;
    }
    if (!GyroI2C_WriteByte(reg)) {
        goto exit_i2c;
    }
    if (!GyroI2C_Start()) {
        goto exit_i2c;
    }
    if (!GyroI2C_WriteByte((uint8_t)((address << 1U) | 0x01U))) {
        goto exit_i2c;
    }

    for (index = 0U; index < length; index++) {
        data[index] = GyroI2C_ReadByte((uint8_t)(index + 1U < length));
    }

    ok = 1U;

exit_i2c:
    GyroI2C_Stop();
    return ok;
}

static uint8_t DeviceGyro_ReadWhoAmI(uint8_t address, uint8_t reg, uint8_t *value)
{
    return GyroI2C_ReadRegs(address, reg, value, 1U);
}

static uint8_t DeviceGyro_ReadReg8(uint8_t address, uint8_t reg, uint8_t *value)
{
    return GyroI2C_ReadRegs(address, reg, value, 1U);
}

static uint8_t DeviceGyro_LoadImu660raConfigChunk(void)
{
    uint8_t init_addr[2];
    uint8_t chunk_length;
    uint16_t addr_word;
    uint16_t remaining;

    if (g_runtime.init_config_offset >= IMU660RA_CONFIG_FILE_SIZE) {
        return 1U;
    }

    remaining = (uint16_t)(IMU660RA_CONFIG_FILE_SIZE - g_runtime.init_config_offset);
    chunk_length = (remaining > IMU660RA_CONFIG_CHUNK_SIZE) ? IMU660RA_CONFIG_CHUNK_SIZE : (uint8_t)remaining;
    addr_word = (uint16_t)(g_runtime.init_config_offset >> 1U);

    init_addr[0] = (uint8_t)(addr_word & 0x0FU);
    init_addr[1] = (uint8_t)((addr_word >> 4U) & 0xFFU);

    if (!GyroI2C_WriteRegs(g_runtime.address, IMU660RA_INIT_ADDR_0_REG, init_addr, 2U)) {
        return 0U;
    }
    if (!GyroI2C_WriteRegs(g_runtime.address, IMU660RA_INIT_DATA_REG,
        &g_imu660ra_config_file[g_runtime.init_config_offset], chunk_length)) {
        return 0U;
    }

    g_runtime.init_config_offset = (uint16_t)(g_runtime.init_config_offset + chunk_length);
    return 1U;
}

static uint8_t DeviceGyro_ReadRawZ(int16_t *raw_z, uint8_t *raw_bytes, uint8_t *byte_count)
{
    uint8_t buffer[6];
    uint8_t read_len;

    if ((raw_z == 0) || (raw_bytes == 0) || (byte_count == 0)) {
        return 0U;
    }

    if (g_runtime.sensor == GYRO_SENSOR_IMU660RA) {
        read_len = 6U;
        if (!GyroI2C_ReadRegs(g_runtime.address, IMU660RA_GYRO_DATA_REG, buffer, read_len)) {
            return 0U;
        }
        *raw_z = (int16_t)(((uint16_t)buffer[5] << 8U) | buffer[4]);
    } else if (g_runtime.sensor == GYRO_SENSOR_MPU6050) {
        read_len = 2U;
        if (!GyroI2C_ReadRegs(g_runtime.address, MPU6050_GYRO_ZOUT_H_REG, buffer, read_len)) {
            return 0U;
        }
        *raw_z = (int16_t)(((uint16_t)buffer[0] << 8U) | buffer[1]);
    } else {
        return 0U;
    }

    raw_bytes[0] = buffer[0];
    raw_bytes[1] = buffer[1];
    if (read_len > 2U) {
        raw_bytes[2] = buffer[2];
        raw_bytes[3] = buffer[3];
        raw_bytes[4] = buffer[4];
        raw_bytes[5] = buffer[5];
    }
    *byte_count = read_len;
    g_gyro.rx_bytes += read_len;
    g_runtime.last_raw_z = *raw_z;
    g_runtime.last_raw_count = read_len;
    memset(g_runtime.last_raw_bytes, 0, sizeof(g_runtime.last_raw_bytes));
    memcpy(g_runtime.last_raw_bytes, raw_bytes, read_len);
    return 1U;
}

static void DeviceGyro_UpdateStreamData(int16_t raw_z, const uint8_t *raw_bytes, uint8_t byte_count)
{
    int16_t corrected_raw;
    int32_t yaw_dps10_raw;
    int32_t yaw_dps10_filtered;
    int32_t heading_integrate_dps10;
    char line_text[32];
    char hex_text[24];

    corrected_raw = (int16_t)(raw_z - g_runtime.bias_raw);

    if (DeviceGyro_AbsInt16(corrected_raw) <= APP_GYRO_BIAS_TRACK_WINDOW_RAW) {
        if (g_runtime.bias_track_count < 255U) {
            g_runtime.bias_track_count++;
        }

        if (g_runtime.bias_track_count >= APP_GYRO_BIAS_TRACK_COUNT) {
            g_runtime.bias_raw = (int16_t)(((int32_t)g_runtime.bias_raw * 31L + raw_z) / 32L);
            corrected_raw = (int16_t)(raw_z - g_runtime.bias_raw);
        }
    } else {
        g_runtime.bias_track_count = 0U;
    }

    if (DeviceGyro_AbsInt16(corrected_raw) <= APP_GYRO_RAW_DEADBAND) {
        corrected_raw = 0;
    }

    yaw_dps10_raw = ((int32_t)corrected_raw * 100L) / 164L;
    g_runtime.yaw_filter_dps10 +=
        (yaw_dps10_raw - g_runtime.yaw_filter_dps10) / APP_GYRO_YAW_FILTER_DIV;
    yaw_dps10_filtered = g_runtime.yaw_filter_dps10;

    if (DeviceGyro_AbsInt16((int16_t)yaw_dps10_filtered) <= APP_GYRO_YAW_DEADBAND_DPS10) {
        yaw_dps10_filtered = 0;
        g_runtime.yaw_filter_dps10 = 0;
    }

    heading_integrate_dps10 = yaw_dps10_filtered;
    if (DeviceGyro_AbsInt16((int16_t)yaw_dps10_filtered) < APP_GYRO_HEADING_INTEGRATE_DPS10) {
        g_runtime.heading_active_count = 0U;
        heading_integrate_dps10 = 0;
    } else {
        if (g_runtime.heading_active_count < 255U) {
            g_runtime.heading_active_count++;
        }
        if (g_runtime.heading_active_count < APP_GYRO_HEADING_ACTIVE_COUNT) {
            heading_integrate_dps10 = 0;
        }
    }

    g_runtime.heading_accum_x100 += heading_integrate_dps10;

    /*
     * The current control-layer interface still keeps int16 yaw fields.
     * We store 0.1 dps here first, and can migrate the whole stack to
     * true mdps int32 later when the closed-loop gyro path is enabled.
     */
    g_gyro.yaw_mdps = DeviceGyro_ClampInt16(yaw_dps10_filtered);
    g_gyro.heading_deg10 = DeviceGyro_ClampInt16(g_runtime.heading_accum_x100 / 100L);
    g_gyro.online = 1U;
    g_gyro.parser_locked = 1U;

    snprintf(line_text, sizeof(line_text), "%s Z=%6d",
        DeviceGyro_GetSensorName(g_runtime.sensor),
        (int)g_gyro.yaw_mdps);

    if (byte_count >= 6U) {
        snprintf(hex_text, sizeof(hex_text), "%02X %02X %02X %02X %02X %02X",
            raw_bytes[0], raw_bytes[1], raw_bytes[2],
            raw_bytes[3], raw_bytes[4], raw_bytes[5]);
    } else {
        snprintf(hex_text, sizeof(hex_text), "%02X %02X raw=%d",
            raw_bytes[0], raw_bytes[1], (int)raw_z);
    }

    DeviceGyro_SetDebugText(line_text, hex_text);
}

static void DeviceGyro_EnterCalibrate(void)
{
    char line_text[32];

    g_runtime.state = GYRO_STATE_CALIBRATE;
    g_runtime.calibrate_count = 0U;
    g_runtime.calibrate_sum_raw = 0;
    g_runtime.bias_raw = 0;
    g_runtime.heading_accum_x100 = 0;
    g_runtime.yaw_filter_dps10 = 0;
    g_runtime.last_raw_z = 0;
    g_runtime.last_raw_count = 0U;
    memset(g_runtime.last_raw_bytes, 0, sizeof(g_runtime.last_raw_bytes));
    g_runtime.stream_error_count = 0U;
    g_runtime.bias_track_count = 0U;
    g_runtime.heading_active_count = 0U;
    g_runtime.next_action_ms = AppTick_GetMs();

    g_gyro.yaw_mdps = 0;
    g_gyro.heading_deg10 = 0;
    g_gyro.online = 1U;
    g_gyro.parser_locked = 0U;

    snprintf(line_text, sizeof(line_text), "%s calibrating",
        DeviceGyro_GetSensorName(g_runtime.sensor));
    DeviceGyro_SetDebugText(line_text, "keep still");
}

static void DeviceGyro_HandleCalibrate(void)
{
    int16_t raw_z;
    uint8_t raw_bytes[6] = {0};
    uint8_t byte_count = 0U;
    char line_text[32];
    char hex_text[24];

    if (!DeviceGyro_ReadRawZ(&raw_z, raw_bytes, &byte_count)) {
        DeviceGyro_SetRetry("Gyro cal fail", "read err");
        return;
    }

    g_runtime.calibrate_sum_raw += raw_z;
    g_runtime.calibrate_count++;
    g_gyro.online = 1U;
    g_gyro.parser_locked = 0U;

    snprintf(line_text, sizeof(line_text), "%s cal %2u/%2u",
        DeviceGyro_GetSensorName(g_runtime.sensor),
        (unsigned)g_runtime.calibrate_count,
        (unsigned)APP_GYRO_CALIBRATE_SAMPLE_COUNT);
    snprintf(hex_text, sizeof(hex_text), "raw=%d", (int)raw_z);
    DeviceGyro_SetDebugText(line_text, hex_text);

    if (g_runtime.calibrate_count >= APP_GYRO_CALIBRATE_SAMPLE_COUNT) {
        g_runtime.bias_raw = (int16_t)(g_runtime.calibrate_sum_raw /
            (int32_t)APP_GYRO_CALIBRATE_SAMPLE_COUNT);
        g_runtime.state = GYRO_STATE_STREAM;
        g_runtime.heading_accum_x100 = 0;
        g_runtime.yaw_filter_dps10 = 0;
        g_runtime.bias_track_count = 0U;
        g_runtime.heading_active_count = 0U;
        g_runtime.next_action_ms = AppTick_GetMs();
        g_gyro.yaw_mdps = 0;
        g_gyro.heading_deg10 = 0;

        snprintf(line_text, sizeof(line_text), "%s ready bias=%d",
            DeviceGyro_GetSensorName(g_runtime.sensor),
            (int)g_runtime.bias_raw);
        DeviceGyro_SetDebugText(line_text, "stream on");
    }
}

static void DeviceGyro_HandleStream(void)
{
    int16_t raw_z;
    uint8_t raw_bytes[6] = {0};
    uint8_t byte_count = 0U;

    if (!DeviceGyro_ReadRawZ(&raw_z, raw_bytes, &byte_count)) {
        g_runtime.stream_error_count++;
        if (g_runtime.stream_error_count >= 3U) {
            DeviceGyro_SetRetry("Gyro stream err", "retry probe");
        } else {
            DeviceGyro_SetDebugText("Gyro read retry", "stream miss");
        }
        return;
    }

    g_runtime.stream_error_count = 0U;
    DeviceGyro_UpdateStreamData(raw_z, raw_bytes, byte_count);
}

void DeviceGyro_Init(void)
{
    memset(&g_gyro, 0, sizeof(g_gyro));
    memset(&g_runtime, 0, sizeof(g_runtime));
    g_last_probe_value = 0U;

    GyroI2C_SclRelease();
    GyroI2C_SdaRelease();
    GyroI2C_Recovery();

    if (APP_BOARD_ENABLE_GYRO_I2C) {
        g_runtime.state = GYRO_STATE_PROBE_IMU660RA;
        g_runtime.next_action_ms = AppTick_GetMs();
        DeviceGyro_SetDebugText("Gyro probe A26/A8", "soft I2C");
    } else {
        g_runtime.state = GYRO_STATE_DISABLED;
        DeviceGyro_SetDebugText("Gyro disabled", "cfg off");
    }
}

void DeviceGyro_Tick10ms(void)
{
    uint8_t who_am_i = 0U;
    uint8_t status_reg = 0U;
    char hex_text[24];

    if (!APP_BOARD_ENABLE_GYRO_I2C) {
        g_runtime.state = GYRO_STATE_DISABLED;
        g_gyro.yaw_mdps = 0;
        g_gyro.heading_deg10 = 0;
        g_gyro.online = 0U;
        g_gyro.parser_locked = 0U;
        DeviceGyro_SetDebugText("Gyro disabled", "cfg off");
        return;
    }

    if (!DeviceGyro_IsReady()) {
        return;
    }

    switch (g_runtime.state) {
    case GYRO_STATE_PROBE_IMU660RA:
        g_last_probe_value = who_am_i;
        if (DeviceGyro_ReadWhoAmI(IMU660RA_ADDR, IMU660RA_WHO_AM_I_REG, &who_am_i) &&
            (who_am_i == IMU660RA_WHO_AM_I_VALUE)) {
            g_last_probe_value = who_am_i;
            g_runtime.sensor = GYRO_SENSOR_IMU660RA;
            g_runtime.address = IMU660RA_ADDR;
            g_runtime.state = GYRO_STATE_INIT_IMU660RA_PWR_CONF;
            g_runtime.init_config_offset = 0U;
            g_runtime.internal_status = 0U;
            DeviceGyro_SetDebugText("IMU660RA found", "WHO=24");
        } else {
            g_last_probe_value = who_am_i;
            snprintf(hex_text, sizeof(hex_text), "WHO=%02X", who_am_i);
            g_runtime.state = GYRO_STATE_PROBE_MPU6050_68;
            DeviceGyro_SetDebugText("Probe MPU6050@68", hex_text);
        }
        break;

    case GYRO_STATE_PROBE_MPU6050_68:
        g_last_probe_value = who_am_i;
        if (DeviceGyro_ReadWhoAmI(MPU6050_ADDR0, MPU6050_WHO_AM_I_REG, &who_am_i) &&
            (who_am_i == MPU6050_WHO_AM_I_VALUE)) {
            g_last_probe_value = who_am_i;
            g_runtime.sensor = GYRO_SENSOR_MPU6050;
            g_runtime.address = MPU6050_ADDR0;
            g_runtime.state = GYRO_STATE_INIT_MPU6050_PWR;
            DeviceGyro_SetDebugText("MPU6050@68 found", "WHO=68");
        } else {
            g_last_probe_value = who_am_i;
            snprintf(hex_text, sizeof(hex_text), "WHO=%02X", who_am_i);
            g_runtime.state = GYRO_STATE_PROBE_MPU6050_69;
            DeviceGyro_SetDebugText("Probe MPU6050@69", hex_text);
        }
        break;

    case GYRO_STATE_PROBE_MPU6050_69:
        g_last_probe_value = who_am_i;
        if (DeviceGyro_ReadWhoAmI(MPU6050_ADDR1, MPU6050_WHO_AM_I_REG, &who_am_i) &&
            (who_am_i == MPU6050_WHO_AM_I_VALUE)) {
            g_last_probe_value = who_am_i;
            g_runtime.sensor = GYRO_SENSOR_MPU6050;
            g_runtime.address = MPU6050_ADDR1;
            g_runtime.state = GYRO_STATE_INIT_MPU6050_PWR;
            DeviceGyro_SetDebugText("MPU6050@69 found", "WHO=68");
        } else {
            g_last_probe_value = who_am_i;
            snprintf(hex_text, sizeof(hex_text), "WHO=%02X", who_am_i);
            DeviceGyro_SetRetry("Gyro not found", hex_text);
        }
        break;

    case GYRO_STATE_INIT_IMU660RA_PWR_CONF:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_PWR_CONF_REG, 0x00U)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "PWR_CONF");
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_INIT_CTRL;
        DeviceGyro_SetDebugText("IMU660 PWR_CONF", "7C=00");
        break;

    case GYRO_STATE_INIT_IMU660RA_INIT_CTRL:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_INIT_CTRL_REG, 0x00U)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "INIT_CTRL0");
            break;
        }
        g_runtime.init_config_offset = 0U;
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_LOAD_CFG;
        DeviceGyro_SetDebugText("IMU660 load cfg", "0/8192");
        break;

    case GYRO_STATE_INIT_IMU660RA_LOAD_CFG:
        if (!DeviceGyro_LoadImu660raConfigChunk()) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "cfg chunk");
            break;
        }
        snprintf(hex_text, sizeof(hex_text), "%u/%u",
            (unsigned int)g_runtime.init_config_offset,
            (unsigned int)IMU660RA_CONFIG_FILE_SIZE);
        if (g_runtime.init_config_offset >= IMU660RA_CONFIG_FILE_SIZE) {
            g_runtime.state = GYRO_STATE_INIT_IMU660RA_ENABLE_CFG;
            DeviceGyro_SetDebugText("IMU660 cfg sent", hex_text);
        } else {
            DeviceGyro_SetDebugText("IMU660 load cfg", hex_text);
        }
        break;

    case GYRO_STATE_INIT_IMU660RA_ENABLE_CFG:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_INIT_CTRL_REG, 0x01U)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "INIT_CTRL1");
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_CHECK_STATUS;
        g_runtime.next_action_ms = AppTick_GetMs() + 30U;
        DeviceGyro_SetDebugText("IMU660 cfg apply", "wait ready");
        break;

    case GYRO_STATE_INIT_IMU660RA_CHECK_STATUS:
        if (!DeviceGyro_ReadReg8(g_runtime.address, IMU660RA_INTERNAL_STATUS_REG, &status_reg)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "INT_STA rd");
            break;
        }
        g_runtime.internal_status = status_reg;
        snprintf(hex_text, sizeof(hex_text), "INT=%02X", status_reg);
        if (status_reg != IMU660RA_INTERNAL_STATUS_READY) {
            DeviceGyro_SetRetry("IMU660 cfg fail", hex_text);
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_PWR_CTRL;
        DeviceGyro_SetDebugText("IMU660 cfg ok", hex_text);
        break;

    case GYRO_STATE_INIT_IMU660RA_PWR_CTRL:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_PWR_CTRL_REG, 0x0EU)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "PWR_CTRL");
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_GYR_CONF;
        DeviceGyro_SetDebugText("IMU660 PWR_CTRL", "7D=0E");
        break;

    case GYRO_STATE_INIT_IMU660RA_GYR_CONF:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_GYR_CONF_REG, 0xA9U)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "GYR_CONF");
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_IMU660RA_GYR_RANGE;
        DeviceGyro_SetDebugText("IMU660 GYR_CONF", "42=A9");
        break;

    case GYRO_STATE_INIT_IMU660RA_GYR_RANGE:
        if (!GyroI2C_WriteReg(g_runtime.address, IMU660RA_GYR_RANGE_REG, 0x00U)) {
            DeviceGyro_SetRetry("IMU660 cfg fail", "GYR_RANGE");
            break;
        }
        DeviceGyro_EnterCalibrate();
        break;

    case GYRO_STATE_INIT_MPU6050_PWR:
        if (!GyroI2C_WriteReg(g_runtime.address, MPU6050_PWR_MGMT_1_REG, 0x00U)) {
            DeviceGyro_SetRetry("MPU cfg fail", "PWR_MGMT");
            break;
        }
        g_runtime.state = GYRO_STATE_INIT_MPU6050_GYRO_CFG;
        DeviceGyro_SetDebugText("MPU PWR_MGMT_1", "6B=00");
        break;

    case GYRO_STATE_INIT_MPU6050_GYRO_CFG:
        if (!GyroI2C_WriteReg(g_runtime.address, MPU6050_GYRO_CONFIG_REG, 0x18U)) {
            DeviceGyro_SetRetry("MPU cfg fail", "GYRO_CFG");
            break;
        }
        DeviceGyro_EnterCalibrate();
        break;

    case GYRO_STATE_CALIBRATE:
        DeviceGyro_HandleCalibrate();
        break;

    case GYRO_STATE_STREAM:
        DeviceGyro_HandleStream();
        break;

    case GYRO_STATE_DISABLED:
    default:
        g_gyro.yaw_mdps = 0;
        g_gyro.heading_deg10 = 0;
        g_gyro.online = 0U;
        g_gyro.parser_locked = 0U;
        DeviceGyro_SetDebugText("Gyro disabled", "cfg off");
        break;
    }
}

void DeviceGyro_GetSnapshot(DeviceGyro_Snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = g_gyro;
}

void DeviceGyro_ResetHeading(void)
{
    g_runtime.heading_accum_x100 = 0;
    g_runtime.yaw_filter_dps10 = 0;
    g_runtime.last_raw_z = 0;
    g_runtime.last_raw_count = 0U;
    g_runtime.heading_active_count = 0U;
    memset(g_runtime.last_raw_bytes, 0, sizeof(g_runtime.last_raw_bytes));
    g_gyro.heading_deg10 = 0;
    g_gyro.yaw_mdps = 0;
}

void DeviceGyro_GetDebugInfo(DeviceGyro_DebugInfo_t *info)
{
    if (info == 0) {
        return;
    }

    memset(info, 0, sizeof(*info));
    info->raw_z = g_runtime.last_raw_z;
    info->yaw_mdps = g_gyro.yaw_mdps;
    info->heading_deg10 = g_gyro.heading_deg10;
    info->bias_raw = g_runtime.bias_raw;
    info->init_config_offset = g_runtime.init_config_offset;
    info->rx_bytes = g_gyro.rx_bytes;
    info->online = g_gyro.online;
    info->parser_locked = g_gyro.parser_locked;
    info->state = (uint8_t)g_runtime.state;
    info->sensor = (uint8_t)g_runtime.sensor;
    info->address = g_runtime.address;
    info->internal_status = g_runtime.internal_status;
    info->calibrate_count = g_runtime.calibrate_count;
    info->stream_error_count = g_runtime.stream_error_count;
    info->last_probe_value = g_last_probe_value;
    info->raw_byte_count = g_runtime.last_raw_count;
    memcpy(info->raw_bytes, g_runtime.last_raw_bytes, sizeof(info->raw_bytes));
    memcpy(info->last_line, g_last_line, sizeof(info->last_line));
    memcpy(info->last_rx_hex, g_last_rx_hex, sizeof(info->last_rx_hex));
}
