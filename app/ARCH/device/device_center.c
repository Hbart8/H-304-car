#include "ARCH/device/device_center.h"

#include "AppTick.h"
#include "Buzzer.h"
#include "ARCH/device/device_gyro.h"
#include "ARCH/device/device_track_sensor.h"
#include "ARCH/hal/hal_chassis.h"

static DeviceCenter_Snapshot_t g_snapshot;
static int32_t g_prev_wheel_ticks[APP_BOARD_WHEEL_COUNT];
static int32_t g_speed_window_delta_ticks[APP_BOARD_WHEEL_COUNT][APP_WHEEL_SPEED_AVG_WINDOW];
static int32_t g_speed_window_sum_ticks[APP_BOARD_WHEEL_COUNT];
static uint8_t g_speed_window_index;
static uint8_t g_odom_ready;
static uint8_t g_odom_auto_zero_pending;
static uint32_t g_odom_auto_zero_ms;

static float DeviceCenter_GetWheelMmPerCount(uint8_t wheel_index)
{
    switch (wheel_index) {
    case APP_BOARD_WHEEL_FRONT_RIGHT_INDEX:
        return APP_ENCODER_FRONT_RIGHT_MM_PER_COUNT * APP_ODOM_DISTANCE_SCALE;
    case APP_BOARD_WHEEL_REAR_RIGHT_INDEX:
        return APP_ENCODER_REAR_RIGHT_MM_PER_COUNT * APP_ODOM_DISTANCE_SCALE;
    case APP_BOARD_WHEEL_FRONT_LEFT_INDEX:
        return APP_ENCODER_FRONT_LEFT_MM_PER_COUNT * APP_ODOM_DISTANCE_SCALE;
    case APP_BOARD_WHEEL_REAR_LEFT_INDEX:
    default:
        return APP_ENCODER_REAR_LEFT_MM_PER_COUNT * APP_ODOM_DISTANCE_SCALE;
    }
}

static int16_t DeviceCenter_RoundToInt16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t DeviceCenter_ClampToInt16(int32_t value)
{
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return (int16_t)value;
}

static int32_t DeviceCenter_RoundToInt32(float value)
{
    if (value > 2147483647.0f) {
        return 2147483647L;
    }
    if (value < -2147483648.0f) {
        return (-2147483647L - 1L);
    }
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t DeviceCenter_FilterSpeed(int16_t prev_value, int16_t raw_value)
{
    return DeviceCenter_RoundToInt16(
        ((float)prev_value * APP_WHEEL_SPEED_FILTER_OLD_WEIGHT) +
        ((float)raw_value * APP_WHEEL_SPEED_FILTER_NEW_WEIGHT));
}

static void DeviceCenter_ResetSpeedObserver(void)
{
    uint8_t i;
    uint8_t j;

    g_speed_window_index = 0U;
    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_speed_window_sum_ticks[i] = 0;
        for (j = 0U; j < APP_WHEEL_SPEED_AVG_WINDOW; j++) {
            g_speed_window_delta_ticks[i][j] = 0;
        }
    }
}

static void DeviceCenter_UpdateOdometry(const HalChassis_EncoderState_t *encoder)
{
    float center_distance_mm;
    float diff_distance_mm;
    float left_side_distance_mm;
    float right_side_distance_mm;
    float left_side_speed_mmps;
    float right_side_speed_mmps;
    int32_t delta_ticks;
    uint8_t i;

    if (encoder == 0) {
        return;
    }

    if (!g_odom_ready) {
        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            g_prev_wheel_ticks[i] = encoder->wheel_ticks[i];
            g_snapshot.motor_wheel_duty[i] = 0;
            g_snapshot.wheel_speed_window_counts[i] = 0;
            g_snapshot.wheel_speed_mmps[i] = 0;
            g_snapshot.wheel_speed_filtered_mmps[i] = 0;
            g_snapshot.wheel_speed_display_mmps[i] = 0;
            g_snapshot.wheel_distance_mm[i] = 0;
        }
        DeviceCenter_ResetSpeedObserver();
        g_snapshot.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] = 0;
        g_snapshot.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX] = 0;
        g_snapshot.side_distance_mm[APP_BOARD_SIDE_LEFT_INDEX] = 0;
        g_snapshot.side_distance_mm[APP_BOARD_SIDE_RIGHT_INDEX] = 0;
        g_snapshot.odom_ready = 1U;
        g_odom_ready = 1U;
        return;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        float delta_mm;
        float avg_delta_mm;
        int16_t avg_speed_mmps;

        delta_ticks = encoder->wheel_ticks[i] - g_prev_wheel_ticks[i];
        g_prev_wheel_ticks[i] = encoder->wheel_ticks[i];
        delta_mm = (float)delta_ticks * DeviceCenter_GetWheelMmPerCount(i);

        g_speed_window_sum_ticks[i] -= g_speed_window_delta_ticks[i][g_speed_window_index];
        g_speed_window_delta_ticks[i][g_speed_window_index] = delta_ticks;
        g_speed_window_sum_ticks[i] += delta_ticks;
        g_snapshot.wheel_speed_window_counts[i] =
            DeviceCenter_ClampToInt16(g_speed_window_sum_ticks[i]);

        g_snapshot.wheel_speed_mmps[i] = DeviceCenter_RoundToInt16(delta_mm / APP_CONTROL_DT_S);
        avg_delta_mm = (float)g_speed_window_sum_ticks[i] * DeviceCenter_GetWheelMmPerCount(i);
        avg_speed_mmps = DeviceCenter_RoundToInt16(
            avg_delta_mm / (APP_CONTROL_DT_S * (float)APP_WHEEL_SPEED_AVG_WINDOW));
        g_snapshot.wheel_speed_filtered_mmps[i] = avg_speed_mmps;
        g_snapshot.wheel_speed_display_mmps[i] = DeviceCenter_FilterSpeed(
            g_snapshot.wheel_speed_display_mmps[i],
            avg_speed_mmps);
        g_snapshot.wheel_distance_mm[i] += DeviceCenter_RoundToInt32(delta_mm);
    }

    g_speed_window_index++;
    if (g_speed_window_index >= APP_WHEEL_SPEED_AVG_WINDOW) {
        g_speed_window_index = 0U;
    }

    right_side_speed_mmps =
        ((float)g_snapshot.wheel_speed_filtered_mmps[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] +
         (float)g_snapshot.wheel_speed_filtered_mmps[APP_BOARD_WHEEL_REAR_RIGHT_INDEX]) * 0.5f;
    left_side_speed_mmps =
        ((float)g_snapshot.wheel_speed_filtered_mmps[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] +
         (float)g_snapshot.wheel_speed_filtered_mmps[APP_BOARD_WHEEL_REAR_LEFT_INDEX]) * 0.5f;
    g_snapshot.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX] = DeviceCenter_RoundToInt16(right_side_speed_mmps);
    g_snapshot.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] = DeviceCenter_RoundToInt16(left_side_speed_mmps);

    right_side_distance_mm =
        ((float)g_snapshot.wheel_distance_mm[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] +
         (float)g_snapshot.wheel_distance_mm[APP_BOARD_WHEEL_REAR_RIGHT_INDEX]) * 0.5f;
    left_side_distance_mm =
        ((float)g_snapshot.wheel_distance_mm[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] +
         (float)g_snapshot.wheel_distance_mm[APP_BOARD_WHEEL_REAR_LEFT_INDEX]) * 0.5f;
    g_snapshot.side_distance_mm[APP_BOARD_SIDE_RIGHT_INDEX] = DeviceCenter_RoundToInt32(right_side_distance_mm);
    g_snapshot.side_distance_mm[APP_BOARD_SIDE_LEFT_INDEX] = DeviceCenter_RoundToInt32(left_side_distance_mm);

    center_distance_mm = (left_side_distance_mm + right_side_distance_mm) * 0.5f;
    diff_distance_mm = right_side_distance_mm - left_side_distance_mm;

    g_snapshot.chassis_distance_mm = DeviceCenter_RoundToInt32(center_distance_mm);
    g_snapshot.encoder_heading_deg10 =
        DeviceCenter_RoundToInt16(diff_distance_mm * APP_CHASSIS_DEG10_PER_MM_DIFF);
}

void DeviceCenter_Init(void)
{
    uint8_t i;

    Buzzer_Init();
    HalChassis_Init();
    DeviceTrackSensor_Init();
    DeviceGyro_Init();

    g_snapshot.uptime_ms         = 0U;
    g_snapshot.imu_yaw_mdps      = 0;
    g_snapshot.imu_heading_deg10 = 0;
    g_snapshot.encoder_heading_deg10 = 0;
    g_snapshot.track_line_position = 0;
    g_snapshot.motor_left_duty = 0;
    g_snapshot.motor_right_duty = 0;
    g_snapshot.chassis_distance_mm = 0;
    g_snapshot.imu_online = 0U;
    g_snapshot.imu_parser_locked = 0U;
    g_snapshot.track_online = 0U;
    g_snapshot.motor_pwm_online = APP_BOARD_ENABLE_MOTOR_PWM_DIRECT;
    g_snapshot.odom_ready = 0U;
    g_snapshot.rgb_r             = 0U;
    g_snapshot.rgb_g             = 0U;
    g_snapshot.rgb_b             = 0U;

    for (i = 0U; i < APP_TRACK_SENSOR_COUNT; i++) {
        g_snapshot.gray_raw[i]    = 0U;
        g_snapshot.gray_binary[i] = 0U;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_snapshot.motor_wheel_duty[i] = 0;
        g_snapshot.wheel_speed_window_counts[i] = 0;
        g_snapshot.wheel_ticks[i] = 0;
        g_snapshot.wheel_speed_mmps[i] = 0;
        g_snapshot.wheel_speed_filtered_mmps[i] = 0;
        g_snapshot.wheel_speed_display_mmps[i] = 0;
        g_snapshot.wheel_distance_mm[i] = 0;
        g_prev_wheel_ticks[i] = 0;
    }

    for (i = 0U; i < APP_BOARD_SIDE_COUNT; i++) {
        g_snapshot.side_speed_mmps[i] = 0;
        g_snapshot.side_distance_mm[i] = 0;
    }

    g_odom_ready = 0U;
    g_odom_auto_zero_pending = 1U;
    g_odom_auto_zero_ms = 3500U;
    DeviceCenter_ResetSpeedObserver();
}

void DeviceCenter_Tick10ms(void)
{
    DeviceGyro_Snapshot_t gyro;
    DeviceTrackSensor_Snapshot_t track;
    HalChassis_MotorState_t motor;
    HalChassis_EncoderState_t encoder;
    uint8_t i;

    DeviceTrackSensor_Tick10ms();
    DeviceGyro_Tick10ms();

    DeviceGyro_GetSnapshot(&gyro);
    DeviceTrackSensor_GetSnapshot(&track);
    HalChassis_GetMotorState(&motor);
    HalChassis_GetEncoderState(&encoder);

    g_snapshot.uptime_ms = AppTick_GetMs();
    g_snapshot.imu_yaw_mdps = gyro.yaw_mdps;
    g_snapshot.imu_heading_deg10 = gyro.heading_deg10;
    g_snapshot.imu_online = gyro.online;
    g_snapshot.imu_parser_locked = gyro.parser_locked;
    g_snapshot.track_online = track.online;
    g_snapshot.track_line_position = track.line_position;
    g_snapshot.motor_left_duty = motor.left_duty;
    g_snapshot.motor_right_duty = motor.right_duty;
    g_snapshot.motor_pwm_online = motor.pwm_online;

    for (i = 0U; i < APP_TRACK_SENSOR_COUNT; i++) {
        g_snapshot.gray_raw[i] = track.raw[i];
        g_snapshot.gray_binary[i] = track.binary[i];
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_snapshot.motor_wheel_duty[i] = motor.wheel_duty[i];
        g_snapshot.wheel_ticks[i] = encoder.wheel_ticks[i];
    }

    DeviceCenter_UpdateOdometry(&encoder);

    if (g_odom_auto_zero_pending &&
        g_snapshot.odom_ready &&
        g_snapshot.imu_online &&
        g_snapshot.imu_parser_locked &&
        (g_snapshot.uptime_ms >= g_odom_auto_zero_ms)) {
        DeviceCenter_ResetOdometry();
        g_odom_auto_zero_pending = 0U;
    }
}

void DeviceCenter_GetSnapshot(DeviceCenter_Snapshot_t *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    *snapshot = g_snapshot;
}

void DeviceCenter_ResetOdometry(void)
{
    uint8_t i;

    g_odom_auto_zero_pending = 0U;
    DeviceCenter_ResetSpeedObserver();

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_snapshot.wheel_speed_mmps[i] = 0;
        g_snapshot.wheel_speed_window_counts[i] = 0;
        g_snapshot.wheel_speed_filtered_mmps[i] = 0;
        g_snapshot.wheel_speed_display_mmps[i] = 0;
        g_snapshot.wheel_distance_mm[i] = 0;
        g_prev_wheel_ticks[i] = g_snapshot.wheel_ticks[i];
    }

    for (i = 0U; i < APP_BOARD_SIDE_COUNT; i++) {
        g_snapshot.side_speed_mmps[i] = 0;
        g_snapshot.side_distance_mm[i] = 0;
    }

    g_snapshot.chassis_distance_mm = 0;
    g_snapshot.encoder_heading_deg10 = 0;
    DeviceGyro_ResetHeading();
    g_snapshot.imu_yaw_mdps = 0;
    g_snapshot.imu_heading_deg10 = 0;
    g_snapshot.odom_ready = g_odom_ready ? 1U : 0U;
}

void DeviceCenter_SetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    g_snapshot.rgb_r = red;
    g_snapshot.rgb_g = green;
    g_snapshot.rgb_b = blue;
    HalChassis_SetRgb(red, green, blue);
}

void DeviceCenter_PlayClick(void)
{
    Buzzer_Play(BUZZER_PATTERN_CLICK);
}

void DeviceCenter_PlayConfirm(void)
{
    Buzzer_Play(BUZZER_PATTERN_CONFIRM);
}

void DeviceCenter_PlayStartup(void)
{
    Buzzer_Play(BUZZER_PATTERN_STARTUP);
}
