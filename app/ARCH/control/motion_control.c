#include "ARCH/control/motion_control.h"

#include <string.h>

#include "ARCH/behavior/behavior_motion.h"
#include "ARCH/config/app_config.h"
#include "ARCH/control/pid.h"
#include "ARCH/device/device_center.h"
#include "ARCH/device/device_gyro.h"
#include "ARCH/device/device_track_sensor.h"
#include "ARCH/hal/hal_chassis.h"

static MotionControl_Status_t g_status;
static PidController_t g_speed_pid_wheel[APP_BOARD_WHEEL_COUNT];
static PidController_t g_distance_pid;
static PidController_t g_attitude_pid;
static PidController_t g_turn_pid;
static PidController_t g_line_pid;

typedef struct
{
    BehaviorMotion_Command_t last_command;
    uint8_t distance_ref_valid;
    uint8_t heading_ref_valid;
    uint8_t turn_exit_boost_ticks;
    uint8_t arc_exit_release_ticks;
    uint8_t arc_exit_stable_ticks;
    float distance_target_mm;
    int16_t heading_target_deg10;
} MotionControl_Runtime_t;

static MotionControl_Runtime_t g_runtime;

static float MotionControl_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float MotionControl_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int16_t MotionControl_AbsInt16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

static int32_t MotionControl_AbsInt32(int32_t value)
{
    return (value >= 0L) ? value : -value;
}

static int16_t MotionControl_ToInt16(float value)
{
    if (value > 32767.0f) {
        return 32767;
    }
    if (value < -32768.0f) {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static float MotionControl_ArcDiffMmpsFromRadius(float center_speed_mmps, float radius_mm)
{
    if ((radius_mm > -0.001f) && (radius_mm < 0.001f)) {
        return 0.0f;
    }

    return center_speed_mmps * (APP_CHASSIS_WHEEL_BASE_MM * 0.5f) / radius_mm;
}

static int16_t MotionControl_GetTurnAssistDuty(int16_t heading_error_deg10)
{
    int16_t heading_error_abs = MotionControl_AbsInt16(heading_error_deg10);

    if (heading_error_abs <= APP_TURN_ASSIST_MIN_ERR_DEG10) {
        return 0;
    }

    if (heading_error_abs >= APP_TURN_ASSIST_SOFT_ERR_DEG10) {
        return APP_TURN_ASSIST_MIN_WHEEL_DUTY;
    }

    {
        int32_t err_span = (int32_t)APP_TURN_ASSIST_SOFT_ERR_DEG10 -
            (int32_t)APP_TURN_ASSIST_MIN_ERR_DEG10;
        int32_t err_offset = (int32_t)heading_error_abs -
            (int32_t)APP_TURN_ASSIST_MIN_ERR_DEG10;
        int32_t duty_span = (int32_t)APP_TURN_ASSIST_MIN_WHEEL_DUTY -
            (int32_t)APP_TURN_ASSIST_SOFT_WHEEL_DUTY;

        return (int16_t)((int32_t)APP_TURN_ASSIST_SOFT_WHEEL_DUTY +
            ((duty_span * err_offset + (err_span / 2L)) / err_span));
    }
}

static int32_t MotionControl_ToInt32(float value)
{
    if (value > 2147483647.0f) {
        return 2147483647L;
    }
    if (value < -2147483648.0f) {
        return (-2147483647L - 1L);
    }
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static int16_t MotionControl_ClampToDuty(float value)
{
    if (value > (float)APP_MOTOR_DUTY_LIMIT) {
        value = (float)APP_MOTOR_DUTY_LIMIT;
    }
    if (value < (float)(-APP_MOTOR_DUTY_LIMIT)) {
        value = (float)(-APP_MOTOR_DUTY_LIMIT);
    }
    return (int16_t)value;
}

static void MotionControl_ApplyTurnAssistDuty(const BehaviorMotion_Command_t *command,
    const DeviceGyro_Snapshot_t *gyro,
    const float *wheel_speed_target,
    int16_t *wheel_duty)
{
    int16_t heading_error_deg10;
    int16_t assist_duty;
    int16_t left_min_duty;
    int16_t right_min_duty;

    if ((command == 0) || (gyro == 0) || (wheel_speed_target == 0) || (wheel_duty == 0)) {
        return;
    }

    if ((command->behavior != MOTION_BEHAVIOR_TURN) || !command->attitude_enable) {
        return;
    }

    heading_error_deg10 = (int16_t)(g_runtime.heading_target_deg10 - gyro->heading_deg10);
    assist_duty = MotionControl_GetTurnAssistDuty(heading_error_deg10);
    if (assist_duty <= 0) {
        return;
    }

    if (heading_error_deg10 > 0) {
        left_min_duty = (int16_t)(-assist_duty);
        right_min_duty = assist_duty;
    } else {
        left_min_duty = assist_duty;
        right_min_duty = (int16_t)(-assist_duty);
    }

    if (heading_error_deg10 > 0) {
        if (wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] > left_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = left_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] > left_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = left_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] < right_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = right_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] < right_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = right_min_duty;
        }
    } else {
        if (wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] < left_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = left_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] < left_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = left_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] > right_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = right_min_duty;
        }
        if (wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] > right_min_duty) {
            wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = right_min_duty;
        }
    }
}

static void MotionControl_ResetLoops(void)
{
    uint8_t i;

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        Pid_Reset(&g_speed_pid_wheel[i]);
    }
    Pid_Reset(&g_distance_pid);
    Pid_Reset(&g_attitude_pid);
    Pid_Reset(&g_turn_pid);
    Pid_Reset(&g_line_pid);
}

static void MotionControl_ClearStatus(void)
{
    memset(&g_status, 0, sizeof(g_status));
}

static void MotionControl_Apply(int16_t left_duty, int16_t right_duty)
{
    int16_t wheel_duty[APP_BOARD_WHEEL_COUNT];

    wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = right_duty;
    wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = right_duty;
    wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = left_duty;
    wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = left_duty;

    g_status.left_duty = left_duty;
    g_status.right_duty = right_duty;
    HalChassis_SetWheelDuty(wheel_duty);
}

static void MotionControl_ApplyWheelDuty(const int16_t *wheel_duty)
{
    if (wheel_duty == 0) {
        return;
    }

    g_status.right_duty = MotionControl_ToInt16(
        ((float)wheel_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] +
         (float)wheel_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX]) * 0.5f);
    g_status.left_duty = MotionControl_ToInt16(
        ((float)wheel_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] +
         (float)wheel_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX]) * 0.5f);
    HalChassis_SetWheelDuty(wheel_duty);
}

static void MotionControl_SetWheelSpeedTargets(float left_speed_target,
    float right_speed_target,
    float *wheel_speed_target)
{
    if (wheel_speed_target == 0) {
        return;
    }

    wheel_speed_target[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = right_speed_target;
    wheel_speed_target[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = right_speed_target;
    wheel_speed_target[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = left_speed_target;
    wheel_speed_target[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = left_speed_target;
}

static void MotionControl_UpdateWheelSpeedStatus(const DeviceCenter_Snapshot_t *device,
    const float *wheel_speed_target)
{
    uint8_t i;

    if ((device == 0) || (wheel_speed_target == 0)) {
        return;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_status.wheel_speed_target_mmps[i] = MotionControl_ToInt16(wheel_speed_target[i]);
        g_status.wheel_speed_measure_mmps[i] = device->wheel_speed_filtered_mmps[i];
    }
}

static void MotionControl_RunWheelSpeedLoop(const DeviceCenter_Snapshot_t *device,
    const float *wheel_speed_target,
    int16_t *wheel_duty_out)
{
    uint8_t i;

    if ((device == 0) || (wheel_speed_target == 0) || (wheel_duty_out == 0)) {
        return;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        float feedforward = wheel_speed_target[i] * APP_MOTOR_SPEED_FEEDFORWARD_GAIN;
        float pid_output = 0.0f;
        float total_output;

        if (device->odom_ready) {
            pid_output = Pid_Update(&g_speed_pid_wheel[i],
                wheel_speed_target[i],
                (float)device->wheel_speed_filtered_mmps[i],
                APP_CONTROL_DT_S);
        } else {
            Pid_Reset(&g_speed_pid_wheel[i]);
        }

        g_status.wheel_speed_output_duty[i] = MotionControl_ToInt16(pid_output);
        total_output = feedforward + pid_output;
        wheel_duty_out[i] = MotionControl_ClampToDuty(total_output);
    }
}

static void MotionControl_RunTurnDutyPath(const float *wheel_speed_target,
    int16_t *wheel_duty_out)
{
    uint8_t i;

    if ((wheel_speed_target == 0) || (wheel_duty_out == 0)) {
        return;
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        float duty_ff = wheel_speed_target[i] * APP_MOTOR_SPEED_FEEDFORWARD_GAIN;

        wheel_duty_out[i] = MotionControl_ClampToDuty(duty_ff);
        g_status.wheel_speed_output_duty[i] = wheel_duty_out[i];
    }
}

static void MotionControl_RunTurnHeadingDutyPath(const BehaviorMotion_Command_t *command,
    const DeviceGyro_Snapshot_t *gyro,
    int16_t *wheel_duty_out)
{
    int16_t heading_error_deg10;
    int16_t heading_error_abs;
    int16_t base_duty;
    int16_t turn_duty_abs;
    float duty_value;
    uint8_t i;

    if ((command == 0) || (gyro == 0) || (wheel_duty_out == 0)) {
        return;
    }

    heading_error_deg10 = (int16_t)(g_runtime.heading_target_deg10 - gyro->heading_deg10);
    heading_error_abs = MotionControl_AbsInt16(heading_error_deg10);
    base_duty = MotionControl_GetTurnAssistDuty(heading_error_deg10);

    if ((heading_error_abs <= APP_TURN_ASSIST_MIN_ERR_DEG10) && (base_duty <= 0)) {
        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            wheel_duty_out[i] = 0;
            g_status.wheel_speed_output_duty[i] = 0;
        }
        g_status.turn_output_diff_mmps = 0;
        return;
    }

    duty_value = ((float)heading_error_abs * APP_TURN_DIRECT_DUTY_KP) +
        (float)base_duty -
        ((float)MotionControl_AbsInt16(gyro->yaw_mdps) * APP_TURN_DIRECT_YAW_DAMP);

    if (duty_value < (float)base_duty) {
        duty_value = (float)base_duty;
    }
    if (duty_value > (float)APP_TURN_DIRECT_MAX_WHEEL_DUTY) {
        duty_value = (float)APP_TURN_DIRECT_MAX_WHEEL_DUTY;
    }

    turn_duty_abs = MotionControl_ClampToDuty(duty_value);
    if (heading_error_deg10 > 0) {
        wheel_duty_out[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = (int16_t)(-turn_duty_abs);
        wheel_duty_out[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = (int16_t)(-turn_duty_abs);
        wheel_duty_out[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = turn_duty_abs;
        wheel_duty_out[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = turn_duty_abs;
        g_status.turn_output_diff_mmps = turn_duty_abs;
    } else {
        wheel_duty_out[APP_BOARD_WHEEL_REAR_LEFT_INDEX] = turn_duty_abs;
        wheel_duty_out[APP_BOARD_WHEEL_FRONT_LEFT_INDEX] = turn_duty_abs;
        wheel_duty_out[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] = (int16_t)(-turn_duty_abs);
        wheel_duty_out[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX] = (int16_t)(-turn_duty_abs);
        g_status.turn_output_diff_mmps = (int16_t)(-turn_duty_abs);
    }

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        g_status.wheel_speed_output_duty[i] = wheel_duty_out[i];
    }
}

static uint8_t MotionControl_CommandEquals(const BehaviorMotion_Command_t *lhs,
    const BehaviorMotion_Command_t *rhs)
{
    if ((lhs == 0) || (rhs == 0)) {
        return 0U;
    }

    return (uint8_t)(
        (lhs->behavior == rhs->behavior) &&
        (lhs->speed_target_mmps == rhs->speed_target_mmps) &&
        (lhs->distance_target_mm == rhs->distance_target_mm) &&
        (lhs->heading_target_deg10 == rhs->heading_target_deg10) &&
        (lhs->yaw_rate_target_mdps == rhs->yaw_rate_target_mdps) &&
        (lhs->arc_radius_mm == rhs->arc_radius_mm) &&
        (lhs->line_target == rhs->line_target) &&
        (lhs->open_loop_left_duty == rhs->open_loop_left_duty) &&
        (lhs->open_loop_right_duty == rhs->open_loop_right_duty) &&
        (lhs->track_target == rhs->track_target) &&
        (lhs->speed_enable == rhs->speed_enable) &&
        (lhs->distance_enable == rhs->distance_enable) &&
        (lhs->attitude_enable == rhs->attitude_enable) &&
        (lhs->turn_enable == rhs->turn_enable) &&
        (lhs->line_enable == rhs->line_enable) &&
        (lhs->open_loop_enable == rhs->open_loop_enable) &&
        (lhs->distance_relative == rhs->distance_relative) &&
        (lhs->heading_lock_on_entry == rhs->heading_lock_on_entry));
}

static void MotionControl_ResetReferences(void)
{
    g_runtime.distance_ref_valid = 0U;
    g_runtime.heading_ref_valid = 0U;
    g_runtime.distance_target_mm = 0.0f;
    g_runtime.heading_target_deg10 = 0;
}

static void MotionControl_LatchReferences(const BehaviorMotion_Command_t *command,
    const DeviceGyro_Snapshot_t *gyro,
    const DeviceCenter_Snapshot_t *device,
    uint8_t command_changed)
{
    if ((command == 0) || (gyro == 0) || (device == 0)) {
        return;
    }

    if (!command->distance_enable) {
        g_runtime.distance_ref_valid = 0U;
        Pid_Reset(&g_distance_pid);
    } else if (command_changed || !g_runtime.distance_ref_valid) {
        if (command->distance_relative) {
            g_runtime.distance_target_mm = (float)device->chassis_distance_mm +
                (float)command->distance_target_mm;
        } else {
            g_runtime.distance_target_mm = (float)command->distance_target_mm;
        }
        g_runtime.distance_ref_valid = 1U;
        Pid_Reset(&g_distance_pid);
    }

    if (!command->attitude_enable) {
        g_runtime.heading_ref_valid = 0U;
        Pid_Reset(&g_attitude_pid);
    } else if (command_changed || !g_runtime.heading_ref_valid) {
        g_runtime.heading_target_deg10 = command->heading_lock_on_entry ?
            gyro->heading_deg10 : command->heading_target_deg10;
        g_runtime.heading_ref_valid = 1U;
        Pid_Reset(&g_attitude_pid);
    }
}

void MotionControl_Init(void)
{
    MotionControl_ClearStatus();
    memset(&g_runtime, 0, sizeof(g_runtime));

    {
        uint8_t i;

        for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
            Pid_Init(&g_speed_pid_wheel[i],
                APP_WHEEL_SPEED_PID_KP,
                APP_WHEEL_SPEED_PID_KI,
                APP_WHEEL_SPEED_PID_KD,
                APP_WHEEL_SPEED_PID_OUT_MIN,
                APP_WHEEL_SPEED_PID_OUT_MAX);
        }
    }
    Pid_Init(&g_distance_pid, APP_DISTANCE_PID_KP, APP_DISTANCE_PID_KI, APP_DISTANCE_PID_KD,
        APP_DISTANCE_PID_OUT_MIN, APP_DISTANCE_PID_OUT_MAX);
    Pid_Init(&g_attitude_pid, APP_ATTITUDE_PID_KP, APP_ATTITUDE_PID_KI, APP_ATTITUDE_PID_KD,
        APP_ATTITUDE_PID_OUT_MIN, APP_ATTITUDE_PID_OUT_MAX);
    Pid_Init(&g_turn_pid, APP_TURN_PID_KP, APP_TURN_PID_KI, APP_TURN_PID_KD,
        APP_TURN_PID_OUT_MIN, APP_TURN_PID_OUT_MAX);
    Pid_Init(&g_line_pid, APP_LINE_PID_KP, APP_LINE_PID_KI, APP_LINE_PID_KD,
        APP_LINE_PID_OUT_MIN, APP_LINE_PID_OUT_MAX);

    MotionControl_ResetLoops();
    MotionControl_ResetReferences();
    MotionControl_Apply(0, 0);
}

void MotionControl_Tick10ms(void)
{
    BehaviorMotion_Command_t command;
    DeviceCenter_Snapshot_t device;
    DeviceGyro_Snapshot_t gyro;
    DeviceTrackSensor_Snapshot_t track;
    uint8_t command_changed;
    float center_speed_ref = 0.0f;
    float desired_yaw_rate_mdps = 0.0f;
    float distance_speed_ref = 0.0f;
    float attitude_yaw_ref = 0.0f;
    float line_yaw_ref = 0.0f;
    float turn_speed_diff = 0.0f;
    float turn_speed_trim = 0.0f;
    float left_speed_target;
    float right_speed_target;
    float wheel_speed_target[APP_BOARD_WHEEL_COUNT] = {0.0f};
    int16_t wheel_duty[APP_BOARD_WHEEL_COUNT] = {0};
    uint8_t i;
    uint8_t arc_exit_release_active;
    uint8_t arc_exit_release_done;

    BehaviorMotion_GetCommand(&command);
    DeviceCenter_GetSnapshot(&device);
    DeviceGyro_GetSnapshot(&gyro);
    DeviceTrackSensor_GetSnapshot(&track);

    command_changed = (uint8_t)!MotionControl_CommandEquals(&command, &g_runtime.last_command);
    if (command_changed) {
        if ((g_runtime.last_command.behavior == MOTION_BEHAVIOR_TURN) &&
            (command.behavior == MOTION_BEHAVIOR_STRAIGHT)) {
            g_runtime.turn_exit_boost_ticks = AppConfig_GetTurnExitBoostTicks();
        } else {
            g_runtime.turn_exit_boost_ticks = 0U;
        }

        if ((g_runtime.last_command.behavior == MOTION_BEHAVIOR_ARC) &&
            (command.behavior == MOTION_BEHAVIOR_STRAIGHT)) {
            g_runtime.arc_exit_release_ticks = APP_ARC_EXIT_RELEASE_TICKS;
            g_runtime.arc_exit_stable_ticks = 0U;
        } else {
            g_runtime.arc_exit_release_ticks = 0U;
            g_runtime.arc_exit_stable_ticks = 0U;
        }
        MotionControl_ResetLoops();
    }

    MotionControl_LatchReferences(&command, &gyro, &device, command_changed);

    g_status.distance_loop_ready = 0U;
    g_status.attitude_loop_ready = 0U;
    g_status.turn_loop_ready = 0U;
    g_status.line_loop_ready = 0U;
    g_status.speed_loop_ready = device.odom_ready;
    g_status.heading_measure_deg10 = gyro.heading_deg10;
    g_status.yaw_rate_measure_mdps = gyro.yaw_mdps;
    g_status.line_measure = track.line_position;
    g_status.distance_measure_mm = device.chassis_distance_mm;
    g_status.speed_measure_left_mmps = device.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX];
    g_status.speed_measure_right_mmps = device.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX];

    if (command.behavior == MOTION_BEHAVIOR_IDLE) {
        MotionControl_ResetLoops();
        MotionControl_ResetReferences();
        memset(&g_runtime.last_command, 0, sizeof(g_runtime.last_command));
        MotionControl_ClearStatus();
        MotionControl_Apply(0, 0);
        return;
    }

    if (command.open_loop_enable && (command.behavior == MOTION_BEHAVIOR_OPEN_LOOP)) {
        MotionControl_ResetLoops();
        MotionControl_ResetReferences();
        g_status.left_duty = MotionControl_ClampToDuty((float)command.open_loop_left_duty);
        g_status.right_duty = MotionControl_ClampToDuty((float)command.open_loop_right_duty);
        MotionControl_Apply(g_status.left_duty, g_status.right_duty);
        g_runtime.last_command = command;
        return;
    }

    if (command.distance_enable && g_runtime.distance_ref_valid) {
        int32_t distance_error_abs_mm;

        distance_speed_ref = Pid_Update(&g_distance_pid,
            g_runtime.distance_target_mm,
            (float)device.chassis_distance_mm,
            APP_CONTROL_DT_S);
        g_status.distance_loop_ready = 1U;
        g_status.distance_target_mm = MotionControl_ToInt32(g_runtime.distance_target_mm);
        g_status.distance_output_speed_mmps = MotionControl_ToInt16(distance_speed_ref);
        g_status.distance_error_mm = MotionControl_ToInt16(
            g_runtime.distance_target_mm - (float)device.chassis_distance_mm);
        distance_error_abs_mm = MotionControl_AbsInt32(
            g_status.distance_target_mm - g_status.distance_measure_mm);

        if (command.speed_enable) {
            float speed_limit = MotionControl_AbsFloat((float)command.speed_target_mmps);

            if (distance_error_abs_mm <= APP_DISTANCE_STOP_FREEZE_ERR_MM) {
                speed_limit = 0.0f;
                distance_speed_ref = 0.0f;
                Pid_Reset(&g_distance_pid);
            } else if (distance_error_abs_mm < APP_DISTANCE_APPROACH_BRAKE_MM) {
                float ratio = (float)(distance_error_abs_mm - APP_DISTANCE_STOP_FREEZE_ERR_MM) /
                    (float)(APP_DISTANCE_APPROACH_BRAKE_MM - APP_DISTANCE_STOP_FREEZE_ERR_MM);
                float near_speed_limit = (float)APP_DISTANCE_APPROACH_MIN_SPEED_MMPS +
                    ((speed_limit - (float)APP_DISTANCE_APPROACH_MIN_SPEED_MMPS) * ratio);

                if (near_speed_limit < (float)APP_DISTANCE_APPROACH_MIN_SPEED_MMPS) {
                    near_speed_limit = (float)APP_DISTANCE_APPROACH_MIN_SPEED_MMPS;
                }
                if (near_speed_limit < speed_limit) {
                    speed_limit = near_speed_limit;
                }
            }

            center_speed_ref = MotionControl_ClampFloat(distance_speed_ref, -speed_limit, speed_limit);
        } else {
            center_speed_ref = distance_speed_ref;
        }
    } else {
        g_status.distance_target_mm = g_status.distance_measure_mm;
        g_status.distance_output_speed_mmps = 0;
        g_status.distance_error_mm = 0;
        if (command.speed_enable) {
            center_speed_ref = (float)command.speed_target_mmps;
        } else {
            Pid_Reset(&g_distance_pid);
        }
    }

    if ((command.behavior == MOTION_BEHAVIOR_STRAIGHT) &&
        command.speed_enable &&
        (g_runtime.turn_exit_boost_ticks > 0U)) {
        float boost_speed = (float)AppConfig_GetTurnExitBoostMmps();

        if (center_speed_ref < boost_speed) {
            center_speed_ref = boost_speed;
        }
        g_runtime.turn_exit_boost_ticks--;
    }

    arc_exit_release_active = (uint8_t)((command.behavior == MOTION_BEHAVIOR_STRAIGHT) &&
        (g_runtime.arc_exit_release_ticks > 0U));
    arc_exit_release_done = 0U;

    if (command.attitude_enable && gyro.online && g_runtime.heading_ref_valid) {
        int16_t heading_error_now_deg10 = (int16_t)(g_runtime.heading_target_deg10 - gyro.heading_deg10);
        uint8_t arc_trim_active = 1U;

        if ((command.behavior == MOTION_BEHAVIOR_ARC) &&
            (MotionControl_AbsInt16(heading_error_now_deg10) > APP_ARC_HEADING_TRIM_ENTRY_DEG10)) {
            arc_trim_active = 0U;
        }

        if (arc_exit_release_active) {
            g_runtime.heading_target_deg10 = gyro.heading_deg10;
            attitude_yaw_ref = 0.0f;
            Pid_Reset(&g_attitude_pid);
        } else if (arc_trim_active) {
            attitude_yaw_ref = Pid_Update(&g_attitude_pid,
                (float)g_runtime.heading_target_deg10,
                (float)gyro.heading_deg10,
                APP_CONTROL_DT_S);
            if (command.behavior == MOTION_BEHAVIOR_ARC) {
                attitude_yaw_ref = MotionControl_ClampFloat(attitude_yaw_ref,
                    -(float)APP_ARC_HEADING_TRIM_MAX_YAW_MDPS,
                    (float)APP_ARC_HEADING_TRIM_MAX_YAW_MDPS);
            }
        } else {
            attitude_yaw_ref = 0.0f;
            Pid_Reset(&g_attitude_pid);
        }

        if ((command.behavior == MOTION_BEHAVIOR_STRAIGHT) &&
            (MotionControl_AbsInt16(heading_error_now_deg10) <=
                APP_STRAIGHT_HEADING_DEADBAND_DEG10)) {
            attitude_yaw_ref = 0.0f;
        }

        desired_yaw_rate_mdps += attitude_yaw_ref;
        g_status.attitude_loop_ready = (uint8_t)(attitude_yaw_ref != 0.0f);
        g_status.heading_target_deg10 = g_runtime.heading_target_deg10;
        g_status.attitude_output_yaw_mdps = MotionControl_ToInt16(attitude_yaw_ref);
        g_status.heading_error_deg10 = heading_error_now_deg10;
    } else {
        g_status.heading_target_deg10 = gyro.heading_deg10;
        g_status.attitude_output_yaw_mdps = 0;
        g_status.heading_error_deg10 = 0;
        Pid_Reset(&g_attitude_pid);
    }

    if (command.turn_enable) {
        desired_yaw_rate_mdps += (float)command.yaw_rate_target_mdps;
    }

    if (command.line_enable && track.online) {
        line_yaw_ref = Pid_Update(&g_line_pid,
            (float)command.line_target,
            (float)track.line_position,
            APP_CONTROL_DT_S);
        desired_yaw_rate_mdps += line_yaw_ref;
        g_status.line_loop_ready = 1U;
        g_status.line_target = command.line_target;
        g_status.line_output_yaw_mdps = MotionControl_ToInt16(line_yaw_ref);
        g_status.line_error = (int16_t)(command.line_target - track.line_position);
    } else {
        g_status.line_target = 0;
        g_status.line_output_yaw_mdps = 0;
        g_status.line_error = 0;
        Pid_Reset(&g_line_pid);
    }

    if ((command.behavior == MOTION_BEHAVIOR_ARC) && command.turn_enable) {
        if ((command.arc_radius_mm > -1) && (command.arc_radius_mm < 1)) {
            g_status.yaw_rate_target_mdps = 0;
            g_status.yaw_rate_error_mdps = 0;
            turn_speed_diff = 0.0f;
        } else {
            float arc_turn_ff = MotionControl_ArcDiffMmpsFromRadius(center_speed_ref,
                (float)command.arc_radius_mm);
            float arc_yaw_target_mdps = (center_speed_ref * 180000.0f) /
                (APP_PI * (float)command.arc_radius_mm);
            desired_yaw_rate_mdps += arc_yaw_target_mdps;

            if (MotionControl_AbsFloat(desired_yaw_rate_mdps) <= (float)APP_ARC_YAW_DEADBAND_MDPS) {
                desired_yaw_rate_mdps = 0.0f;
            }

            if (gyro.online) {
                turn_speed_trim = Pid_Update(&g_turn_pid,
                    desired_yaw_rate_mdps,
                    (float)gyro.yaw_mdps,
                    APP_CONTROL_DT_S);
                turn_speed_trim = MotionControl_ClampFloat(turn_speed_trim,
                    -(float)APP_ARC_RADIUS_TRIM_MAX_DIFF_MMPS,
                    (float)APP_ARC_RADIUS_TRIM_MAX_DIFF_MMPS);
            } else {
                turn_speed_trim = 0.0f;
                Pid_Reset(&g_turn_pid);
            }

            turn_speed_diff = arc_turn_ff + turn_speed_trim;
            g_status.yaw_rate_target_mdps = MotionControl_ToInt16(desired_yaw_rate_mdps);
            g_status.yaw_rate_error_mdps = MotionControl_ToInt16(
                desired_yaw_rate_mdps - (float)gyro.yaw_mdps);
        }
        g_status.turn_loop_ready = 1U;
        g_status.turn_output_diff_mmps = MotionControl_ToInt16(turn_speed_diff);
    } else if ((command.turn_enable || command.attitude_enable || command.line_enable) && gyro.online) {
        int16_t heading_error_deg10_now = (int16_t)(g_runtime.heading_target_deg10 - gyro.heading_deg10);

        if (arc_exit_release_active) {
            desired_yaw_rate_mdps = 0.0f;
        } else if ((command.behavior == MOTION_BEHAVIOR_STRAIGHT) &&
            (MotionControl_AbsFloat(desired_yaw_rate_mdps) <= (float)APP_STRAIGHT_YAW_DEADBAND_MDPS)) {
            desired_yaw_rate_mdps = 0.0f;
        }

        g_status.yaw_rate_target_mdps = MotionControl_ToInt16(desired_yaw_rate_mdps);

        if (arc_exit_release_active) {
            turn_speed_diff = 0.0f;
            Pid_Reset(&g_turn_pid);
        } else {
            turn_speed_diff = Pid_Update(&g_turn_pid,
                desired_yaw_rate_mdps,
                (float)gyro.yaw_mdps,
                APP_CONTROL_DT_S);
        }

        if ((command.behavior == MOTION_BEHAVIOR_TURN) &&
            command.attitude_enable &&
            (((heading_error_deg10_now > 0) && (turn_speed_diff < 0.0f)) ||
             ((heading_error_deg10_now < 0) && (turn_speed_diff > 0.0f)))) {
            turn_speed_diff = 0.0f;
        }

        if ((command.behavior == MOTION_BEHAVIOR_STRAIGHT) &&
            (MotionControl_AbsFloat(turn_speed_diff) <= (float)APP_STRAIGHT_DIFF_DEADBAND_MMPS)) {
            turn_speed_diff = 0.0f;
        }

        g_status.turn_loop_ready = 1U;
        g_status.turn_output_diff_mmps = MotionControl_ToInt16(turn_speed_diff);
        g_status.yaw_rate_error_mdps = MotionControl_ToInt16(
            desired_yaw_rate_mdps - (float)gyro.yaw_mdps);
    } else {
        g_status.turn_output_diff_mmps = 0;
        g_status.yaw_rate_error_mdps = 0;
        g_status.yaw_rate_target_mdps = MotionControl_ToInt16(desired_yaw_rate_mdps);
        Pid_Reset(&g_turn_pid);
    }

    left_speed_target = center_speed_ref - turn_speed_diff;
    right_speed_target = center_speed_ref + turn_speed_diff;

    if (command.behavior == MOTION_BEHAVIOR_STRAIGHT) {
        float side_bias_mmps = AppConfig_GetStraightSideBiasMmps();

        if (!arc_exit_release_active) {
            left_speed_target += side_bias_mmps;
            right_speed_target -= side_bias_mmps;
        }
    }

    MotionControl_SetWheelSpeedTargets(left_speed_target, right_speed_target, wheel_speed_target);
    MotionControl_UpdateWheelSpeedStatus(&device, wheel_speed_target);

    g_status.speed_target_left_mmps = MotionControl_ToInt16(left_speed_target);
    g_status.speed_target_right_mmps = MotionControl_ToInt16(right_speed_target);
    if ((command.behavior == MOTION_BEHAVIOR_TURN) && command.attitude_enable) {
        MotionControl_RunTurnHeadingDutyPath(&command, &gyro, wheel_duty);
    } else {
        MotionControl_RunWheelSpeedLoop(&device, wheel_speed_target, wheel_duty);
    }

    g_status.speed_output_left_duty = MotionControl_ToInt16(
        ((float)g_status.wheel_speed_output_duty[APP_BOARD_WHEEL_REAR_LEFT_INDEX] +
         (float)g_status.wheel_speed_output_duty[APP_BOARD_WHEEL_FRONT_LEFT_INDEX]) * 0.5f);
    g_status.speed_output_right_duty = MotionControl_ToInt16(
        ((float)g_status.wheel_speed_output_duty[APP_BOARD_WHEEL_REAR_RIGHT_INDEX] +
         (float)g_status.wheel_speed_output_duty[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX]) * 0.5f);

    for (i = 0U; i < APP_BOARD_WHEEL_COUNT; i++) {
        if (!device.odom_ready) {
            g_status.wheel_speed_output_duty[i] = 0;
        }
    }

    MotionControl_ApplyWheelDuty(wheel_duty);

    if (arc_exit_release_active) {
        if (MotionControl_AbsInt16(gyro.yaw_mdps) <= APP_ARC_EXIT_RELEASE_YAW_MDPS) {
            if (g_runtime.arc_exit_stable_ticks < 255U) {
                g_runtime.arc_exit_stable_ticks++;
            }
        } else {
            g_runtime.arc_exit_stable_ticks = 0U;
        }

        if (g_runtime.arc_exit_stable_ticks >= APP_ARC_EXIT_RELEASE_HOLD_TICKS) {
            arc_exit_release_done = 1U;
        }

        if (g_runtime.arc_exit_release_ticks > 0U) {
            g_runtime.arc_exit_release_ticks--;
        }

        if ((g_runtime.arc_exit_release_ticks == 0U) || arc_exit_release_done) {
            g_runtime.arc_exit_release_ticks = 0U;
            g_runtime.arc_exit_stable_ticks = 0U;
        }
    }

    g_runtime.last_command = command;
}

void MotionControl_GetStatus(MotionControl_Status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_status;
}
