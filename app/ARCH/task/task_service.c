#include "ARCH/task/task_service.h"

#include "AppTick.h"
#include "ARCH/behavior/behavior_motion.h"
#include "ARCH/config/app_config.h"
#include "ARCH/device/device_center.h"
#include "ARCH/device/device_gyro.h"

typedef struct
{
    TaskAction_e data[APP_TASK_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} TaskQueue_t;

static TaskQueue_t g_queue;
static TaskService_Status_t g_status;
static uint32_t g_phase_start_ms = 0U;
static int16_t g_turn_target_heading_deg10 = 0;
static uint8_t g_turn_settle_count = 0U;
static uint8_t g_turn_stall_count = 0U;
static uint8_t g_distance_settle_count = 0U;
static uint8_t g_arc_settle_count = 0U;
static int32_t g_phase_origin_distance_mm = 0;
static int16_t g_phase_origin_heading_deg10 = 0;
static int16_t g_phase_origin_encoder_heading_deg10 = 0;
static int16_t g_turn_last_heading_error_deg10 = 0;

typedef struct
{
    uint8_t enabled;
    int16_t speed_mmps;
    int16_t radius_mm;
    int16_t angle_deg10;
} TaskService_ArcSegment_t;

typedef struct
{
    TaskService_ArcSegment_t entry;
    TaskService_ArcSegment_t main;
} TaskService_ArcTemplate_t;

typedef enum
{
    TASK_SERVICE_ARC_ANCHOR_CENTER_LEFT = 0,
    TASK_SERVICE_ARC_ANCHOR_CENTER_RIGHT,
    TASK_SERVICE_ARC_ANCHOR_FRONT_LEFT,
    TASK_SERVICE_ARC_ANCHOR_FRONT_RIGHT,
    TASK_SERVICE_ARC_ANCHOR_REAR_LEFT,
    TASK_SERVICE_ARC_ANCHOR_REAR_RIGHT,
} TaskService_ArcAnchor_e;

static void TaskService_FinishCurrent(void)
{
    g_status.last_completed_action = g_status.current_action;
    g_status.current_action        = TASK_ACTION_NONE;
    g_status.stage                 = TASK_STAGE_COMPLETE;
    g_status.busy                  = 0U;
    g_status.progress              = 100U;
    g_status.phase_index           = 0U;
    BehaviorMotion_Stop();
}

static void TaskService_ResetQueue(void)
{
    g_queue.head = 0U;
    g_queue.tail = 0U;
    g_queue.size = 0U;
    g_status.queue_depth = 0U;
}

static void TaskService_Start(TaskAction_e action)
{
    g_status.current_action = action;
    g_status.stage          = TASK_STAGE_PREPARE;
    g_status.busy           = 1U;
    g_status.progress       = 0U;
    g_status.phase_index    = 0U;
    g_phase_start_ms        = AppTick_GetMs();
    g_turn_target_heading_deg10 = 0;
    g_turn_settle_count = 0U;
    g_turn_stall_count = 0U;
    g_distance_settle_count = 0U;
    g_arc_settle_count = 0U;
    g_phase_origin_distance_mm = 0;
    g_phase_origin_heading_deg10 = 0;
    g_phase_origin_encoder_heading_deg10 = 0;
    g_turn_last_heading_error_deg10 = 0;
}

static uint8_t TaskService_Dequeue(TaskAction_e *action)
{
    if ((action == 0) || (g_queue.size == 0U)) {
        return 0U;
    }

    *action = g_queue.data[g_queue.head];
    g_queue.head = (uint8_t)((g_queue.head + 1U) % APP_TASK_QUEUE_DEPTH);
    g_queue.size--;
    g_status.queue_depth = g_queue.size;
    return 1U;
}

static uint8_t TaskService_IsQueued(TaskAction_e action)
{
    uint8_t index;
    uint8_t count;

    if (action == TASK_ACTION_NONE) {
        return 0U;
    }

    if (g_status.busy && (g_status.current_action == action)) {
        return 1U;
    }

    index = g_queue.head;
    count = g_queue.size;
    while (count-- > 0U) {
        if (g_queue.data[index] == action) {
            return 1U;
        }
        index = (uint8_t)((index + 1U) % APP_TASK_QUEUE_DEPTH);
    }

    return 0U;
}

static int16_t TaskService_AbsInt16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

static int32_t TaskService_AbsInt32(int32_t value)
{
    return (value >= 0L) ? value : -value;
}

static int32_t TaskService_ArcLengthMmByRadiusAngle(int16_t radius_mm, int16_t delta_heading_deg10)
{
    float angle_deg;
    float arc_length_mm;
    float radius_abs_mm = (float)TaskService_AbsInt16(radius_mm);

    angle_deg = ((float)TaskService_AbsInt16(delta_heading_deg10)) * 0.1f;
    arc_length_mm = (APP_PI * radius_abs_mm * angle_deg) / 180.0f;
    return (int32_t)(arc_length_mm + 0.5f);
}

static int16_t TaskService_ArcSignedAngleByRadius(int16_t radius_mm, int16_t angle_deg10)
{
    int16_t angle_abs = TaskService_AbsInt16(angle_deg10);

    return (radius_mm >= 0) ? angle_abs : (int16_t)(-angle_abs);
}

static void TaskService_ClearArcTemplate(TaskService_ArcTemplate_t *arc_template)
{
    if (arc_template == 0) {
        return;
    }

    arc_template->entry.enabled = 0U;
    arc_template->entry.speed_mmps = 0;
    arc_template->entry.radius_mm = 0;
    arc_template->entry.angle_deg10 = 0;
    arc_template->main.enabled = 0U;
    arc_template->main.speed_mmps = 0;
    arc_template->main.radius_mm = 0;
    arc_template->main.angle_deg10 = 0;
}

static void TaskService_BuildArcNoEntry(TaskService_ArcTemplate_t *arc_template,
    int16_t speed_mmps,
    int16_t radius_mm,
    int16_t angle_deg10)
{
    if (arc_template == 0) {
        return;
    }

    TaskService_ClearArcTemplate(arc_template);
    arc_template->main.enabled = 1U;
    arc_template->main.speed_mmps = speed_mmps;
    arc_template->main.radius_mm = radius_mm;
    arc_template->main.angle_deg10 = angle_deg10;
}

static void TaskService_BuildArcWithEntry(TaskService_ArcTemplate_t *arc_template,
    int16_t entry_speed_mmps,
    int16_t entry_radius_mm,
    int16_t entry_angle_deg10,
    int16_t main_speed_mmps,
    int16_t main_radius_mm,
    int16_t main_angle_deg10)
{
    if (arc_template == 0) {
        return;
    }

    TaskService_BuildArcNoEntry(arc_template, main_speed_mmps, main_radius_mm, main_angle_deg10);
    arc_template->entry.enabled = 1U;
    arc_template->entry.speed_mmps = entry_speed_mmps;
    arc_template->entry.radius_mm = entry_radius_mm;
    arc_template->entry.angle_deg10 = entry_angle_deg10;
}

static void TaskService_BuildArcByAnchor(TaskService_ArcTemplate_t *arc_template,
    TaskService_ArcAnchor_e anchor,
    int16_t main_speed_mmps,
    int16_t main_radius_mm,
    int16_t main_angle_deg10)
{
    int16_t radius_abs_mm;
    int16_t angle_abs_deg10;
    int16_t entry_radius_abs_mm;
    int16_t entry_angle_abs_deg10;
    int16_t entry_speed_mmps;

    if (arc_template == 0) {
        return;
    }

    radius_abs_mm = TaskService_AbsInt16(main_radius_mm);
    angle_abs_deg10 = TaskService_AbsInt16(main_angle_deg10);
    if (radius_abs_mm <= 0) {
        radius_abs_mm = 10;
    }
    if (angle_abs_deg10 <= 0) {
        angle_abs_deg10 = 50;
    }

    entry_radius_abs_mm = (int16_t)((((int32_t)radius_abs_mm *
        (int32_t)APP_Q2_ENTRY_RADIUS_SCALE_X100) + 50L) / 100L);
    if (entry_radius_abs_mm < 80) {
        entry_radius_abs_mm = 80;
    }

    entry_angle_abs_deg10 = APP_Q2_ENTRY_ARC_ANGLE_DEG10;
    if (anchor == TASK_SERVICE_ARC_ANCHOR_REAR_LEFT ||
        anchor == TASK_SERVICE_ARC_ANCHOR_REAR_RIGHT) {
        entry_angle_abs_deg10 = (int16_t)(APP_Q2_ENTRY_ARC_ANGLE_DEG10 / 2);
    }
    if (entry_angle_abs_deg10 <= 0) {
        entry_angle_abs_deg10 = 50;
    }

    entry_speed_mmps = (int16_t)(main_speed_mmps - APP_Q2_ENTRY_SPEED_OFFSET_MMPS);
    if (entry_speed_mmps < 100) {
        entry_speed_mmps = 100;
    }

    switch (anchor) {
    case TASK_SERVICE_ARC_ANCHOR_CENTER_LEFT:
        TaskService_BuildArcNoEntry(arc_template,
            main_speed_mmps,
            radius_abs_mm,
            angle_abs_deg10);
        break;

    case TASK_SERVICE_ARC_ANCHOR_CENTER_RIGHT:
        TaskService_BuildArcNoEntry(arc_template,
            main_speed_mmps,
            (int16_t)(-radius_abs_mm),
            (int16_t)(-angle_abs_deg10));
        break;

    case TASK_SERVICE_ARC_ANCHOR_FRONT_LEFT:
        TaskService_BuildArcWithEntry(arc_template,
            entry_speed_mmps,
            entry_radius_abs_mm,
            entry_angle_abs_deg10,
            main_speed_mmps,
            main_radius_mm,
            main_angle_deg10);
        break;

    case TASK_SERVICE_ARC_ANCHOR_FRONT_RIGHT:
        TaskService_BuildArcWithEntry(arc_template,
            entry_speed_mmps,
            (int16_t)(-entry_radius_abs_mm),
            (int16_t)(-entry_angle_abs_deg10),
            main_speed_mmps,
            main_radius_mm,
            main_angle_deg10);
        break;

    case TASK_SERVICE_ARC_ANCHOR_REAR_LEFT:
        TaskService_BuildArcWithEntry(arc_template,
            entry_speed_mmps,
            entry_radius_abs_mm,
            (int16_t)(-entry_angle_abs_deg10),
            main_speed_mmps,
            main_radius_mm,
            main_angle_deg10);
        break;

    case TASK_SERVICE_ARC_ANCHOR_REAR_RIGHT:
    default:
        TaskService_BuildArcWithEntry(arc_template,
            entry_speed_mmps,
            (int16_t)(-entry_radius_abs_mm),
            entry_angle_abs_deg10,
            main_speed_mmps,
            main_radius_mm,
            main_angle_deg10);
        break;
    }
}

static uint8_t TaskService_IsDistanceDoneByMeasure(int32_t distance_measure_mm, int32_t distance_target_mm)
{
    DeviceCenter_Snapshot_t device;
    int32_t distance_error_mm;
    int16_t center_speed_abs_mmps;

    DeviceCenter_GetSnapshot(&device);

    distance_error_mm = distance_target_mm - distance_measure_mm;
    center_speed_abs_mmps = TaskService_AbsInt16((int16_t)(
        (device.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] +
         device.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX]) / 2));

    if ((TaskService_AbsInt16((int16_t)distance_error_mm) <= APP_DISTANCE_DONE_ERR_MM) &&
        (center_speed_abs_mmps <= APP_DISTANCE_DONE_SPEED_MMPS)) {
        if (g_distance_settle_count < 255U) {
            g_distance_settle_count++;
        }
    } else {
        g_distance_settle_count = 0U;
    }

    return (uint8_t)(g_distance_settle_count >= APP_DISTANCE_DONE_HOLD_CYCLES);
}

static uint8_t TaskService_IsRouteDistanceSwitchReady(int32_t distance_measure_mm, int32_t distance_target_mm)
{
    DeviceCenter_Snapshot_t device;
    int32_t distance_error_mm;
    int16_t center_speed_abs_mmps;

    DeviceCenter_GetSnapshot(&device);

    distance_error_mm = distance_target_mm - distance_measure_mm;
    center_speed_abs_mmps = TaskService_AbsInt16((int16_t)(
        (device.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] +
         device.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX]) / 2));

    if ((TaskService_AbsInt16((int16_t)distance_error_mm) <= APP_ROUTE_DISTANCE_DONE_ERR_MM) &&
        (center_speed_abs_mmps <= APP_ROUTE_DISTANCE_DONE_SPEED_MMPS)) {
        if (g_distance_settle_count < 255U) {
            g_distance_settle_count++;
        }
    } else {
        g_distance_settle_count = 0U;
    }

    return (uint8_t)(g_distance_settle_count >= APP_ROUTE_DISTANCE_DONE_HOLD_CYCLES);
}

static uint8_t TaskService_IsRouteTurnEntryReady(int32_t distance_measure_mm, int32_t distance_target_mm)
{
    int32_t switch_distance_mm;

    if (distance_target_mm <= 0L) {
        return 1U;
    }

    switch_distance_mm = distance_target_mm - AppConfig_GetRouteTurnEntryLeadMm();
    if (switch_distance_mm < 0L) {
        switch_distance_mm = 0L;
    }

    return (uint8_t)(distance_measure_mm >= switch_distance_mm);
}

static uint8_t TaskService_IsRouteArcEntryReady(int32_t distance_measure_mm,
    int32_t distance_target_mm,
    int32_t entry_lead_mm)
{
    int32_t switch_distance_mm;

    if (distance_target_mm <= 0L) {
        return 1U;
    }

    if (entry_lead_mm < 0L) {
        entry_lead_mm = 0L;
    }

    switch_distance_mm = distance_target_mm - entry_lead_mm;
    if (switch_distance_mm < 0L) {
        switch_distance_mm = 0L;
    }

    return (uint8_t)(distance_measure_mm >= switch_distance_mm);
}

static uint8_t TaskService_IsTurnDone(void)
{
    DeviceGyro_Snapshot_t gyro;
    int16_t heading_error_deg10;
    int16_t yaw_abs_mdps;

    DeviceGyro_GetSnapshot(&gyro);

    heading_error_deg10 = (int16_t)(g_turn_target_heading_deg10 - gyro.heading_deg10);
    yaw_abs_mdps = TaskService_AbsInt16(gyro.yaw_mdps);

    if ((TaskService_AbsInt16(heading_error_deg10) <= APP_TURN_DONE_HEADING_ERR_DEG10) &&
        (yaw_abs_mdps <= APP_TURN_DONE_YAW_ABS_MDPS)) {
        if (g_turn_settle_count < 255U) {
            g_turn_settle_count++;
        }
    } else {
        g_turn_settle_count = 0U;
    }

    return (uint8_t)(g_turn_settle_count >= APP_TURN_DONE_HOLD_CYCLES);
}

static uint8_t TaskService_IsTurnDoneCustom(int16_t heading_err_limit_deg10,
    int16_t yaw_abs_limit_mdps,
    uint8_t hold_cycles)
{
    DeviceGyro_Snapshot_t gyro;
    int16_t heading_error_deg10;
    int16_t heading_delta_abs_deg10;
    int16_t yaw_abs_mdps;

    DeviceGyro_GetSnapshot(&gyro);

    heading_error_deg10 = (int16_t)(g_turn_target_heading_deg10 - gyro.heading_deg10);
    heading_delta_abs_deg10 = TaskService_AbsInt16(
        (int16_t)(heading_error_deg10 - g_turn_last_heading_error_deg10));
    yaw_abs_mdps = TaskService_AbsInt16(gyro.yaw_mdps);

    if ((TaskService_AbsInt16(heading_error_deg10) <= heading_err_limit_deg10) &&
        (yaw_abs_mdps <= yaw_abs_limit_mdps)) {
        if (g_turn_settle_count < 255U) {
            g_turn_settle_count++;
        }
        g_turn_stall_count = 0U;
    } else {
        g_turn_settle_count = 0U;

        if ((TaskService_AbsInt16(heading_error_deg10) <= APP_ROUTE_TURN_STALL_HEADING_ERR_DEG10) &&
            (yaw_abs_mdps <= APP_ROUTE_TURN_STALL_YAW_ABS_MDPS) &&
            (heading_delta_abs_deg10 <= APP_ROUTE_TURN_STALL_DELTA_DEG10)) {
            if (g_turn_stall_count < 255U) {
                g_turn_stall_count++;
            }
        } else {
            g_turn_stall_count = 0U;
        }
    }

    g_turn_last_heading_error_deg10 = heading_error_deg10;

    return (uint8_t)((g_turn_settle_count >= hold_cycles) ||
        (g_turn_stall_count >= APP_ROUTE_TURN_STALL_HOLD_CYCLES));
}

static void TaskService_StartDistancePhase(int16_t speed_target_mmps, int32_t distance_target_mm)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);
    g_phase_origin_distance_mm = device.chassis_distance_mm;
    g_distance_settle_count = 0U;
    BehaviorMotion_StraightDistance(speed_target_mmps, distance_target_mm);
}

static void TaskService_StartRouteDistancePhase(int16_t speed_target_mmps, int32_t distance_target_mm)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);
    g_phase_origin_distance_mm = device.chassis_distance_mm;
    g_distance_settle_count = 0U;
    BehaviorMotion_StraightDistance(speed_target_mmps, distance_target_mm);
}

static void TaskService_StartRouteDistancePhaseToHeading(int16_t speed_target_mmps,
    int32_t distance_target_mm,
    int16_t heading_target_deg10)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);
    g_phase_origin_distance_mm = device.chassis_distance_mm;
    g_distance_settle_count = 0U;
    BehaviorMotion_StraightDistanceToHeading(speed_target_mmps,
        distance_target_mm,
        heading_target_deg10);
}

static void TaskService_StartTurnPhase(int16_t delta_heading_deg10)
{
    DeviceGyro_Snapshot_t gyro;

    DeviceGyro_GetSnapshot(&gyro);
    g_turn_target_heading_deg10 = (int16_t)(gyro.heading_deg10 + delta_heading_deg10);
    g_turn_settle_count = 0U;
    g_turn_stall_count = 0U;
    g_turn_last_heading_error_deg10 = delta_heading_deg10;
    g_arc_settle_count = 0U;
    BehaviorMotion_TurnToHeading(g_turn_target_heading_deg10);
}

static void TaskService_StartArcPhase(int16_t speed_target_mmps,
    int16_t radius_mm,
    int16_t delta_heading_deg10)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);
    g_phase_origin_distance_mm = device.chassis_distance_mm;
    g_phase_origin_heading_deg10 = device.imu_heading_deg10;
    g_phase_origin_encoder_heading_deg10 = device.encoder_heading_deg10;
    g_turn_target_heading_deg10 = (int16_t)(device.imu_heading_deg10 + delta_heading_deg10);
    g_distance_settle_count = 0U;
    g_arc_settle_count = 0U;
    BehaviorMotion_ArcToHeading(speed_target_mmps, radius_mm, g_turn_target_heading_deg10);
}

static void TaskService_StartArcRadiusPhase(int16_t speed_target_mmps,
    int16_t radius_mm,
    int16_t delta_heading_deg10)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);
    g_phase_origin_distance_mm = device.chassis_distance_mm;
    g_phase_origin_heading_deg10 = device.imu_heading_deg10;
    g_phase_origin_encoder_heading_deg10 = device.encoder_heading_deg10;
    g_turn_target_heading_deg10 = (int16_t)(device.imu_heading_deg10 + delta_heading_deg10);
    g_distance_settle_count = 0U;
    g_arc_settle_count = 0U;
    BehaviorMotion_ArcByRadius(speed_target_mmps, radius_mm);
}

static int32_t TaskService_GetArcProgressDeg10(const DeviceCenter_Snapshot_t *device,
    int16_t arc_angle_deg10)
{
    int32_t encoder_heading_progress_deg10;

    if (device == 0) {
        return 0L;
    }

    encoder_heading_progress_deg10 = (int32_t)device->encoder_heading_deg10 -
        (int32_t)g_phase_origin_encoder_heading_deg10;
    if (arc_angle_deg10 < 0) {
        encoder_heading_progress_deg10 = -encoder_heading_progress_deg10;
    }
    if (encoder_heading_progress_deg10 < 0L) {
        encoder_heading_progress_deg10 = 0L;
    }

    return encoder_heading_progress_deg10;
}

static uint8_t TaskService_IsArcDoneByMeasureCustom(const DeviceCenter_Snapshot_t *device,
    int16_t arc_radius_mm,
    int16_t arc_angle_deg10,
    int32_t phase_distance_mm);

static uint8_t TaskService_IsArcDoneStable(const DeviceCenter_Snapshot_t *device,
    int16_t arc_radius_mm,
    int16_t arc_angle_deg10,
    int32_t phase_distance_mm,
    int16_t yaw_abs_limit_mdps,
    uint8_t hold_cycles);

static uint8_t TaskService_IsArcDoneByMeasure(const DeviceCenter_Snapshot_t *device,
    int16_t arc_angle_deg10,
    int32_t phase_distance_mm)
{
    return TaskService_IsArcDoneByMeasureCustom(device,
        AppConfig_GetArcTestRadiusMm(),
        arc_angle_deg10,
        phase_distance_mm);
}

static uint8_t TaskService_IsArcDoneByMeasureCustom(const DeviceCenter_Snapshot_t *device,
    int16_t arc_radius_mm,
    int16_t arc_angle_deg10,
    int32_t phase_distance_mm)
{
    int32_t heading_target_abs_deg10;
    int32_t encoder_heading_progress_deg10;
    int32_t encoder_heading_remain_deg10;
    int32_t arc_target_distance_mm;
    int32_t arc_done_min_distance_mm;
    int32_t arc_safety_distance_mm;

    if (device == 0) {
        return 0U;
    }

    heading_target_abs_deg10 = (int32_t)TaskService_AbsInt16(arc_angle_deg10);
    encoder_heading_progress_deg10 = TaskService_GetArcProgressDeg10(device, arc_angle_deg10);
    encoder_heading_remain_deg10 = heading_target_abs_deg10 - encoder_heading_progress_deg10;
    if (encoder_heading_remain_deg10 < 0L) {
        encoder_heading_remain_deg10 = 0L;
    }

    arc_target_distance_mm = TaskService_ArcLengthMmByRadiusAngle(
        arc_radius_mm,
        arc_angle_deg10);
    arc_done_min_distance_mm = (arc_target_distance_mm *
        (int32_t)APP_ARC_TEST_DONE_DISTANCE_FLOOR_X10) / 10L;
    arc_safety_distance_mm = (arc_target_distance_mm *
        (int32_t)APP_ARC_TEST_DISTANCE_SAFETY_SCALE_X10) / 10L;
    if (arc_safety_distance_mm < (arc_target_distance_mm + APP_ARC_TEST_DISTANCE_MARGIN_MM)) {
        arc_safety_distance_mm = arc_target_distance_mm + APP_ARC_TEST_DISTANCE_MARGIN_MM;
    }

    return (uint8_t)(((encoder_heading_remain_deg10 <= (int32_t)APP_ARC_TEST_DONE_ANGLE_ERR_DEG10) &&
        (phase_distance_mm >= arc_done_min_distance_mm)) ||
        (phase_distance_mm >= arc_safety_distance_mm));
}

static uint8_t TaskService_IsArcDoneStable(const DeviceCenter_Snapshot_t *device,
    int16_t arc_radius_mm,
    int16_t arc_angle_deg10,
    int32_t phase_distance_mm,
    int16_t yaw_abs_limit_mdps,
    uint8_t hold_cycles)
{
    DeviceGyro_Snapshot_t gyro;
    uint8_t arc_done;

    arc_done = TaskService_IsArcDoneByMeasureCustom(device,
        arc_radius_mm,
        arc_angle_deg10,
        phase_distance_mm);

    if (!arc_done) {
        g_arc_settle_count = 0U;
        return 0U;
    }

    DeviceGyro_GetSnapshot(&gyro);
    if (TaskService_AbsInt16(gyro.yaw_mdps) <= yaw_abs_limit_mdps) {
        if (g_arc_settle_count < 255U) {
            g_arc_settle_count++;
        }
    } else {
        g_arc_settle_count = 0U;
    }

    return (uint8_t)(g_arc_settle_count >= hold_cycles);
}

static void TaskService_RunModeA(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        BehaviorMotion_Straight(180);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 20U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if ((g_status.phase_index == 1U) && AppTick_IsExpired(g_phase_start_ms, 800U)) {
        BehaviorMotion_LineFollow(160);
        g_status.progress    = 60U;
        g_status.phase_index = 2U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if ((g_status.phase_index == 2U) && AppTick_IsExpired(g_phase_start_ms, 1200U)) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

static void TaskService_RunModeB(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        BehaviorMotion_Turn(900);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 30U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if ((g_status.phase_index == 1U) && AppTick_IsExpired(g_phase_start_ms, 600U)) {
        BehaviorMotion_Straight(200);
        g_status.progress    = 70U;
        g_status.phase_index = 2U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if ((g_status.phase_index == 2U) && AppTick_IsExpired(g_phase_start_ms, 900U)) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

static void TaskService_RunStraightTest(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        BehaviorMotion_Straight(APP_STRAIGHT_TEST_SPEED_MMPS);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 80U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    g_status.progress = 80U;
}

static void TaskService_RunArcTest(void)
{
    DeviceCenter_Snapshot_t device;
    int32_t encoder_heading_progress_deg10;
    int32_t encoder_heading_remain_deg10;
    int32_t heading_target_abs_deg10;
    int32_t phase_distance_mm;
    int16_t arc_speed_mmps = AppConfig_GetArcTestSpeedMmps();
    int16_t arc_radius_mm = AppConfig_GetArcTestRadiusMm();
    int16_t arc_angle_deg10 = TaskService_ArcSignedAngleByRadius(
        arc_radius_mm,
        AppConfig_GetArcTestAngleDeg10());

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartArcPhase(arc_speed_mmps,
            arc_radius_mm,
            arc_angle_deg10);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 10U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    phase_distance_mm = device.chassis_distance_mm - g_phase_origin_distance_mm;
    encoder_heading_progress_deg10 = TaskService_GetArcProgressDeg10(&device, arc_angle_deg10);
    heading_target_abs_deg10 = (int32_t)TaskService_AbsInt16(arc_angle_deg10);
    encoder_heading_remain_deg10 = heading_target_abs_deg10 - encoder_heading_progress_deg10;
    if (encoder_heading_remain_deg10 < 0L) {
        encoder_heading_remain_deg10 = 0L;
    }

    if (encoder_heading_remain_deg10 <= (int32_t)APP_ARC_TEST_APPROACH_ANGLE_DEG10) {
        arc_speed_mmps = APP_ARC_TEST_APPROACH_SPEED_MMPS;
    }

    BehaviorMotion_ArcToHeading(arc_speed_mmps, arc_radius_mm, g_turn_target_heading_deg10);

    if (heading_target_abs_deg10 > 0L) {
        int32_t progress = (encoder_heading_progress_deg10 * 100L) / heading_target_abs_deg10;

        if (progress < 10L) {
            progress = 10L;
        }
        if (progress > 95L) {
            progress = 95L;
        }
        g_status.progress = (uint8_t)progress;
    } else {
        g_status.progress = 80U;
    }

    if (TaskService_IsArcDoneByMeasure(&device, arc_angle_deg10, phase_distance_mm)) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

static void TaskService_RunGoDistance(int32_t distance_target_mm)
{
    DeviceCenter_Snapshot_t device;

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartDistancePhase(APP_DISTANCE_TEST_SPEED_MMPS, distance_target_mm);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 10U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if (distance_target_mm > 0) {
        int32_t progress = ((int32_t)device.chassis_distance_mm * 100L) / distance_target_mm;
        if (progress < 10L) {
            progress = 10L;
        }
        if (progress > 95L) {
            progress = 95L;
        }
        g_status.progress = (uint8_t)progress;
    } else {
        g_status.progress = 80U;
    }

    if (TaskService_IsDistanceDoneByMeasure(device.chassis_distance_mm, distance_target_mm)) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

static void TaskService_RunMotorTest1(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        BehaviorMotion_SpeedHold(APP_MOTOR_TEST_SPEED_MMPS);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 50U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    g_status.progress = 80U;
}

static void TaskService_RunMotorTest2(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        BehaviorMotion_OpenLoop(-APP_MOTOR_TEST_TURN_DUTY, APP_MOTOR_TEST_TURN_DUTY);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 50U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    g_status.progress = 80U;
}

static void TaskService_RunTurnAngle(int16_t delta_heading_deg10)
{
    DeviceCenter_Snapshot_t device;
    int16_t heading_error_deg10;

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        TaskService_StartTurnPhase(delta_heading_deg10);
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 20U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    heading_error_deg10 = (int16_t)(g_turn_target_heading_deg10 - device.imu_heading_deg10);
    if (TaskService_AbsInt16(heading_error_deg10) <= APP_TURN_DONE_HEADING_ERR_DEG10) {
        g_status.progress = 80U;
    } else {
        g_status.progress = 40U;
    }

    if (TaskService_IsTurnDone()) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

static void TaskService_RunRoute2500L90_500R90_500(void)
{
    DeviceCenter_Snapshot_t device;
    int32_t phase_distance_mm;
    int16_t route_speed_mmps = AppConfig_GetRouteTestSpeedMmps();
    int32_t segment1_mm = AppConfig_GetRouteSegment1Mm();
    int32_t segment2_mm = AppConfig_GetRouteSegment2Mm();
    int32_t segment3_mm = AppConfig_GetRouteSegment3Mm();
    int16_t turn_angle_deg10 = AppConfig_GetRouteTurnAngleDeg10();
    int16_t turn_done_yaw_abs_mdps = AppConfig_GetRouteTurnDoneYawAbsMdps();
    uint8_t turn_done_hold_cycles = AppConfig_GetRouteTurnDoneHoldCycles();

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartRouteDistancePhase(route_speed_mmps, segment1_mm);
        g_status.stage = TASK_STAGE_EXECUTE;
        g_status.phase_index = 1U;
        g_status.progress = 5U;
        g_phase_start_ms = AppTick_GetMs();
        return;
    }

    phase_distance_mm = device.chassis_distance_mm - g_phase_origin_distance_mm;

    switch (g_status.phase_index) {

    case 1U:
        if (segment1_mm > 0) {
            int32_t progress = (phase_distance_mm * 45L) / segment1_mm;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 45L) {
                progress = 45L;
            }
            g_status.progress = (uint8_t)(5U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, segment1_mm)) {
            TaskService_StartTurnPhase(turn_angle_deg10);
            g_status.phase_index = 3U;
            g_status.progress = 60U;
        }
        break;

    case 2U:
        g_status.progress = 58U;
        if (AppTick_IsExpired(g_phase_start_ms, APP_ROUTE_PHASE_DELAY_MS)) {
            TaskService_StartTurnPhase(turn_angle_deg10);
            g_status.phase_index = 3U;
            g_status.progress = 60U;
        }
        break;

    case 3U:
        g_status.progress = 64U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(route_speed_mmps,
                segment2_mm,
                g_turn_target_heading_deg10);
            g_status.phase_index = 4U;
            g_status.progress = 66U;
        }
        break;

    case 4U:
        if (segment2_mm > 0) {
            int32_t progress = (phase_distance_mm * 14L) / segment2_mm;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 14L) {
                progress = 14L;
            }
            g_status.progress = (uint8_t)(66U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, segment2_mm)) {
            TaskService_StartTurnPhase((int16_t)(-turn_angle_deg10));
            g_status.phase_index = 6U;
            g_status.progress = 88U;
        }
        break;

    case 5U:
        g_status.progress = 86U;
        if (AppTick_IsExpired(g_phase_start_ms, APP_ROUTE_PHASE_DELAY_MS)) {
            TaskService_StartTurnPhase((int16_t)(-turn_angle_deg10));
            g_status.phase_index = 6U;
            g_status.progress = 88U;
        }
        break;

    case 6U:
        g_status.progress = 92U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(route_speed_mmps,
                segment3_mm,
                g_turn_target_heading_deg10);
            g_status.phase_index = 7U;
            g_status.progress = 94U;
        }
        break;

    case 7U:
        if (segment3_mm > 0) {
            int32_t progress = (phase_distance_mm * 6L) / segment3_mm;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 6L) {
                progress = 6L;
            }
            g_status.progress = (uint8_t)(94U + progress);
        }
        if (TaskService_IsDistanceDoneByMeasure(phase_distance_mm, segment3_mm)) {
            g_status.stage = TASK_STAGE_COMPLETE;
            g_status.progress = 100U;
            TaskService_FinishCurrent();
        }
        break;

    default:
        g_status.progress = 0U;
        break;
    }
}

static void TaskService_RunQ2_700_Arc180_Arc180_R90_700(uint8_t start_left)
{
    DeviceCenter_Snapshot_t device;
    int32_t phase_distance_mm;
    int16_t turn1_angle_deg10;
    int16_t turn2_angle_deg10;
    int16_t turn3_angle_deg10;
    int16_t turn4_angle_deg10;
    int16_t straight_speed_mmps = AppConfig_GetRouteTestSpeedMmps();
    int16_t turn_done_yaw_abs_mdps = APP_Q2_TURN_DONE_YAW_ABS_MDPS;
    uint8_t turn_done_hold_cycles = APP_Q2_TURN_DONE_HOLD_CYCLES;

    if (start_left != 0U) {
        turn1_angle_deg10 = APP_Q2_TURN1_ANGLE_DEG10;
        turn2_angle_deg10 = (int16_t)(-APP_Q2_TURN2_ANGLE_DEG10);
        turn3_angle_deg10 = APP_Q2_TURN3_ANGLE_DEG10;
        turn4_angle_deg10 = (int16_t)(-APP_Q2_TURN4_ANGLE_DEG10);
    } else {
        turn1_angle_deg10 = (int16_t)(-APP_Q2_TURN1_ANGLE_DEG10);
        turn2_angle_deg10 = APP_Q2_TURN2_ANGLE_DEG10;
        turn3_angle_deg10 = (int16_t)(-APP_Q2_TURN3_ANGLE_DEG10);
        turn4_angle_deg10 = APP_Q2_TURN4_ANGLE_DEG10;
    }

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartRouteDistancePhase(straight_speed_mmps, APP_Q2_SEGMENT1_MM);
        g_status.stage = TASK_STAGE_EXECUTE;
        g_status.phase_index = 1U;
        g_status.progress = 5U;
        g_phase_start_ms = AppTick_GetMs();
        return;
    }

    phase_distance_mm = device.chassis_distance_mm - g_phase_origin_distance_mm;

    switch (g_status.phase_index) {
    case 1U:
        if (APP_Q2_SEGMENT1_MM > 0) {
            int32_t progress = (phase_distance_mm * 20L) / APP_Q2_SEGMENT1_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 20L) {
                progress = 20L;
            }
            g_status.progress = (uint8_t)(5U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q2_SEGMENT1_MM)) {
            TaskService_StartTurnPhase(turn1_angle_deg10);
            g_status.phase_index = 2U;
            g_status.progress = 22U;
        }
        break;

    case 2U:
        g_status.progress = 28U;
        if (TaskService_IsTurnDoneCustom(APP_Q2_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q2_SEGMENT2_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 3U;
            g_status.progress = 32U;
        }
        break;

    case 3U:
        if (APP_Q2_SEGMENT2_MM > 0) {
            int32_t progress = (phase_distance_mm * 12L) / APP_Q2_SEGMENT2_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 12L) {
                progress = 12L;
            }
            g_status.progress = (uint8_t)(32U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q2_SEGMENT2_MM)) {
            TaskService_StartTurnPhase(turn2_angle_deg10);
            g_status.phase_index = 4U;
            g_status.progress = 46U;
        }
        break;

    case 4U:
        g_status.progress = 52U;
        if (TaskService_IsTurnDoneCustom(APP_Q2_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q2_SEGMENT3_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 5U;
            g_status.progress = 56U;
        }
        break;

    case 5U:
        if (APP_Q2_SEGMENT3_MM > 0) {
            int32_t progress = (phase_distance_mm * 12L) / APP_Q2_SEGMENT3_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 12L) {
                progress = 12L;
            }
            g_status.progress = (uint8_t)(56U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q2_SEGMENT3_MM)) {
            TaskService_StartTurnPhase(turn3_angle_deg10);
            g_status.phase_index = 6U;
            g_status.progress = 70U;
        }
        break;

    case 6U:
        g_status.progress = 76U;
        if (TaskService_IsTurnDoneCustom(APP_Q2_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q2_SEGMENT4_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 7U;
            g_status.progress = 80U;
        }
        break;

    case 7U:
        if (APP_Q2_SEGMENT4_MM > 0) {
            int32_t progress = (phase_distance_mm * 8L) / APP_Q2_SEGMENT4_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 8L) {
                progress = 8L;
            }
            g_status.progress = (uint8_t)(80U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q2_SEGMENT4_MM)) {
            TaskService_StartTurnPhase(turn4_angle_deg10);
            g_status.phase_index = 8U;
            g_status.progress = 89U;
        }
        break;

    case 8U:
        g_status.progress = 92U;
        if (TaskService_IsTurnDoneCustom(APP_Q2_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q2_SEGMENT5_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 9U;
            g_status.progress = 94U;
        }
        break;

    case 9U:
        if (APP_Q2_SEGMENT5_MM > 0) {
            int32_t progress = (phase_distance_mm * 6L) / APP_Q2_SEGMENT5_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 6L) {
                progress = 6L;
            }
            g_status.progress = (uint8_t)(94U + progress);
        }
        if (TaskService_IsDistanceDoneByMeasure(phase_distance_mm, APP_Q2_SEGMENT5_MM)) {
            g_status.stage = TASK_STAGE_COMPLETE;
            g_status.progress = 100U;
            TaskService_FinishCurrent();
        }
        break;

    default:
        g_status.progress = 0U;
        break;
    }
}

static void TaskService_RunQ3_1200_R270_500_R420_1800(void)
{
    DeviceCenter_Snapshot_t device;
    int32_t phase_distance_mm;
    int32_t arc_progress_deg10;
    int32_t final_stabilize_mm = APP_Q3_FINAL_STABILIZE_MM;
    int32_t final_main_mm;
    int16_t straight_speed_mmps = AppConfig_GetRouteTestSpeedMmps();
    int16_t arc_speed_mmps = APP_Q3_ARC_SPEED_MMPS;
    int16_t arc_radius_mm = (int16_t)(-APP_Q3_ARC_RADIUS_MM);

    final_main_mm = APP_Q3_SEGMENT3_MM - final_stabilize_mm;
    if (final_main_mm < 0L) {
        final_main_mm = 0L;
    }
    if (final_stabilize_mm < 0L) {
        final_stabilize_mm = 0L;
    }
    if (final_stabilize_mm > APP_Q3_SEGMENT3_MM) {
        final_stabilize_mm = APP_Q3_SEGMENT3_MM;
    }

    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartRouteDistancePhase(straight_speed_mmps, APP_Q3_SEGMENT1_MM);
        g_status.stage = TASK_STAGE_EXECUTE;
        g_status.phase_index = 1U;
        g_status.progress = 5U;
        g_phase_start_ms = AppTick_GetMs();
        return;
    }

    phase_distance_mm = device.chassis_distance_mm - g_phase_origin_distance_mm;

    switch (g_status.phase_index) {
    case 1U:
        if (APP_Q3_SEGMENT1_MM > 0) {
            int32_t progress = (phase_distance_mm * 28L) / APP_Q3_SEGMENT1_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 28L) {
                progress = 28L;
            }
            g_status.progress = (uint8_t)(5U + progress);
        }
        if (TaskService_IsRouteArcEntryReady(phase_distance_mm,
            APP_Q3_SEGMENT1_MM,
            APP_Q3_ARC1_ENTRY_LEAD_MM)) {
            TaskService_StartArcRadiusPhase(arc_speed_mmps,
                arc_radius_mm,
                (int16_t)(-APP_Q3_ARC1_ANGLE_DEG10));
            g_status.phase_index = 2U;
            g_status.progress = 35U;
        }
        break;

    case 2U:
        arc_progress_deg10 = TaskService_GetArcProgressDeg10(&device,
            (int16_t)(-APP_Q3_ARC1_ANGLE_DEG10));
        if (arc_progress_deg10 > 0L) {
            int32_t progress = (arc_progress_deg10 * 18L) / APP_Q3_ARC1_ANGLE_DEG10;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 18L) {
                progress = 18L;
            }
            g_status.progress = (uint8_t)(35U + progress);
        }
        if (TaskService_IsArcDoneByMeasureCustom(&device,
            arc_radius_mm,
            (int16_t)(-APP_Q3_ARC1_ANGLE_DEG10),
            phase_distance_mm)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q3_SEGMENT2_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 3U;
            g_status.progress = 54U;
        }
        break;

    case 3U:
        if (APP_Q3_SEGMENT2_MM > 0) {
            int32_t progress = (phase_distance_mm * 8L) / APP_Q3_SEGMENT2_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 8L) {
                progress = 8L;
            }
            g_status.progress = (uint8_t)(54U + progress);
        }
        if (TaskService_IsRouteArcEntryReady(phase_distance_mm,
            APP_Q3_SEGMENT2_MM,
            APP_Q3_ARC2_ENTRY_LEAD_MM)) {
            TaskService_StartArcRadiusPhase(arc_speed_mmps,
                arc_radius_mm,
                (int16_t)(-APP_Q3_ARC2_ANGLE_DEG10));
            g_status.phase_index = 4U;
            g_status.progress = 64U;
        }
        break;

    case 4U:
        arc_progress_deg10 = TaskService_GetArcProgressDeg10(&device,
            (int16_t)(-APP_Q3_ARC2_ANGLE_DEG10));
        if (arc_progress_deg10 > 0L) {
            int32_t progress = (arc_progress_deg10 * 24L) / APP_Q3_ARC2_ANGLE_DEG10;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 24L) {
                progress = 24L;
            }
            g_status.progress = (uint8_t)(64U + progress);
        }
        if (TaskService_IsArcDoneByMeasureCustom(&device,
            arc_radius_mm,
            (int16_t)(-APP_Q3_ARC2_ANGLE_DEG10),
            phase_distance_mm)) {
            TaskService_StartRouteDistancePhase(straight_speed_mmps,
                APP_Q3_SEGMENT3_MM);
            g_status.phase_index = 5U;
            g_status.progress = 86U;
        }
        break;

    case 5U:
        if (APP_Q3_SEGMENT3_MM > 0L) {
            int32_t progress = (phase_distance_mm * 14L) / APP_Q3_SEGMENT3_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 14L) {
                progress = 14L;
            }
            g_status.progress = (uint8_t)(86U + progress);
        }
        if (TaskService_IsDistanceDoneByMeasure(phase_distance_mm, APP_Q3_SEGMENT3_MM)) {
            g_status.stage = TASK_STAGE_COMPLETE;
            g_status.progress = 100U;
            TaskService_FinishCurrent();
        }
        break;

    case 6U:
        break;

    default:
        g_status.progress = 0U;
        break;
    }
}

static void TaskService_RunQ4(void)
{
    DeviceCenter_Snapshot_t device;
    int32_t phase_distance_mm;
    int16_t straight_speed_mmps = AppConfig_GetRouteTestSpeedMmps();
    int16_t turn_done_yaw_abs_mdps = APP_ROUTE_TURN_DONE_YAW_ABS_MDPS;
    uint8_t turn_done_hold_cycles = APP_ROUTE_TURN_DONE_HOLD_CYCLES;

    /*
     * phase 1: 直行 1000
     * phase 2: 左转 90
     * phase 3: 直行 1000
     * phase 4: 右转 90
     * phase 5: 直行 1500
     * phase 6: 右转 90
     * phase 7: 直行 75
     * phase 8: 左转 90
     * phase 9: 直行 50
     */
    DeviceCenter_GetSnapshot(&device);

    if (g_status.stage == TASK_STAGE_PREPARE) {
        DeviceCenter_ResetOdometry();
        TaskService_StartRouteDistancePhase(straight_speed_mmps, 1000);
        g_status.stage = TASK_STAGE_EXECUTE;
        g_status.phase_index = 1U;
        g_status.progress = 5U;
        g_phase_start_ms = AppTick_GetMs();
        return;
    }

    phase_distance_mm = device.chassis_distance_mm - g_phase_origin_distance_mm;

    switch (g_status.phase_index) {
    case 1U:
        if (phase_distance_mm > 0L) {
            int32_t progress = (phase_distance_mm * 12L) / APP_Q4_SEGMENT1_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 12L) {
                progress = 12L;
            }
            g_status.progress = (uint8_t)(5U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q4_SEGMENT1_MM)) {
            TaskService_StartTurnPhase(APP_Q4_TURN1_ANGLE_DEG10);
            g_status.phase_index = 2U;
            g_status.progress = 20U;
        }
        break;

    case 2U:
        g_status.progress = 24U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q4_SEGMENT2_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 3U;
            g_status.progress = 28U;
        }
        break;

    case 3U:
        if (phase_distance_mm > 0L) {
            int32_t progress = (phase_distance_mm * 12L) / APP_Q4_SEGMENT2_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 12L) {
                progress = 12L;
            }
            g_status.progress = (uint8_t)(28U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q4_SEGMENT2_MM)) {
            TaskService_StartTurnPhase(APP_Q4_TURN2_ANGLE_DEG10);
            g_status.phase_index = 4U;
            g_status.progress = 42U;
        }
        break;

    case 4U:
        g_status.progress = 46U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q4_SEGMENT3_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 5U;
            g_status.progress = 50U;
        }
        break;

    case 5U:
        if (phase_distance_mm > 0L) {
            int32_t progress = (phase_distance_mm * 20L) / APP_Q4_SEGMENT3_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 20L) {
                progress = 20L;
            }
            g_status.progress = (uint8_t)(50U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q4_SEGMENT3_MM)) {
            TaskService_StartTurnPhase(APP_Q4_TURN3_ANGLE_DEG10);
            g_status.phase_index = 6U;
            g_status.progress = 72U;
        }
        break;

    case 6U:
        g_status.progress = 76U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q4_SEGMENT4_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 7U;
            g_status.progress = 80U;
        }
        break;

    case 7U:
        if (phase_distance_mm > 0L) {
            int32_t progress = (phase_distance_mm * 6L) / APP_Q4_SEGMENT4_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 6L) {
                progress = 6L;
            }
            g_status.progress = (uint8_t)(80U + progress);
        }
        if (TaskService_IsRouteTurnEntryReady(phase_distance_mm, APP_Q4_SEGMENT4_MM)) {
            TaskService_StartTurnPhase(APP_Q4_TURN4_ANGLE_DEG10);
            g_status.phase_index = 8U;
            g_status.progress = 88U;
        }
        break;

    case 8U:
        g_status.progress = 92U;
        if (TaskService_IsTurnDoneCustom(APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10,
            turn_done_yaw_abs_mdps,
            turn_done_hold_cycles)) {
            TaskService_StartRouteDistancePhaseToHeading(straight_speed_mmps,
                APP_Q4_SEGMENT5_MM,
                g_turn_target_heading_deg10);
            g_status.phase_index = 9U;
            g_status.progress = 95U;
        }
        break;

    case 9U:
        if (phase_distance_mm > 0L) {
            int32_t progress = (phase_distance_mm * 5L) / APP_Q4_SEGMENT5_MM;
            if (progress < 0L) {
                progress = 0L;
            }
            if (progress > 5L) {
                progress = 5L;
            }
            g_status.progress = (uint8_t)(95U + progress);
        }
        if (TaskService_IsDistanceDoneByMeasure(phase_distance_mm, APP_Q4_SEGMENT5_MM)) {
            g_status.stage = TASK_STAGE_COMPLETE;
            g_status.progress = 100U;
            TaskService_FinishCurrent();
        }
        break;

    default:
        g_status.progress = 0U;
        break;
    }
}


static void TaskService_RunSensorScan(void)
{
    if (g_status.stage == TASK_STAGE_PREPARE) {
        BehaviorMotion_Stop();
        g_status.stage       = TASK_STAGE_EXECUTE;
        g_status.progress    = 60U;
        g_status.phase_index = 1U;
        g_phase_start_ms     = AppTick_GetMs();
        return;
    }

    if (AppTick_IsExpired(g_phase_start_ms, 200U)) {
        g_status.stage    = TASK_STAGE_COMPLETE;
        g_status.progress = 100U;
        TaskService_FinishCurrent();
    }
}

void TaskService_Init(void)
{
    g_queue.head  = 0U;
    g_queue.tail  = 0U;
    g_queue.size  = 0U;

    g_status.current_action        = TASK_ACTION_NONE;
    g_status.last_completed_action = TASK_ACTION_NONE;
    g_status.stage                 = TASK_STAGE_IDLE;
    g_status.queue_depth           = 0U;
    g_status.busy                  = 0U;
    g_status.progress              = 0U;
    g_status.phase_index           = 0U;
}

uint8_t TaskService_Enqueue(TaskAction_e action)
{
    if ((action == TASK_ACTION_NONE) || (g_queue.size >= APP_TASK_QUEUE_DEPTH)) {
        return 0U;
    }

    if (TaskService_IsQueued(action)) {
        return 1U;
    }

    g_queue.data[g_queue.tail] = action;
    g_queue.tail = (uint8_t)((g_queue.tail + 1U) % APP_TASK_QUEUE_DEPTH);
    g_queue.size++;
    g_status.queue_depth = g_queue.size;
    return 1U;
}

void TaskService_CancelCurrent(void)
{
    BehaviorMotion_Stop();
    TaskService_ResetQueue();
    g_status.current_action = TASK_ACTION_NONE;
    g_status.stage = TASK_STAGE_IDLE;
    g_status.busy = 0U;
    g_status.progress = 0U;
    g_status.phase_index = 0U;
    g_turn_target_heading_deg10 = 0;
    g_turn_settle_count = 0U;
    g_distance_settle_count = 0U;
    g_phase_origin_distance_mm = 0;
    g_phase_origin_heading_deg10 = 0;
    g_phase_origin_encoder_heading_deg10 = 0;
}

void TaskService_Tick10ms(void)
{
    TaskAction_e action;

    if (!g_status.busy) {
        if (TaskService_Dequeue(&action)) {
            TaskService_Start(action);
        } else {
            BehaviorMotion_Stop();
            return;
        }
    }

    switch (g_status.current_action) {
    case TASK_ACTION_RUN_MODE_A:
        TaskService_RunModeA();
        break;
    case TASK_ACTION_RUN_MODE_B:
        TaskService_RunModeB();
        break;
    case TASK_ACTION_STRAIGHT_TEST:
        TaskService_RunStraightTest();
        break;
    case TASK_ACTION_ARC_TEST:
        TaskService_RunArcTest();
        break;
    case TASK_ACTION_GO_1000MM:
        TaskService_RunGoDistance(APP_DISTANCE_TEST_TARGET_MM);
        break;
    case TASK_ACTION_ROUTE_2500_L90_500_R90_500:
        TaskService_RunRoute2500L90_500R90_500();
        break;
    case TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT:
        TaskService_RunQ2_700_Arc180_Arc180_R90_700(1U);
        break;
    case TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT:
        TaskService_RunQ2_700_Arc180_Arc180_R90_700(0U);
        break;
    case TASK_ACTION_Q3_1200_R270_500_R420_1800:
        TaskService_RunQ3_1200_R270_500_R420_1800();
        break;
    case TASK_ACTION_Q4:
        TaskService_RunQ4();
        break;
    case TASK_ACTION_MOTOR_TEST_1:
        TaskService_RunMotorTest1();
        break;
    case TASK_ACTION_MOTOR_TEST_2:
        TaskService_RunMotorTest2();
        break;
    case TASK_ACTION_TURN_LEFT_90:
        TaskService_RunTurnAngle(APP_TURN_TEST_ANGLE_90_DEG10);
        break;
    case TASK_ACTION_TURN_RIGHT_90:
        TaskService_RunTurnAngle(-APP_TURN_TEST_ANGLE_90_DEG10);
        break;
    case TASK_ACTION_SENSOR_SCAN:
        TaskService_RunSensorScan();
        break;
    case TASK_ACTION_NONE:
    default:
        TaskService_FinishCurrent();
        break;
    }
}

void TaskService_GetStatus(TaskService_Status_t *status)
{
    if (status == 0) {
        return;
    }

    *status = g_status;
}

const char *TaskService_GetActionName(TaskAction_e action)
{
    switch (action) {
    case TASK_ACTION_RUN_MODE_A:
        return "Run A";
    case TASK_ACTION_RUN_MODE_B:
        return "Run B";
    case TASK_ACTION_STRAIGHT_TEST:
        return "Straight";
    case TASK_ACTION_ARC_TEST:
        return "Arc";
    case TASK_ACTION_GO_1000MM:
        return "Go1000";
    case TASK_ACTION_ROUTE_2500_L90_500_R90_500:
        return "Route250";
    case TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT:
        return "Q2Left";
    case TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT:
        return "Q2Right";
    case TASK_ACTION_Q3_1200_R270_500_R420_1800:
        return "Q3Route";
    case TASK_ACTION_Q4:
        return "Q4Route";
    case TASK_ACTION_MOTOR_TEST_1:
        return "Motor1";
    case TASK_ACTION_MOTOR_TEST_2:
        return "Motor2";
    case TASK_ACTION_TURN_LEFT_90:
        return "TurnL90";
    case TASK_ACTION_TURN_RIGHT_90:
        return "TurnR90";
    case TASK_ACTION_SENSOR_SCAN:
        return "Sensor";
    case TASK_ACTION_NONE:
    default:
        return "Idle";
    }
}

const char *TaskService_GetStageName(TaskStage_e stage)
{
    switch (stage) {
    case TASK_STAGE_PREPARE:
        return "Prepare";
    case TASK_STAGE_EXECUTE:
        return "Execute";
    case TASK_STAGE_COMPLETE:
        return "Done";
    case TASK_STAGE_IDLE:
    default:
        return "Idle";
    }
}
