#include "ARCH/interaction/interaction_service.h"

#include <stdio.h>
#include <string.h>

#include "AppTick.h"
#include "ARCH/behavior/behavior_motion.h"
#include "ARCH/control/motion_control.h"
#include "ARCH/device/device_center.h"
#include "ARCH/task/task_service.h"

#define INTERACTION_REFRESH_PERIOD_MS           (200U)

typedef struct
{
    InteractionPage_t page;
    InteractionAction_e current_action;
    uint8_t dirty;
    uint32_t last_refresh_ms;
} InteractionService_State_t;

static InteractionService_State_t g_interaction;

static void InteractionService_SetPage(const char *l1, const char *l2, const char *l3, const char *l4);

typedef enum
{
    INTERACTION_SETTINGS_PAGE_Q1 = 0,
    INTERACTION_SETTINGS_PAGE_ARC,
} InteractionSettingsPage_e;

typedef enum
{
    INTERACTION_SETTING_Q1_SPEED = 0,
    INTERACTION_SETTING_Q1_SEG1,
    INTERACTION_SETTING_Q1_SEG2,
    INTERACTION_SETTING_Q1_SEG3,
    INTERACTION_SETTING_Q1_ANGLE,
    INTERACTION_SETTING_Q1_LEAD,
    INTERACTION_SETTING_Q1_BOOST_SPEED,
    INTERACTION_SETTING_Q1_BOOST_TICKS,
    INTERACTION_SETTING_Q1_TURN_DONE_YAW,
    INTERACTION_SETTING_Q1_STRAIGHT_BIAS,
    INTERACTION_SETTING_COUNT
} InteractionSetting_e;

static uint8_t g_settings_index = 0U;
static InteractionSettingsPage_e g_settings_page = INTERACTION_SETTINGS_PAGE_Q1;

static int32_t InteractionService_ClampInt32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static const char *InteractionService_GetSettingName(uint8_t index)
{
    if (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) {
        switch (index) {
        case 0U:
            return "ARC SPD";
        case 1U:
            return "ARC RAD";
        case 2U:
            return "ARC ANG";
        default:
            return "ARC";
        }
    }

    switch ((InteractionSetting_e)index) {
    case INTERACTION_SETTING_Q1_SPEED:
        return "Q1 SPD";
    case INTERACTION_SETTING_Q1_SEG1:
        return "Q1 D1";
    case INTERACTION_SETTING_Q1_SEG2:
        return "Q1 D2";
    case INTERACTION_SETTING_Q1_SEG3:
        return "Q1 D3";
    case INTERACTION_SETTING_Q1_ANGLE:
        return "Q1 ANG";
    case INTERACTION_SETTING_Q1_LEAD:
        return "Q1 LEAD";
    case INTERACTION_SETTING_Q1_BOOST_SPEED:
        return "Q1 BST";
    case INTERACTION_SETTING_Q1_BOOST_TICKS:
        return "Q1 BTK";
    case INTERACTION_SETTING_Q1_TURN_DONE_YAW:
        return "Q1 TYAW";
    case INTERACTION_SETTING_Q1_STRAIGHT_BIAS:
        return "Q1 BIAS";
    case INTERACTION_SETTING_COUNT:
    default:
        return "Q1";
    }
}

static int32_t InteractionService_GetSettingValue(uint8_t index)
{
    if (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) {
        switch (index) {
        case 0U:
            return g_app_arc_test_speed_mmps;
        case 1U:
            return g_app_arc_test_radius_mm;
        case 2U:
            return g_app_arc_test_angle_deg10;
        default:
            return 0;
        }
    }

    switch ((InteractionSetting_e)index) {
    case INTERACTION_SETTING_Q1_SPEED:
        return g_app_route_test_speed_mmps;
    case INTERACTION_SETTING_Q1_SEG1:
        return g_app_route_segment1_mm;
    case INTERACTION_SETTING_Q1_SEG2:
        return g_app_route_segment2_mm;
    case INTERACTION_SETTING_Q1_SEG3:
        return g_app_route_segment3_mm;
    case INTERACTION_SETTING_Q1_ANGLE:
        return g_app_route_turn_angle_deg10;
    case INTERACTION_SETTING_Q1_LEAD:
        return g_app_route_turn_entry_lead_mm;
    case INTERACTION_SETTING_Q1_BOOST_SPEED:
        return g_app_turn_exit_boost_mmps;
    case INTERACTION_SETTING_Q1_BOOST_TICKS:
        return g_app_turn_exit_boost_ticks;
    case INTERACTION_SETTING_Q1_TURN_DONE_YAW:
        return g_app_route_turn_done_yaw_abs_mdps;
    case INTERACTION_SETTING_Q1_STRAIGHT_BIAS:
        return g_app_straight_side_bias_x10;
    case INTERACTION_SETTING_COUNT:
    default:
        return 0;
    }
}

static int32_t InteractionService_GetSettingStep(uint8_t index)
{
    if (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) {
        switch (index) {
        case 0U:
            return 10;
        case 1U:
            return 10;
        case 2U:
            return 50;
        default:
            return 1;
        }
    }

    switch ((InteractionSetting_e)index) {
    case INTERACTION_SETTING_Q1_SPEED:
        return 10;
    case INTERACTION_SETTING_Q1_SEG1:
    case INTERACTION_SETTING_Q1_SEG2:
    case INTERACTION_SETTING_Q1_SEG3:
        return 10;
    case INTERACTION_SETTING_Q1_ANGLE:
        return 1;
    case INTERACTION_SETTING_Q1_LEAD:
        return 5;
    case INTERACTION_SETTING_Q1_BOOST_SPEED:
        return 10;
    case INTERACTION_SETTING_Q1_BOOST_TICKS:
        return 1;
    case INTERACTION_SETTING_Q1_TURN_DONE_YAW:
        return 10;
    case INTERACTION_SETTING_Q1_STRAIGHT_BIAS:
        return 1;
    case INTERACTION_SETTING_COUNT:
    default:
        return 1;
    }
}

static void InteractionService_RenderSettingsPage(void)
{
    char line1[APP_UI_LINE_LEN + 1U];
    char line2[APP_UI_LINE_LEN + 1U];
    char line3[APP_UI_LINE_LEN + 1U];
    char line4[APP_UI_LINE_LEN + 1U];
    const char *name = InteractionService_GetSettingName(g_settings_index);
    int32_t value = InteractionService_GetSettingValue(g_settings_index);
    int32_t step = InteractionService_GetSettingStep(g_settings_index);
    uint8_t item_count = (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) ? 3U :
        (uint8_t)INTERACTION_SETTING_COUNT;

    snprintf(line1, sizeof(line1), "%s %u/%u",
        name,
        (unsigned)(g_settings_index + 1U),
        (unsigned)item_count);

    if ((g_settings_page == INTERACTION_SETTINGS_PAGE_Q1) &&
        (g_settings_index == (uint8_t)INTERACTION_SETTING_Q1_STRAIGHT_BIAS)) {
        int32_t bias_abs_x10 = (value < 0) ? (-value) : value;
        char sign = (value < 0) ? '-' : '+';
        snprintf(line2, sizeof(line2), "VAL:%c%ld.%ld ST:.1",
            sign,
            (long)(bias_abs_x10 / 10),
            (long)(bias_abs_x10 % 10));
    } else {
        snprintf(line2, sizeof(line2), "VAL:%5ld ST:%-2ld",
            (long)value,
            (long)step);
    }

    if (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) {
        snprintf(line3, sizeof(line3), "R:+L  -:R 3600");
        snprintf(line4, sizeof(line4), "K1+ K2- K3NXT");
        InteractionService_SetPage(line1, line2, line3, line4);
    } else {
        snprintf(line3, sizeof(line3), "K1+ K2- K3NEXT");
        snprintf(line4, sizeof(line4), "LIVE K4:BACK");
        InteractionService_SetPage(line1, line2, line3, line4);
    }
}

static void InteractionService_AdjustCurrentSetting(int8_t delta)
{
    int32_t value;
    int32_t step;

    value = InteractionService_GetSettingValue(g_settings_index);
    step = InteractionService_GetSettingStep(g_settings_index);
    value += ((int32_t)delta * step);

    if (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) {
        switch (g_settings_index) {
        case 0U:
            g_app_arc_test_speed_mmps = (int16_t)InteractionService_ClampInt32(value, 80, 500);
            break;
        case 1U:
            g_app_arc_test_radius_mm = (int16_t)InteractionService_ClampInt32(value, -2000, 2000);
            if (g_app_arc_test_radius_mm == 0) {
                g_app_arc_test_radius_mm = (delta >= 0) ? 10 : -10;
            }
            break;
        case 2U:
            g_app_arc_test_angle_deg10 = (int16_t)InteractionService_ClampInt32(value, -20000, 20000);
            if (g_app_arc_test_angle_deg10 == 0) {
                g_app_arc_test_angle_deg10 = (delta >= 0) ? 50 : -50;
            }
            break;
        default:
            break;
        }
        return;
    }

    switch ((InteractionSetting_e)g_settings_index) {
    case INTERACTION_SETTING_Q1_SPEED:
        g_app_route_test_speed_mmps = (int16_t)InteractionService_ClampInt32(value, 100, 600);
        break;
    case INTERACTION_SETTING_Q1_SEG1:
        g_app_route_segment1_mm = InteractionService_ClampInt32(value, 100, 5000);
        break;
    case INTERACTION_SETTING_Q1_SEG2:
        g_app_route_segment2_mm = InteractionService_ClampInt32(value, 50, 3000);
        break;
    case INTERACTION_SETTING_Q1_SEG3:
        g_app_route_segment3_mm = InteractionService_ClampInt32(value, 50, 3000);
        break;
    case INTERACTION_SETTING_Q1_ANGLE:
        g_app_route_turn_angle_deg10 = (int16_t)InteractionService_ClampInt32(value, 100, 1800);
        break;
    case INTERACTION_SETTING_Q1_LEAD:
        g_app_route_turn_entry_lead_mm = (int16_t)InteractionService_ClampInt32(value, 0, 600);
        break;
    case INTERACTION_SETTING_Q1_BOOST_SPEED:
        g_app_turn_exit_boost_mmps = (int16_t)InteractionService_ClampInt32(value, 0, 600);
        break;
    case INTERACTION_SETTING_Q1_BOOST_TICKS:
        g_app_turn_exit_boost_ticks = (uint8_t)InteractionService_ClampInt32(value, 0, 40);
        break;
    case INTERACTION_SETTING_Q1_TURN_DONE_YAW:
        g_app_route_turn_done_yaw_abs_mdps = (int16_t)InteractionService_ClampInt32(value, 20, 1200);
        break;
    case INTERACTION_SETTING_Q1_STRAIGHT_BIAS:
        g_app_straight_side_bias_x10 = (int16_t)InteractionService_ClampInt32(value, -50, 50);
        break;
    case INTERACTION_SETTING_COUNT:
    default:
        break;
    }
}

static void InteractionService_FillSpaces(char *buf, uint8_t len)
{
    uint8_t i;

    for (i = 0U; i < len; i++) {
        buf[i] = ' ';
    }
    buf[len] = '\0';
}

static void InteractionService_WriteLabel(char *buf, uint8_t pos, char label)
{
    if (pos < APP_UI_LINE_LEN) {
        buf[pos] = label;
    }
    if ((uint8_t)(pos + 1U) < APP_UI_LINE_LEN) {
        buf[pos + 1U] = ':';
    }
}

static void InteractionService_WriteSignedValue(char *buf, uint8_t pos, int32_t value, uint8_t width)
{
    char temp[12];
    uint8_t digit_count = 0U;
    uint8_t out_width = width;
    uint32_t magnitude;

    if (width == 0U) {
        return;
    }

    if (value < 0) {
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        temp[digit_count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while ((magnitude > 0U) && (digit_count < sizeof(temp)));

    if ((value < 0) && (digit_count < sizeof(temp))) {
        temp[digit_count++] = '-';
    }

    if (digit_count > out_width) {
        digit_count = out_width;
    }

    while ((out_width > digit_count) && (pos < APP_UI_LINE_LEN)) {
        buf[pos++] = ' ';
        out_width--;
    }

    while ((digit_count > 0U) && (pos < APP_UI_LINE_LEN)) {
        buf[pos++] = temp[--digit_count];
    }
}

static void InteractionService_CopyLine(char *dst, const char *src)
{
    uint8_t i;
    uint8_t end_of_string = 0U;

    for (i = 0U; i < APP_UI_LINE_LEN; i++) {
        if (!end_of_string && (src != 0) && (src[i] != '\0')) {
            dst[i] = src[i];
        } else {
            dst[i] = ' ';
            end_of_string = 1U;
        }
    }
    dst[APP_UI_LINE_LEN] = '\0';
}

static void InteractionService_SetPage(const char *l1, const char *l2, const char *l3, const char *l4)
{
    InteractionPage_t next_page;

    memset(&next_page, 0, sizeof(next_page));
    next_page.visible = 1U;
    InteractionService_CopyLine(next_page.lines[0], l1);
    InteractionService_CopyLine(next_page.lines[1], l2);
    InteractionService_CopyLine(next_page.lines[2], l3);
    InteractionService_CopyLine(next_page.lines[3], l4);

    if ((g_interaction.page.visible == next_page.visible) &&
        (memcmp(g_interaction.page.lines, next_page.lines, sizeof(next_page.lines)) == 0)) {
        return;
    }

    g_interaction.page.visible = 1U;
    memcpy(g_interaction.page.lines, next_page.lines, sizeof(next_page.lines));
    g_interaction.dirty = 1U;
}

static void InteractionService_UpdateRuntimePage(void)
{
    char line2[APP_UI_LINE_LEN + 1U];
    char line3[APP_UI_LINE_LEN + 1U];
    char line4[APP_UI_LINE_LEN + 1U];
    TaskService_Status_t task_status;
    MotionControl_Status_t motion_status;
    DeviceCenter_Snapshot_t device_snapshot;
    const char *title;

    TaskService_GetStatus(&task_status);
    MotionControl_GetStatus(&motion_status);
    DeviceCenter_GetSnapshot(&device_snapshot);

    switch (g_interaction.current_action) {
    case INTERACTION_ACTION_RUN_MODE_A:
        title = "Run Mode A";
        snprintf(line2, sizeof(line2), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        snprintf(line3, sizeof(line3), "Bhv:%-9s", BehaviorMotion_GetName());
        snprintf(line4, sizeof(line4), "Q:%u  P:%3u%%", task_status.queue_depth, task_status.progress);
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_RUN_MODE_B:
        title = "Run Mode B";
        snprintf(line2, sizeof(line2), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        snprintf(line3, sizeof(line3), "Bhv:%-9s", BehaviorMotion_GetName());
        snprintf(line4, sizeof(line4), "Q:%u  P:%3u%%", task_status.queue_depth, task_status.progress);
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_STRAIGHT_TEST:
        title = "Straight Test";
        snprintf(line2, sizeof(line2), "H:%5d Y:%4d",
            (int)device_snapshot.imu_heading_deg10,
            (int)device_snapshot.imu_yaw_mdps);
        snprintf(line3, sizeof(line3), "V:%4d %4d",
            (int)(device_snapshot.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] / 10),
            (int)(device_snapshot.side_speed_mmps[APP_BOARD_SIDE_RIGHT_INDEX] / 10));
        snprintf(line4, sizeof(line4), "O:%4d %4d",
            (int)motion_status.left_duty,
            (int)motion_status.right_duty);
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_ARC_TEST:
        title = "Arc Test";
        snprintf(line2, sizeof(line2), "H:%5d Y:%4d",
            (int)device_snapshot.imu_heading_deg10,
            (int)device_snapshot.imu_yaw_mdps);
        snprintf(line3, sizeof(line3), "S:%5d P:%3u%%",
            (int)device_snapshot.chassis_distance_mm,
            (unsigned)task_status.progress);
        snprintf(line4, sizeof(line4), "R:%4d A:%4d",
            (int)AppConfig_GetArcTestRadiusMm(),
            (int)((AppConfig_GetArcTestAngleDeg10() >= 0) ?
                AppConfig_GetArcTestAngleDeg10() :
                -AppConfig_GetArcTestAngleDeg10()));
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_GO_1000MM:
        title = "Go 1000mm";
        snprintf(line2, sizeof(line2), "S:%5d T:%4d",
            (int)device_snapshot.chassis_distance_mm,
            (int)APP_DISTANCE_TEST_TARGET_MM);
        snprintf(line3, sizeof(line3), "E:%5d V:%4d",
            (int)(APP_DISTANCE_TEST_TARGET_MM - device_snapshot.chassis_distance_mm),
            (int)(device_snapshot.side_speed_mmps[APP_BOARD_SIDE_LEFT_INDEX] / 10));
        snprintf(line4, sizeof(line4), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_ROUTE_2500_L90_500_R90_500:
        title = "Route 250-L-R";
        snprintf(line2, sizeof(line2), "S:%5d H:%4d",
            (int)device_snapshot.chassis_distance_mm,
            (int)device_snapshot.imu_heading_deg10);
        snprintf(line3, sizeof(line3), "Step:%u P:%3u%%",
            (unsigned)task_status.phase_index,
            (unsigned)task_status.progress);
        snprintf(line4, sizeof(line4), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT:
        title = "Q2 L-R-L";
        snprintf(line2, sizeof(line2), "S:%5d H:%4d",
            (int)device_snapshot.chassis_distance_mm,
            (int)device_snapshot.imu_heading_deg10);
        snprintf(line3, sizeof(line3), "Step:%u P:%3u%%",
            (unsigned)task_status.phase_index,
            (unsigned)task_status.progress);
        snprintf(line4, sizeof(line4), "45/45/45/22");
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT:
        title = "Q2 R-L-R";
        snprintf(line2, sizeof(line2), "S:%5d H:%4d",
            (int)device_snapshot.chassis_distance_mm,
            (int)device_snapshot.imu_heading_deg10);
        snprintf(line3, sizeof(line3), "Step:%u P:%3u%%",
            (unsigned)task_status.phase_index,
            (unsigned)task_status.progress);
        snprintf(line4, sizeof(line4), "45/45/45/22");
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_Q3_1200_R270_500_R420_1800:
        title = "Q3 Arc R100";
        snprintf(line2, sizeof(line2), "S:%5d H:%4d",
            (int)device_snapshot.chassis_distance_mm,
            (int)device_snapshot.imu_heading_deg10);
        snprintf(line3, sizeof(line3), "Step:%u P:%3u%%",
            (unsigned)task_status.phase_index,
            (unsigned)task_status.progress);
        snprintf(line4, sizeof(line4), "12 A270 5 A420");
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_MOTOR_TEST_1:
        title = "M1 Q50 Count";
        snprintf(line2, sizeof(line2), "A:%4d B:%4d",
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_REAR_RIGHT_INDEX],
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX]);
        snprintf(line3, sizeof(line3), "C:%4d D:%4d",
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_REAR_LEFT_INDEX],
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_FRONT_LEFT_INDEX]);
        snprintf(line4, sizeof(line4), "L:%4d R:%4d",
            (int)device_snapshot.motor_left_duty,
            (int)device_snapshot.motor_right_duty);
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_MOTOR_TEST_2:
        title = "M2 Q50 Open";
        snprintf(line2, sizeof(line2), "A:%4d B:%4d",
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_REAR_RIGHT_INDEX],
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX]);
        snprintf(line3, sizeof(line3), "C:%4d D:%4d",
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_REAR_LEFT_INDEX],
            (int)device_snapshot.wheel_speed_window_counts[APP_BOARD_WHEEL_FRONT_LEFT_INDEX]);
        snprintf(line4, sizeof(line4), "L:%4d R:%4d",
            (int)device_snapshot.motor_left_duty,
            (int)device_snapshot.motor_right_duty);
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_TURN_LEFT_90:
        title = "Turn Left 90";
        snprintf(line2, sizeof(line2), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        snprintf(line3, sizeof(line3), "H:%5d Y:%5d",
            (int)device_snapshot.imu_heading_deg10,
            (int)device_snapshot.imu_yaw_mdps);
        snprintf(line4, sizeof(line4), "Last:%-9s", TaskService_GetActionName(task_status.last_completed_action));
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_TURN_RIGHT_90:
        title = "Turn Right90";
        snprintf(line2, sizeof(line2), "Task:%-8s", TaskService_GetStageName(task_status.stage));
        snprintf(line3, sizeof(line3), "H:%5d Y:%5d",
            (int)device_snapshot.imu_heading_deg10,
            (int)device_snapshot.imu_yaw_mdps);
        snprintf(line4, sizeof(line4), "Last:%-9s", TaskService_GetActionName(task_status.last_completed_action));
        InteractionService_SetPage(title, line2, line3, line4);
        break;
    case INTERACTION_ACTION_SENSOR_PAGE:
        InteractionService_FillSpaces(line2, APP_UI_LINE_LEN);
        InteractionService_FillSpaces(line3, APP_UI_LINE_LEN);
        InteractionService_FillSpaces(line4, APP_UI_LINE_LEN);

        InteractionService_WriteLabel(line2, 0U, 'A');
        InteractionService_WriteSignedValue(line2, 2U,
            device_snapshot.wheel_ticks[APP_BOARD_WHEEL_REAR_RIGHT_INDEX], 4U);
        InteractionService_WriteLabel(line2, 8U, 'B');
        InteractionService_WriteSignedValue(line2, 10U,
            device_snapshot.wheel_ticks[APP_BOARD_WHEEL_FRONT_RIGHT_INDEX], 4U);

        InteractionService_WriteLabel(line3, 0U, 'C');
        InteractionService_WriteSignedValue(line3, 2U,
            device_snapshot.wheel_ticks[APP_BOARD_WHEEL_REAR_LEFT_INDEX], 4U);
        InteractionService_WriteLabel(line3, 8U, 'D');
        InteractionService_WriteSignedValue(line3, 10U,
            device_snapshot.wheel_ticks[APP_BOARD_WHEEL_FRONT_LEFT_INDEX], 4U);

        InteractionService_WriteLabel(line4, 0U, 'S');
        InteractionService_WriteSignedValue(line4, 2U, device_snapshot.chassis_distance_mm, 5U);
        InteractionService_WriteLabel(line4, 9U, 'H');
        InteractionService_WriteSignedValue(line4, 11U, device_snapshot.encoder_heading_deg10, 4U);
        InteractionService_SetPage("ODOM", line2, line3, line4);
        break;
    case INTERACTION_ACTION_SETTINGS_PAGE:
        InteractionService_RenderSettingsPage();
        break;
    case INTERACTION_ACTION_ARC_SETTINGS_PAGE:
        InteractionService_RenderSettingsPage();
        break;
    case INTERACTION_ACTION_ABOUT_PAGE:
        InteractionService_SetPage("About", "MSPM0 Layered", "Non Blocking", "EDC 2026 Ready");
        break;
    case INTERACTION_ACTION_NONE:
    default:
        break;
    }
}

void InteractionService_Init(void)
{
    memset(&g_interaction, 0, sizeof(g_interaction));
    g_interaction.last_refresh_ms = 0U;
}

void InteractionService_RequestAction(InteractionAction_e action)
{
    uint8_t queued = 1U;

    g_interaction.current_action = action;

    switch (action) {
    case INTERACTION_ACTION_RUN_MODE_A:
        queued = TaskService_Enqueue(TASK_ACTION_RUN_MODE_A);
        break;
    case INTERACTION_ACTION_RUN_MODE_B:
        queued = TaskService_Enqueue(TASK_ACTION_RUN_MODE_B);
        break;
    case INTERACTION_ACTION_STRAIGHT_TEST:
        queued = TaskService_Enqueue(TASK_ACTION_STRAIGHT_TEST);
        break;
    case INTERACTION_ACTION_ARC_TEST:
        queued = TaskService_Enqueue(TASK_ACTION_ARC_TEST);
        break;
    case INTERACTION_ACTION_GO_1000MM:
        queued = TaskService_Enqueue(TASK_ACTION_GO_1000MM);
        break;
    case INTERACTION_ACTION_ROUTE_2500_L90_500_R90_500:
        queued = TaskService_Enqueue(TASK_ACTION_ROUTE_2500_L90_500_R90_500);
        break;
    case INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT:
        queued = TaskService_Enqueue(TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT);
        break;
    case INTERACTION_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT:
        queued = TaskService_Enqueue(TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT);
        break;
    case INTERACTION_ACTION_Q3_1200_R270_500_R420_1800:
        queued = TaskService_Enqueue(TASK_ACTION_Q3_1200_R270_500_R420_1800);
        break;
    case INTERACTION_ACTION_MOTOR_TEST_1:
        queued = TaskService_Enqueue(TASK_ACTION_MOTOR_TEST_1);
        break;
    case INTERACTION_ACTION_MOTOR_TEST_2:
        queued = TaskService_Enqueue(TASK_ACTION_MOTOR_TEST_2);
        break;
    case INTERACTION_ACTION_TURN_LEFT_90:
        queued = TaskService_Enqueue(TASK_ACTION_TURN_LEFT_90);
        break;
    case INTERACTION_ACTION_TURN_RIGHT_90:
        queued = TaskService_Enqueue(TASK_ACTION_TURN_RIGHT_90);
        break;
    case INTERACTION_ACTION_SENSOR_PAGE:
        queued = TaskService_Enqueue(TASK_ACTION_SENSOR_SCAN);
        break;
    case INTERACTION_ACTION_SETTINGS_PAGE:
        g_settings_page = INTERACTION_SETTINGS_PAGE_Q1;
        g_settings_index = 0U;
        queued = 1U;
        break;
    case INTERACTION_ACTION_ARC_SETTINGS_PAGE:
        g_settings_page = INTERACTION_SETTINGS_PAGE_ARC;
        g_settings_index = 0U;
        queued = 1U;
        break;
    case INTERACTION_ACTION_ABOUT_PAGE:
        queued = 1U;
        break;
    case INTERACTION_ACTION_NONE:
    default:
        queued = 0U;
        break;
    }

    if (!queued) {
        g_interaction.current_action = INTERACTION_ACTION_NONE;
        InteractionService_SetPage("Task Queue Full", "Try Again", "K3/K4 Return", APP_NAME);
        return;
    }

    DeviceCenter_PlayConfirm();
    g_interaction.last_refresh_ms = AppTick_GetMs();
    InteractionService_UpdateRuntimePage();
}

void InteractionService_Tick50ms(void)
{
    if (!g_interaction.page.visible) {
        return;
    }

    if (!AppTick_IsExpired(g_interaction.last_refresh_ms, INTERACTION_REFRESH_PERIOD_MS)) {
        return;
    }

    g_interaction.last_refresh_ms = AppTick_GetMs();
    InteractionService_UpdateRuntimePage();
}

void InteractionService_ClosePage(void)
{
    if (g_interaction.current_action != INTERACTION_ACTION_NONE) {
        TaskService_CancelCurrent();
    }
    g_interaction.page.visible = 0U;
    g_interaction.current_action = INTERACTION_ACTION_NONE;
    g_interaction.dirty = 0U;
}

uint8_t InteractionService_FetchPage(InteractionPage_t *page, uint8_t force_copy)
{
    if ((page == 0) || !g_interaction.page.visible) {
        return 0U;
    }

    if (!g_interaction.dirty && !force_copy) {
        return 0U;
    }

    *page = g_interaction.page;
    g_interaction.dirty = 0U;
    return 1U;
}

InteractionAction_e InteractionService_GetCurrentAction(void)
{
    return g_interaction.current_action;
}

void InteractionService_SettingsAdjust(int8_t delta)
{
    if (((g_interaction.current_action != INTERACTION_ACTION_SETTINGS_PAGE) &&
         (g_interaction.current_action != INTERACTION_ACTION_ARC_SETTINGS_PAGE)) ||
        (delta == 0)) {
        return;
    }

    InteractionService_AdjustCurrentSetting(delta);
    InteractionService_UpdateRuntimePage();
}

void InteractionService_SettingsNext(void)
{
    uint8_t item_count;

    if ((g_interaction.current_action != INTERACTION_ACTION_SETTINGS_PAGE) &&
        (g_interaction.current_action != INTERACTION_ACTION_ARC_SETTINGS_PAGE)) {
        return;
    }

    item_count = (g_settings_page == INTERACTION_SETTINGS_PAGE_ARC) ? 3U :
        (uint8_t)INTERACTION_SETTING_COUNT;
    g_settings_index++;
    if (g_settings_index >= item_count) {
        g_settings_index = 0U;
    }

    InteractionService_UpdateRuntimePage();
}
