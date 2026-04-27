#include "ARCH/behavior/behavior_motion.h"

static BehaviorMotion_Command_t g_motion_command;

static void BehaviorMotion_ClearCommand(void)
{
    g_motion_command.behavior              = MOTION_BEHAVIOR_IDLE;
    g_motion_command.speed_target_mmps     = 0;
    g_motion_command.distance_target_mm    = 0;
    g_motion_command.heading_target_deg10  = 0;
    g_motion_command.yaw_rate_target_mdps  = 0;
    g_motion_command.arc_radius_mm         = 0;
    g_motion_command.line_target           = 0;
    g_motion_command.open_loop_left_duty   = 0;
    g_motion_command.open_loop_right_duty  = 0;
    g_motion_command.track_target          = APP_TRACK_TARGET_BLACK;
    g_motion_command.speed_enable          = 0U;
    g_motion_command.distance_enable       = 0U;
    g_motion_command.attitude_enable       = 0U;
    g_motion_command.turn_enable           = 0U;
    g_motion_command.line_enable           = 0U;
    g_motion_command.open_loop_enable      = 0U;
    g_motion_command.distance_relative     = 0U;
    g_motion_command.heading_lock_on_entry = 0U;
}

void BehaviorMotion_Init(void)
{
    BehaviorMotion_ClearCommand();
}

void BehaviorMotion_Stop(void)
{
    BehaviorMotion_ClearCommand();
}

void BehaviorMotion_OpenLoop(int16_t left_duty, int16_t right_duty)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior            = MOTION_BEHAVIOR_OPEN_LOOP;
    g_motion_command.open_loop_left_duty = left_duty;
    g_motion_command.open_loop_right_duty = right_duty;
    g_motion_command.open_loop_enable    = 1U;
}

void BehaviorMotion_SpeedHold(int16_t speed_target_mmps)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior          = MOTION_BEHAVIOR_SPEED_HOLD;
    g_motion_command.speed_target_mmps = speed_target_mmps;
    g_motion_command.speed_enable      = 1U;
}

void BehaviorMotion_Straight(int16_t speed_target_mmps)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior              = MOTION_BEHAVIOR_STRAIGHT;
    g_motion_command.speed_target_mmps     = speed_target_mmps;
    g_motion_command.speed_enable          = 1U;
    g_motion_command.attitude_enable       = 1U;
    g_motion_command.turn_enable           = 1U;
    g_motion_command.heading_lock_on_entry = 1U;
}

void BehaviorMotion_StraightToHeading(int16_t speed_target_mmps, int16_t heading_target_deg10)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior             = MOTION_BEHAVIOR_STRAIGHT;
    g_motion_command.speed_target_mmps    = speed_target_mmps;
    g_motion_command.heading_target_deg10 = heading_target_deg10;
    g_motion_command.speed_enable         = 1U;
    g_motion_command.attitude_enable      = 1U;
    g_motion_command.turn_enable          = 1U;
    g_motion_command.heading_lock_on_entry = 0U;
}

void BehaviorMotion_StraightDistance(int16_t speed_target_mmps, int32_t distance_target_mm)
{
    BehaviorMotion_Straight(speed_target_mmps);
    g_motion_command.distance_target_mm = distance_target_mm;
    g_motion_command.distance_enable    = 1U;
    g_motion_command.distance_relative  = 1U;
}

void BehaviorMotion_StraightDistanceToHeading(int16_t speed_target_mmps,
    int32_t distance_target_mm,
    int16_t heading_target_deg10)
{
    BehaviorMotion_StraightToHeading(speed_target_mmps, heading_target_deg10);
    g_motion_command.distance_target_mm = distance_target_mm;
    g_motion_command.distance_enable    = 1U;
    g_motion_command.distance_relative  = 1U;
}

void BehaviorMotion_Arc(int16_t speed_target_mmps, int16_t radius_mm)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior             = MOTION_BEHAVIOR_ARC;
    g_motion_command.speed_target_mmps    = speed_target_mmps;
    g_motion_command.arc_radius_mm        = radius_mm;
    g_motion_command.speed_enable         = 1U;
    g_motion_command.turn_enable          = 1U;
}

void BehaviorMotion_ArcToHeading(int16_t speed_target_mmps,
    int16_t radius_mm,
    int16_t heading_target_deg10)
{
    BehaviorMotion_Arc(speed_target_mmps, radius_mm);
    g_motion_command.heading_target_deg10 = heading_target_deg10;
    g_motion_command.attitude_enable      = 1U;
    g_motion_command.heading_lock_on_entry = 0U;
}

void BehaviorMotion_ArcDistance(int16_t speed_target_mmps,
    int16_t radius_mm,
    int32_t distance_target_mm)
{
    BehaviorMotion_Arc(speed_target_mmps, radius_mm);
    g_motion_command.distance_target_mm = distance_target_mm;
    g_motion_command.distance_enable    = 1U;
    g_motion_command.distance_relative  = 1U;
}

void BehaviorMotion_ArcByRadius(int16_t speed_target_mmps, int16_t radius_mm)
{
    BehaviorMotion_Arc(speed_target_mmps, radius_mm);
}

void BehaviorMotion_ArcDistanceByRadius(int16_t speed_target_mmps,
    int16_t radius_mm,
    int32_t distance_target_mm)
{
    BehaviorMotion_ArcDistance(speed_target_mmps, radius_mm, distance_target_mm);
}

void BehaviorMotion_Turn(int16_t yaw_target_mdps)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior             = MOTION_BEHAVIOR_TURN;
    g_motion_command.yaw_rate_target_mdps = yaw_target_mdps;
    g_motion_command.turn_enable          = 1U;
}

void BehaviorMotion_TurnToHeading(int16_t heading_target_deg10)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior            = MOTION_BEHAVIOR_TURN;
    g_motion_command.heading_target_deg10 = heading_target_deg10;
    g_motion_command.attitude_enable     = 1U;
}

void BehaviorMotion_LineFollow(int16_t speed_target_mmps)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior          = MOTION_BEHAVIOR_LINE_FOLLOW;
    g_motion_command.speed_target_mmps = speed_target_mmps;
    g_motion_command.speed_enable      = 1U;
    g_motion_command.turn_enable       = 1U;
    g_motion_command.line_enable       = 1U;
}

void BehaviorMotion_GoToLine(AppTrackTarget_e track_target, int16_t speed_target_mmps)
{
    BehaviorMotion_ClearCommand();
    g_motion_command.behavior          = MOTION_BEHAVIOR_GO_TO_LINE;
    g_motion_command.speed_target_mmps = speed_target_mmps;
    g_motion_command.track_target      = track_target;
    g_motion_command.speed_enable      = 1U;
    g_motion_command.turn_enable       = 1U;
    g_motion_command.line_enable       = 1U;
}

void BehaviorMotion_GetCommand(BehaviorMotion_Command_t *command)
{
    if (command == 0) {
        return;
    }

    *command = g_motion_command;
}

const char *BehaviorMotion_GetName(void)
{
    switch (g_motion_command.behavior) {
    case MOTION_BEHAVIOR_OPEN_LOOP:
        return "OpenLoop";
    case MOTION_BEHAVIOR_SPEED_HOLD:
        return "SpeedHold";
    case MOTION_BEHAVIOR_STRAIGHT:
        return "Straight";
    case MOTION_BEHAVIOR_ARC:
        return "Arc";
    case MOTION_BEHAVIOR_TURN:
        return "Turn";
    case MOTION_BEHAVIOR_LINE_FOLLOW:
        return "Track";
    case MOTION_BEHAVIOR_GO_TO_LINE:
        return (g_motion_command.track_target == APP_TRACK_TARGET_BLACK) ? "To Black" : "To White";
    case MOTION_BEHAVIOR_IDLE:
    default:
        return "Idle";
    }
}
