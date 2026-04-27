#ifndef APP_ARCH_BEHAVIOR_MOTION_H
#define APP_ARCH_BEHAVIOR_MOTION_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

typedef enum
{
    MOTION_BEHAVIOR_IDLE = 0,
    MOTION_BEHAVIOR_OPEN_LOOP,
    MOTION_BEHAVIOR_SPEED_HOLD,
    MOTION_BEHAVIOR_STRAIGHT,
    MOTION_BEHAVIOR_ARC,
    MOTION_BEHAVIOR_TURN,
    MOTION_BEHAVIOR_LINE_FOLLOW,
    MOTION_BEHAVIOR_GO_TO_LINE,
} MotionBehavior_e;

typedef struct
{
    MotionBehavior_e behavior;
    int16_t speed_target_mmps;
    int32_t distance_target_mm;
    int16_t heading_target_deg10;
    int16_t yaw_rate_target_mdps;
    int16_t arc_radius_mm;
    int16_t line_target;
    int16_t open_loop_left_duty;
    int16_t open_loop_right_duty;
    AppTrackTarget_e track_target;
    uint8_t speed_enable;
    uint8_t distance_enable;
    uint8_t attitude_enable;
    uint8_t turn_enable;
    uint8_t line_enable;
    uint8_t open_loop_enable;
    uint8_t distance_relative;
    uint8_t heading_lock_on_entry;
} BehaviorMotion_Command_t;

void BehaviorMotion_Init(void);
void BehaviorMotion_Stop(void);
void BehaviorMotion_OpenLoop(int16_t left_duty, int16_t right_duty);
void BehaviorMotion_SpeedHold(int16_t speed_target_mmps);
void BehaviorMotion_Straight(int16_t speed_target_mmps);
void BehaviorMotion_StraightDistance(int16_t speed_target_mmps, int32_t distance_target_mm);
void BehaviorMotion_StraightToHeading(int16_t speed_target_mmps, int16_t heading_target_deg10);
void BehaviorMotion_StraightDistanceToHeading(int16_t speed_target_mmps,
    int32_t distance_target_mm,
    int16_t heading_target_deg10);
void BehaviorMotion_Arc(int16_t speed_target_mmps, int16_t radius_mm);
void BehaviorMotion_ArcToHeading(int16_t speed_target_mmps,
    int16_t radius_mm,
    int16_t heading_target_deg10);
void BehaviorMotion_ArcDistance(int16_t speed_target_mmps,
    int16_t radius_mm,
    int32_t distance_target_mm);
void BehaviorMotion_ArcByRadius(int16_t speed_target_mmps, int16_t radius_mm);
void BehaviorMotion_ArcDistanceByRadius(int16_t speed_target_mmps,
    int16_t radius_mm,
    int32_t distance_target_mm);
void BehaviorMotion_Turn(int16_t yaw_target_mdps);
void BehaviorMotion_TurnToHeading(int16_t heading_target_deg10);
void BehaviorMotion_LineFollow(int16_t speed_target_mmps);
void BehaviorMotion_GoToLine(AppTrackTarget_e track_target, int16_t speed_target_mmps);
void BehaviorMotion_GetCommand(BehaviorMotion_Command_t *command);
const char *BehaviorMotion_GetName(void);

#endif
