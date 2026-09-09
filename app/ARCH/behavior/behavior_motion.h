#ifndef APP_ARCH_BEHAVIOR_MOTION_H
#define APP_ARCH_BEHAVIOR_MOTION_H

#include <stdint.h>

#include "ARCH/config/app_config.h"

/**
 * @brief 运动行为类型
 * @note 该枚举描述任务层希望底盘执行的高层动作语义。
 */
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

/**
 * @brief 运动控制命令快照
 * @note 任务层通过填充该结构体中的目标量和开关位，描述本周期的控制意图。
 */
typedef struct
{
    MotionBehavior_e behavior;           /**< 当前行为模式 */
    int16_t speed_target_mmps;           /**< 目标速度，单位 mm/s */
    int32_t distance_target_mm;          /**< 目标距离，单位 mm */
    int16_t heading_target_deg10;        /**< 目标航向角，单位 0.1 度 */
    int16_t yaw_rate_target_mdps;        /**< 目标角速度，单位 mdps */
    int16_t arc_radius_mm;               /**< 圆弧半径，单位 mm，符号表示左右方向 */
    int16_t line_target;                 /**< 巡线目标位置 */
    int16_t open_loop_left_duty;         /**< 左侧开环占空比 */
    int16_t open_loop_right_duty;        /**< 右侧开环占空比 */
    AppTrackTarget_e track_target;       /**< 期望寻找的黑/白线目标 */
    uint8_t speed_enable;                /**< 是否启用速度环 */
    uint8_t distance_enable;             /**< 是否启用距离环 */
    uint8_t attitude_enable;             /**< 是否启用航向环 */
    uint8_t turn_enable;                 /**< 是否启用转向差速控制 */
    uint8_t line_enable;                 /**< 是否启用巡线控制 */
    uint8_t open_loop_enable;            /**< 是否直接输出开环占空比 */
    uint8_t distance_relative;           /**< 距离目标是否相对当前里程 */
    uint8_t heading_lock_on_entry;       /**< 进入动作时是否锁定当前航向为目标航向 */
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
