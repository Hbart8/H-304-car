#ifndef APP_ARCH_CONFIG_H
#define APP_ARCH_CONFIG_H

#include <stdint.h>

#include "ARCH/config/board_profile.h"

#define APP_NAME                                "EDC Control Project"

#define APP_UI_LINE_LEN                         (16U)
#define APP_TASK_QUEUE_DEPTH                    (8U)
#define APP_SCHEDULER_MAX_TASKS                 (8U)

#define APP_SCHED_PERIOD_CONTROL_MS             (10U)     /**< 控制循环周期 10ms */
#define APP_SCHED_PERIOD_DEVICE_MS              (10U)     /**< 设备读取周期 10ms */
#define APP_SCHED_PERIOD_TASK_MS                (10U)     /**< 任务调度周期 10ms */
#define APP_SCHED_PERIOD_INTERACTION_MS         (50U)     /**< 人机交互(按键/显示)周期 50ms */
#define APP_SCHED_PERIOD_DEBUG_MS               (300U)    /**< 调试信息打印周期 300ms */
#define APP_DEBUG_STARTUP_MUTE_MS              (1200U)    /**< 启动后静音(不打印)时间 1200ms */
#define APP_SCHED_PERIOD_HAL_MS                 (1U)      /**< 硬件抽象层周期 1ms */

#define APP_DEBUG_KEY_MONITOR                   (0U)

#define APP_PI                                  (3.1415926f)
#define APP_CONTROL_DT_S                        (0.010f)

#define APP_UART_DEBUG_BAUD                     (9600U)
#define APP_UART_TX_BUFFER_SIZE                 (256U)

#define APP_GYRO_CALIBRATE_SAMPLE_COUNT         (128U)    /**< 陀螺仪校准采样次数 */
#define APP_GYRO_RAW_DEADBAND                   (6)       /**< 陀螺仪原始数据死区 */
#define APP_GYRO_BIAS_TRACK_WINDOW_RAW          (8)       /**< 陀螺仪零漂追踪窗口大小 */
#define APP_GYRO_BIAS_TRACK_COUNT               (20U)     /**< 陀螺仪零漂追踪次数 */
#define APP_GYRO_YAW_FILTER_DIV                 (4)       /**< 偏航角滤波除数 */
#define APP_GYRO_YAW_DEADBAND_DPS10            (3)        /**< 偏航角角速度死区 (0.1 degree/s) */
#define APP_GYRO_HEADING_INTEGRATE_DPS10       (8)        /**< 航向角积分阈值 (0.1 degree/s) */
#define APP_GYRO_HEADING_ACTIVE_COUNT          (3U)       /**< 航向角激活计数 */

#define APP_TRACK_SENSOR_COUNT                  (8U)
#define APP_TRACK_THRESHOLD_DEFAULT             (2048U)

/*
 * Vehicle geometry and measured encoder calibration.
 * 底盘几何参数和编码器校准参数。
 * 轮子顺序假设：
 * pair0 = 右前轮, pair1 = 右后轮,
 * pair2 = 左前轮, pair3 = 左后轮.
 */
#define APP_CHASSIS_WHEEL_DIAMETER_MM           (65.0f)   /**< 轮子直径 65mm */
#define APP_CHASSIS_WHEEL_BASE_MM               (120.0f)  /**< 轮距 120mm */
#define APP_CHASSIS_GEAR_RATIO                  (28.0f)   /**< 减速比 1:28 */

#define APP_ENCODER_PAIR0_COUNT_PER_REV         (220.0f)  /**< 右前轮每圈脉冲数 */
#define APP_ENCODER_PAIR1_COUNT_PER_REV         (224.0f)  /**< 右后轮每圈脉冲数 */
#define APP_ENCODER_PAIR2_COUNT_PER_REV         (224.0f)  /**< 左前轮每圈脉冲数 */
#define APP_ENCODER_PAIR3_COUNT_PER_REV         (223.0f)  /**< 左后轮每圈脉冲数 */

#define APP_ENCODER_COUNT_PER_REV_AVG           ((APP_ENCODER_PAIR0_COUNT_PER_REV + \
                                                  APP_ENCODER_PAIR1_COUNT_PER_REV + \
                                                  APP_ENCODER_PAIR2_COUNT_PER_REV + \
                                                  APP_ENCODER_PAIR3_COUNT_PER_REV) / 4.0f)

#define APP_ENCODER_FRONT_RIGHT_COUNT_PER_REV    (APP_ENCODER_PAIR0_COUNT_PER_REV)
#define APP_ENCODER_REAR_RIGHT_COUNT_PER_REV     (APP_ENCODER_PAIR1_COUNT_PER_REV)
#define APP_ENCODER_FRONT_LEFT_COUNT_PER_REV     (APP_ENCODER_PAIR2_COUNT_PER_REV)
#define APP_ENCODER_REAR_LEFT_COUNT_PER_REV      (APP_ENCODER_PAIR3_COUNT_PER_REV)

#define APP_ENCODER_LEFT_COUNT_PER_REV           ((APP_ENCODER_FRONT_LEFT_COUNT_PER_REV + APP_ENCODER_REAR_LEFT_COUNT_PER_REV) * 0.5f)
#define APP_ENCODER_RIGHT_COUNT_PER_REV          ((APP_ENCODER_FRONT_RIGHT_COUNT_PER_REV + APP_ENCODER_REAR_RIGHT_COUNT_PER_REV) * 0.5f)

#define APP_WHEEL_CIRCUMFERENCE_MM              (APP_PI * APP_CHASSIS_WHEEL_DIAMETER_MM)
#define APP_CHASSIS_TURN_CIRCUMFERENCE_MM       (APP_PI * APP_CHASSIS_WHEEL_BASE_MM)

#define APP_ENCODER_PAIR0_MM_PER_COUNT          (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_PAIR0_COUNT_PER_REV)
#define APP_ENCODER_PAIR1_MM_PER_COUNT          (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_PAIR1_COUNT_PER_REV)
#define APP_ENCODER_PAIR2_MM_PER_COUNT          (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_PAIR2_COUNT_PER_REV)
#define APP_ENCODER_PAIR3_MM_PER_COUNT          (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_PAIR3_COUNT_PER_REV)
#define APP_ENCODER_MM_PER_COUNT_AVG            (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_COUNT_PER_REV_AVG)
#define APP_ENCODER_FRONT_RIGHT_MM_PER_COUNT    (APP_ENCODER_PAIR0_MM_PER_COUNT)
#define APP_ENCODER_REAR_RIGHT_MM_PER_COUNT     (APP_ENCODER_PAIR1_MM_PER_COUNT)
#define APP_ENCODER_FRONT_LEFT_MM_PER_COUNT     (APP_ENCODER_PAIR2_MM_PER_COUNT)
#define APP_ENCODER_REAR_LEFT_MM_PER_COUNT      (APP_ENCODER_PAIR3_MM_PER_COUNT)
#define APP_ENCODER_LEFT_MM_PER_COUNT           (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_LEFT_COUNT_PER_REV)
#define APP_ENCODER_RIGHT_MM_PER_COUNT          (APP_WHEEL_CIRCUMFERENCE_MM / APP_ENCODER_RIGHT_COUNT_PER_REV)

#define APP_ENCODER_PAIR0_COUNT_PER_MM          (APP_ENCODER_PAIR0_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_PAIR1_COUNT_PER_MM          (APP_ENCODER_PAIR1_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_PAIR2_COUNT_PER_MM          (APP_ENCODER_PAIR2_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_PAIR3_COUNT_PER_MM          (APP_ENCODER_PAIR3_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_COUNT_PER_MM_AVG            (APP_ENCODER_COUNT_PER_REV_AVG / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_LEFT_COUNT_PER_MM           (APP_ENCODER_LEFT_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)
#define APP_ENCODER_RIGHT_COUNT_PER_MM          (APP_ENCODER_RIGHT_COUNT_PER_REV / APP_WHEEL_CIRCUMFERENCE_MM)

#define APP_ODOM_DISTANCE_SCALE                 (0.1562f)

#define APP_CHASSIS_DEG_PER_MM_DIFF             (360.0f / APP_CHASSIS_TURN_CIRCUMFERENCE_MM)
#define APP_CHASSIS_DEG10_PER_MM_DIFF           (APP_CHASSIS_DEG_PER_MM_DIFF * 10.0f)

#define APP_MOTOR_DUTY_LIMIT                    (1000)
#define APP_MOTOR_SPEED_TO_DUTY_GAIN            (2.8f)
#define APP_MOTOR_SPEED_FEEDFORWARD_GAIN        (APP_MOTOR_SPEED_TO_DUTY_GAIN)
#define APP_MOTOR_OUTPUT_SLEW_STEP              (140)
#define APP_MOTOR_OUTPUT_SLEW_REVERSE_STEP      (320)

/*
 * Four-wheel motor output compensation lives here so that later PID and
 * behavior layers can reuse the same calibrated execution path.
 * 四轮电机输出补偿，用于修正不同电机的机械差异。
 */
#define APP_MOTOR_REAR_RIGHT_DUTY_SCALE         (1.00f)   /**< 右后轮输出占空比缩放系数 */
#define APP_MOTOR_FRONT_RIGHT_DUTY_SCALE        (1.00f)   /**< 右前轮输出占空比缩放系数 */
#define APP_MOTOR_REAR_LEFT_DUTY_SCALE          (0.99f)   /**< 左后轮输出占空比缩放系数 */
#define APP_MOTOR_FRONT_LEFT_DUTY_SCALE         (1.01f)   /**< 左前轮输出占空比缩放系数 */

#define APP_MOTOR_REAR_RIGHT_DUTY_TRIM          (0)       /**< 右后轮输出占空比微调常数 */
#define APP_MOTOR_FRONT_RIGHT_DUTY_TRIM         (0)       /**< 右前轮输出占空比微调常数 */
#define APP_MOTOR_REAR_LEFT_DUTY_TRIM           (0)       /**< 左后轮输出占空比微调常数 */
#define APP_MOTOR_FRONT_LEFT_DUTY_TRIM          (0)       /**< 左前轮输出占空比微调常数 */

#define APP_MOTOR_REAR_RIGHT_START_DUTY         (0)       /**< 右后轮启动死区占空比 */
#define APP_MOTOR_FRONT_RIGHT_START_DUTY        (0)       /**< 右前轮启动死区占空比 */
#define APP_MOTOR_REAR_LEFT_START_DUTY          (0)       /**< 左后轮启动死区占空比 */
#define APP_MOTOR_FRONT_LEFT_START_DUTY         (0)       /**< 左前轮启动死区占空比 */

#define APP_MOTOR_TEST_SPEED_MMPS               (120)
#define APP_MOTOR_TEST_TURN_DUTY                (320)
#define APP_STRAIGHT_TEST_SPEED_MMPS            (220)
#define APP_ARC_TEST_SPEED_MMPS                 (180)
#define APP_ARC_TEST_RADIUS_MM                  (250)
#define APP_ARC_TEST_ANGLE_DEG10                (3600)
#define APP_ARC_TEST_APPROACH_ANGLE_DEG10       (180)
#define APP_ARC_TEST_APPROACH_SPEED_MMPS        (150)
#define APP_ARC_TEST_DONE_ANGLE_ERR_DEG10       (20)
#define APP_ARC_TEST_DISTANCE_MARGIN_MM         (40)
#define APP_ARC_TEST_DONE_DISTANCE_FLOOR_X10    (7)
#define APP_ARC_TEST_DISTANCE_SAFETY_SCALE_X10  (30)
#define APP_ARC_HEADING_TRIM_ENTRY_DEG10        (900)
#define APP_ARC_HEADING_TRIM_MAX_YAW_MDPS       (1400)
#define APP_ARC_YAW_DEADBAND_MDPS               (20)
#define APP_ARC_RADIUS_TRIM_MAX_DIFF_MMPS       (50)
#define APP_DISTANCE_TEST_SPEED_MMPS            (180)
#define APP_DISTANCE_TEST_TARGET_MM             (1000)
#define APP_ROUTE_TEST_SPEED_MMPS               (320)
#define APP_ROUTE_SEGMENT1_MM                   (2400)
#define APP_ROUTE_SEGMENT2_MM                   (500)
#define APP_ROUTE_SEGMENT3_MM                   (500)
#define APP_ROUTE_TURN_ANGLE_DEG10              (375)
#define APP_Q2_SEGMENT1_MM                      (800)
#define APP_Q2_SEGMENT2_MM                      (800)
#define APP_Q2_SEGMENT3_MM                      (800)
#define APP_Q2_SEGMENT4_MM                      (800)
#define APP_Q2_SEGMENT5_MM                      (900)
#define APP_Q2_TURN1_ANGLE_DEG10                (223)
#define APP_Q2_TURN2_ANGLE_DEG10                (450)
#define APP_Q2_TURN3_ANGLE_DEG10                (450)
#define APP_Q2_TURN4_ANGLE_DEG10                (225)
#define APP_Q2_TURN_DONE_HEADING_ERR_DEG10      (25)
#define APP_Q2_TURN_DONE_YAW_ABS_MDPS           (420)
#define APP_Q2_TURN_DONE_HOLD_CYCLES            (1U)
#define APP_Q3_SEGMENT1_MM                      (1170)
#define APP_Q3_SEGMENT2_MM                      (500)
#define APP_Q3_SEGMENT3_MM                      (1800)
#define APP_Q3_FINAL_STABILIZE_MM               (120)
#define APP_Q3_ARC1_ENTRY_LEAD_MM              (160)
#define APP_Q3_ARC2_ENTRY_LEAD_MM              (120)
#define APP_Q3_ARC_RADIUS_MM                    (300)
#define APP_Q3_ARC1_ANGLE_DEG10                 (2427)
#define APP_Q3_ARC2_ANGLE_DEG10                 (4150)
#define APP_Q3_ARC_SPEED_MMPS                   (140)
#define APP_Q3_ARC_DONE_YAW_ABS_MDPS            (260)
#define APP_Q3_ARC_DONE_HOLD_CYCLES             (2U)
#define APP_Q3_TURN_DONE_HEADING_ERR_DEG10      (25)
#define APP_Q3_TURN_DONE_YAW_ABS_MDPS           (420)
#define APP_Q3_TURN_DONE_HOLD_CYCLES            (1U)
#define APP_Q2_ENTRY_ARC_ANGLE_DEG10            (260)
#define APP_Q2_ENTRY_RADIUS_SCALE_X100          (60)
#define APP_Q2_ENTRY_SPEED_OFFSET_MMPS          (20)
#define APP_ROUTE_TURN_ENTRY_LEAD_MM            (160)
#define APP_ROUTE_PHASE_DELAY_MS                (40U)
#define APP_TURN_EXIT_BOOST_MMPS                (300)
#define APP_TURN_EXIT_BOOST_TICKS               (12U)
#define APP_ARC_EXIT_RELEASE_TICKS              (18U)
#define APP_ARC_EXIT_RELEASE_YAW_MDPS           (160)
#define APP_ARC_EXIT_RELEASE_HOLD_TICKS         (4U)
#define APP_ROUTE_DISTANCE_DONE_ERR_MM          (35)
#define APP_ROUTE_DISTANCE_DONE_SPEED_MMPS      (300)
#define APP_ROUTE_DISTANCE_DONE_HOLD_CYCLES     (1U)
#define APP_ROUTE_TURN_DONE_HEADING_ERR_DEG10   (20)
#define APP_ROUTE_TURN_DONE_YAW_ABS_MDPS        (320)
#define APP_ROUTE_TURN_DONE_HOLD_CYCLES         (2U)
#define APP_ROUTE_TURN_STALL_HEADING_ERR_DEG10  (25)
#define APP_ROUTE_TURN_STALL_YAW_ABS_MDPS       (40)
#define APP_ROUTE_TURN_STALL_DELTA_DEG10        (1)
#define APP_ROUTE_TURN_STALL_HOLD_CYCLES        (8U)
#define APP_DISTANCE_APPROACH_BRAKE_MM          (120)
#define APP_DISTANCE_APPROACH_MIN_SPEED_MMPS    (150)
#define APP_DISTANCE_STOP_FREEZE_ERR_MM         (20)
#define APP_STRAIGHT_SIDE_BIAS_MMPS             (1.1f)
#define APP_STRAIGHT_SIDE_BIAS_X10_DEFAULT      (13)
#define APP_STRAIGHT_HEADING_DEADBAND_DEG10     (8)
#define APP_STRAIGHT_YAW_DEADBAND_MDPS          (25)
#define APP_STRAIGHT_DIFF_DEADBAND_MMPS         (8)
#define APP_DISTANCE_DONE_ERR_MM                (28)
#define APP_DISTANCE_DONE_SPEED_MMPS            (60)
#define APP_DISTANCE_DONE_HOLD_CYCLES           (5U)

#define APP_WHEEL_SPEED_AVG_WINDOW              (7U)
#define APP_WHEEL_SPEED_FILTER_OLD_WEIGHT       (0.90f)
#define APP_WHEEL_SPEED_FILTER_NEW_WEIGHT       (0.10f)

#define APP_WHEEL_SPEED_PID_KP                 (0.17f)    /**< 轮速控制 PID: 比例系数 P */
#define APP_WHEEL_SPEED_PID_KI                 (0.008f)   /**< 轮速控制 PID: 积分系数 I */
#define APP_WHEEL_SPEED_PID_KD                 (0.00f)    /**< 轮速控制 PID: 微分系数 D */
#define APP_WHEEL_SPEED_PID_OUT_MIN            (-180.0f)  /**< 轮速控制 PID: 最小输出限制 */
#define APP_WHEEL_SPEED_PID_OUT_MAX            (180.0f)   /**< 轮速控制 PID: 最大输出限制 */

#define APP_SPEED_PID_KP                        (1.20f)   /**< 整体速度控制 PID: 比例系数 P */
#define APP_SPEED_PID_KI                        (0.08f)   /**< 整体速度控制 PID: 积分系数 I */
#define APP_SPEED_PID_KD                        (0.02f)   /**< 整体速度控制 PID: 微分系数 D */
#define APP_SPEED_PID_OUT_MIN                   (-(float)APP_MOTOR_DUTY_LIMIT)  /**< 整体速度 PID: 最小输出 */
#define APP_SPEED_PID_OUT_MAX                   ((float)APP_MOTOR_DUTY_LIMIT)   /**< 整体速度 PID: 最大输出 */

#define APP_DISTANCE_PID_KP                     (0.75f)   /**< 距离(位置)控制 PID: 比例系数 P */
#define APP_DISTANCE_PID_KI                     (0.010f)  /**< 距离(位置)控制 PID: 积分系数 I */
#define APP_DISTANCE_PID_KD                     (0.00f)   /**< 距离(位置)控制 PID: 微分系数 D */
#define APP_DISTANCE_PID_OUT_MIN                (-260.0f) /**< 距离控制 PID: 最小输出限制 */
#define APP_DISTANCE_PID_OUT_MAX                (260.0f)  /**< 距离控制 PID: 最大输出限制 */

#define APP_ATTITUDE_PID_KP                     (10.50f)  /**< 姿态(偏航角)控制 PID: 比例系数 P */
#define APP_ATTITUDE_PID_KI                     (0.02f)   /**< 姿态控制 PID: 积分系数 I */
#define APP_ATTITUDE_PID_KD                     (0.00f)   /**< 姿态控制 PID: 微分系数 D */
#define APP_ATTITUDE_PID_OUT_MIN                (-1800.0f)/**< 姿态控制 PID: 最小输出 */
#define APP_ATTITUDE_PID_OUT_MAX                (1800.0f) /**< 姿态控制 PID: 最大输出 */

#define APP_TURN_PID_KP                         (0.12f)   /**< 转向(角速度)控制 PID: 比例系数 P */
#define APP_TURN_PID_KI                         (0.001f)  /**< 转向控制 PID: 积分系数 I */
#define APP_TURN_PID_KD                         (0.00f)   /**< 转向控制 PID: 微分系数 D */
#define APP_TURN_PID_OUT_MIN                    (-200.0f) /**< 转向控制 PID: 最小输出限制 */
#define APP_TURN_PID_OUT_MAX                    (200.0f)  /**< 转向控制 PID: 最大输出限制 */
#define APP_TURN_ASSIST_MIN_ERR_DEG10           (20)      /**< 辅助转向最小误差阈值 (0.1度) */
#define APP_TURN_ASSIST_SOFT_ERR_DEG10          (120)
#define APP_TURN_ASSIST_MIN_WHEEL_DUTY          (160)
#define APP_TURN_ASSIST_SOFT_WHEEL_DUTY         (100)
#define APP_TURN_DIRECT_DUTY_KP                 (1.05f)
#define APP_TURN_DIRECT_YAW_DAMP                (0.08f)
#define APP_TURN_DIRECT_MAX_WHEEL_DUTY          (680)

#define APP_LINE_PID_KP                         (0.25f)
#define APP_LINE_PID_KI                         (0.00f)
#define APP_LINE_PID_KD                         (0.00f)
#define APP_LINE_PID_OUT_MIN                    (-1200.0f)
#define APP_LINE_PID_OUT_MAX                    (1200.0f)

#define APP_TURN_TEST_ANGLE_90_DEG10            (900)
#define APP_TURN_DONE_HEADING_ERR_DEG10         (30)
#define APP_TURN_DONE_YAW_ABS_MDPS              (25)
#define APP_TURN_DONE_HOLD_CYCLES               (8U)

#define APP_ODOM_PID_KP                         (APP_DISTANCE_PID_KP)
#define APP_ODOM_PID_KI                         (APP_DISTANCE_PID_KI)
#define APP_ODOM_PID_KD                         (APP_DISTANCE_PID_KD)

typedef enum
{
    APP_TRACK_TARGET_BLACK = 0,
    APP_TRACK_TARGET_WHITE = 1,
} AppTrackTarget_e;

extern int16_t g_app_route_test_speed_mmps;
extern int32_t g_app_route_segment1_mm;
extern int32_t g_app_route_segment2_mm;
extern int32_t g_app_route_segment3_mm;
extern int16_t g_app_route_turn_angle_deg10;
extern int16_t g_app_route_turn_entry_lead_mm;
extern int16_t g_app_turn_exit_boost_mmps;
extern uint8_t g_app_turn_exit_boost_ticks;
extern int16_t g_app_route_turn_done_yaw_abs_mdps;
extern uint8_t g_app_route_turn_done_hold_cycles;
extern int16_t g_app_straight_side_bias_x10;
extern int16_t g_app_arc_test_speed_mmps;
extern int16_t g_app_arc_test_radius_mm;
extern int16_t g_app_arc_test_angle_deg10;

static inline int16_t AppConfig_GetRouteTestSpeedMmps(void)
{
    return g_app_route_test_speed_mmps;
}

static inline int32_t AppConfig_GetRouteSegment1Mm(void)
{
    return g_app_route_segment1_mm;
}

static inline int32_t AppConfig_GetRouteSegment2Mm(void)
{
    return g_app_route_segment2_mm;
}

static inline int32_t AppConfig_GetRouteSegment3Mm(void)
{
    return g_app_route_segment3_mm;
}

static inline int16_t AppConfig_GetRouteTurnAngleDeg10(void)
{
    return g_app_route_turn_angle_deg10;
}

static inline int16_t AppConfig_GetRouteTurnEntryLeadMm(void)
{
    return g_app_route_turn_entry_lead_mm;
}

static inline int16_t AppConfig_GetTurnExitBoostMmps(void)
{
    return g_app_turn_exit_boost_mmps;
}

static inline uint8_t AppConfig_GetTurnExitBoostTicks(void)
{
    return g_app_turn_exit_boost_ticks;
}

static inline int16_t AppConfig_GetRouteTurnDoneYawAbsMdps(void)
{
    return g_app_route_turn_done_yaw_abs_mdps;
}

static inline uint8_t AppConfig_GetRouteTurnDoneHoldCycles(void)
{
    return g_app_route_turn_done_hold_cycles;
}

static inline int16_t AppConfig_GetStraightSideBiasX10(void)
{
    return g_app_straight_side_bias_x10;
}

static inline float AppConfig_GetStraightSideBiasMmps(void)
{
    return ((float)g_app_straight_side_bias_x10) * 0.1f;
}

static inline int16_t AppConfig_GetArcTestSpeedMmps(void)
{
    return g_app_arc_test_speed_mmps;
}

static inline int16_t AppConfig_GetArcTestRadiusMm(void)
{
    return g_app_arc_test_radius_mm;
}

static inline int16_t AppConfig_GetArcTestAngleDeg10(void)
{
    return g_app_arc_test_angle_deg10;
}

#endif
