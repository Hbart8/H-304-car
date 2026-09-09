#include "ARCH/app_system.h"

#include "AppTick.h"
#include "Serial.h"
#include "ARCH/behavior/behavior_motion.h"
#include "ARCH/config/app_config.h"
#include "ARCH/control/motion_control.h"
#include "ARCH/device/device_center.h"
#include "ARCH/device/device_gyro.h"
#include "ARCH/hal/hal_chassis.h"
#include "ARCH/interaction/interaction_service.h"
#include "ARCH/scheduler/app_scheduler.h"
#include "ARCH/task/task_service.h"

/*
 * 这些全局变量作为运行时可调参数导出，便于菜单层或调试逻辑直接修改路线、转向和圆弧测试配置，
 * 同时保留 app_config.h 中的编译期默认值作为启动初值。
 */
int16_t g_app_route_test_speed_mmps = APP_ROUTE_TEST_SPEED_MMPS;
int32_t g_app_route_segment1_mm = APP_ROUTE_SEGMENT1_MM;
int32_t g_app_route_segment2_mm = APP_ROUTE_SEGMENT2_MM;
int32_t g_app_route_segment3_mm = APP_ROUTE_SEGMENT3_MM;
int16_t g_app_route_turn_angle_deg10 = APP_ROUTE_TURN_ANGLE_DEG10;
int16_t g_app_route_turn_entry_lead_mm = APP_ROUTE_TURN_ENTRY_LEAD_MM;
int16_t g_app_turn_exit_boost_mmps = APP_TURN_EXIT_BOOST_MMPS;
uint8_t g_app_turn_exit_boost_ticks = APP_TURN_EXIT_BOOST_TICKS;
int16_t g_app_route_turn_done_yaw_abs_mdps = APP_ROUTE_TURN_DONE_YAW_ABS_MDPS;
uint8_t g_app_route_turn_done_hold_cycles = APP_ROUTE_TURN_DONE_HOLD_CYCLES;
int16_t g_app_straight_side_bias_x10 = APP_STRAIGHT_SIDE_BIAS_X10_DEFAULT;
int16_t g_app_arc_test_speed_mmps = APP_ARC_TEST_SPEED_MMPS;
int16_t g_app_arc_test_radius_mm = APP_ARC_TEST_RADIUS_MM;
int16_t g_app_arc_test_angle_deg10 = APP_ARC_TEST_ANGLE_DEG10;

/*
 * 500ms 调试心跳负责把任务层、控制层和设备层的关键状态拼成串口日志。
 * 在特定路线的转向阶段输出更细的转向调试字段，其余阶段输出简化的里程与姿态概览。
 */
static void AppSystem_DebugTick500ms(void)
{
    TaskService_Status_t task_status;
    MotionControl_Status_t motion_status;
    DeviceCenter_Snapshot_t device_snapshot;
    static uint32_t sample_index = 0U;
    static uint8_t sync_sent = 0U;

    if (AppTick_GetMs() < APP_DEBUG_STARTUP_MUTE_MS) {
        return;
    }

    if (sync_sent == 0U) {
        Serial_Printf((char *)"\r\n#DBG,START\r\n");
        sync_sent = 1U;
        return;
    }

    TaskService_GetStatus(&task_status);
    MotionControl_GetStatus(&motion_status);
    DeviceCenter_GetSnapshot(&device_snapshot);

    if (task_status.current_action == TASK_ACTION_ROUTE_2500_L90_500_R90_500 &&
        (task_status.phase_index == 3U || task_status.phase_index == 6U)) {
        Serial_Printf((char *)
            "T,%u,%u,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned int)sample_index++,
            (unsigned int)task_status.phase_index,
            (int)motion_status.heading_target_deg10,
            (int)device_snapshot.imu_heading_deg10,
            (int)motion_status.heading_error_deg10,
            (int)device_snapshot.imu_yaw_mdps,
            (int)motion_status.turn_output_diff_mmps,
            (int)motion_status.left_duty,
            (int)motion_status.right_duty);
    } else {
        Serial_Printf((char *)
            "S,%u,%u,%u,%d,%d,%d,%d\r\n",
            (unsigned int)sample_index++,
            (unsigned int)task_status.current_action,
            (unsigned int)task_status.phase_index,
            (int)device_snapshot.chassis_distance_mm,
            (int)device_snapshot.imu_heading_deg10,
            (int)motion_status.left_duty,
            (int)motion_status.right_duty);
    }
}

/*
 * 1ms 周期只处理必须高频刷新的底层服务，避免把串口发送和底盘驱动延后到较慢的业务周期。
 */
static void AppSystem_HalTick1ms(void)
{
    Serial_Tick1ms();
    HalChassis_Service1ms();
}

void AppSystem_Init(void)
{
    /*
     * 初始化顺序遵循“底层采集/控制能力 -> 任务/交互 -> 调度器”的依赖关系，
     * 保证后续被注册的周期任务在第一次触发前，其依赖模块已经准备完成。
     */
    Serial_Init();
    DeviceCenter_Init();
    BehaviorMotion_Init();
    MotionControl_Init();
    TaskService_Init();
    InteractionService_Init();
    AppScheduler_Init();

    /*
     * 调度顺序按照硬件维护、任务生成、设备采样、控制执行、交互刷新来排列。
     * start_delay_ms 用于错开同周期任务，减小同一毫秒内的执行扎堆。
     */
    AppScheduler_Register(AppSystem_HalTick1ms, APP_SCHED_PERIOD_HAL_MS, 0U);
    AppScheduler_Register(HalChassis_Tick10ms, APP_SCHED_PERIOD_CONTROL_MS, 0U);
    AppScheduler_Register(TaskService_Tick10ms, APP_SCHED_PERIOD_TASK_MS, 1U);
    AppScheduler_Register(DeviceCenter_Tick10ms, APP_SCHED_PERIOD_DEVICE_MS, 2U);
    AppScheduler_Register(MotionControl_Tick10ms, APP_SCHED_PERIOD_CONTROL_MS, 3U);
    AppScheduler_Register(InteractionService_Tick50ms, APP_SCHED_PERIOD_INTERACTION_MS, 5U);
    AppScheduler_Register(AppSystem_DebugTick500ms, APP_SCHED_PERIOD_DEBUG_MS, 20U);

    DeviceCenter_PlayStartup();
    DeviceCenter_SetRgb(1U, 0U, 0U);
}

void AppSystem_Run(void)
{
    /* 主循环本身不做业务判断，所有周期行为统一交给调度器分发。 */
    AppScheduler_Run();
}
