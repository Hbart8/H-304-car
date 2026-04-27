#ifndef APP_ARCH_TASK_SERVICE_H
#define APP_ARCH_TASK_SERVICE_H

#include <stdint.h>

/**
 * @brief 任务服务可以执行的各种动作枚举
 * @note 包含比赛用的各种模式和测试用的动作
 */
typedef enum
{
    TASK_ACTION_NONE = 0,                             /**< 无动作 */
    TASK_ACTION_RUN_MODE_A,                           /**< 运行模式 A */
    TASK_ACTION_RUN_MODE_B,                           /**< 运行模式 B */
    TASK_ACTION_STRAIGHT_TEST,                        /**< 直线测试 */
    TASK_ACTION_ARC_TEST,                             /**< 圆弧测试 */
    TASK_ACTION_GO_1000MM,                            /**< 前进 1000 毫米 */
    TASK_ACTION_ROUTE_2500_L90_500_R90_500,           /**< 路线: 2500mm -> 左转90度 -> 500mm -> 右转90度 -> 500mm */
    TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_LEFT,    /**< 题目2左侧路线: 700mm -> 180度圆弧 -> 180度圆弧 -> 右转90度 -> 700mm */
    TASK_ACTION_Q2_700_ARC180_ARC180_R90_700_RIGHT,   /**< 题目2右侧路线: 700mm -> 180度圆弧 -> 180度圆弧 -> 右转90度 -> 700mm */
    TASK_ACTION_Q3_1200_R270_500_R420_1800,           /**< 题目3路线: 1200mm -> 右转270度 -> 500mm -> 右转420度 -> 1800mm */
    TASK_ACTION_MOTOR_TEST_1,                         /**< 电机测试 1 */
    TASK_ACTION_MOTOR_TEST_2,                         /**< 电机测试 2 */
    TASK_ACTION_TURN_LEFT_90,                         /**< 左转 90 度 */
    TASK_ACTION_TURN_RIGHT_90,                        /**< 右转 90 度 */
    TASK_ACTION_SENSOR_SCAN,                          /**< 传感器扫描测试 */
} TaskAction_e;

/**
 * @brief 任务执行的阶段状态
 */
typedef enum
{
    TASK_STAGE_IDLE = 0,      /**< 空闲状态，没有任务在运行 */
    TASK_STAGE_PREPARE,       /**< 准备阶段，任务即将开始 */
    TASK_STAGE_EXECUTE,       /**< 执行阶段，任务正在运行中 */
    TASK_STAGE_COMPLETE,      /**< 完成阶段，任务已经运行结束 */
} TaskStage_e;

/**
 * @brief 任务服务状态结构体，用于记录当前任务的信息
 */
typedef struct
{
    TaskAction_e current_action;        /**< 当前正在执行的动作 */
    TaskAction_e last_completed_action; /**< 上一个已完成的动作 */
    TaskStage_e stage;                  /**< 当前任务的执行阶段 */
    uint8_t queue_depth;                /**< 任务队列当前的深度（有多少个任务在排队） */
    uint8_t busy;                       /**< 忙碌标志：1 表示任务正在运行，0 表示空闲 */
    uint8_t progress;                   /**< 任务执行进度 (0-100) */
    uint8_t phase_index;                /**< 当前任务所处的子阶段/步骤索引 */
} TaskService_Status_t;

/**
 * @brief 任务服务初始化
 * @note 应在系统启动时调用，初始化任务队列和状态机
 */
void TaskService_Init(void);

/**
 * @brief 将新任务加入执行队列
 * @param action 要加入队列的任务动作
 * @return uint8_t 如果成功加入返回 1，队列满返回 0
 */
uint8_t TaskService_Enqueue(TaskAction_e action);

/**
 * @brief 取消当前正在执行的任务，并清空任务队列
 */
void TaskService_CancelCurrent(void);

/**
 * @brief 任务服务的心跳函数
 * @note 必须由定时器每 10ms 调用一次，用于驱动任务状态机
 */
void TaskService_Tick10ms(void);

/**
 * @brief 获取任务服务的当前状态
 * @param status 指向用于接收状态数据的结构体指针
 */
void TaskService_GetStatus(TaskService_Status_t *status);

/**
 * @brief 获取任务动作的名称字符串（用于显示或调试）
 * @param action 任务动作枚举值
 * @return const char* 返回对应的名称字符串
 */
const char *TaskService_GetActionName(TaskAction_e action);

/**
 * @brief 获取任务阶段的名称字符串（用于显示或调试）
 * @param stage 任务阶段枚举值
 * @return const char* 返回对应的名称字符串
 */
const char *TaskService_GetStageName(TaskStage_e stage);

#endif
