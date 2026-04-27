#ifndef APP_ARCH_CONTROL_PID_H
#define APP_ARCH_CONTROL_PID_H

/**
 * @brief PID 控制器结构体
 * @note 包含 PID 参数、运行状态及输出限幅
 */
typedef struct
{
    float kp;           /**< 比例系数 (Proportional gain) */
    float ki;           /**< 积分系数 (Integral gain) */
    float kd;           /**< 微分系数 (Derivative gain) */
    float integral;     /**< 误差积分累计值 (Integral state) */
    float prev_error;   /**< 上一次的误差 (Previous error) */
    float out_min;      /**< 输出最小值限制 (Minimum output limit) */
    float out_max;      /**< 输出最大值限制 (Maximum output limit) */
} PidController_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid     指向 PID 控制器结构体的指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param out_min 最小输出限幅
 * @param out_max 最大输出限幅
 */
void Pid_Init(PidController_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 重置 PID 控制器的内部状态（清空积分和历史误差）
 * @param pid 指向 PID 控制器结构体的指针
 */
void Pid_Reset(PidController_t *pid);

/**
 * @brief 更新 PID 控制器计算并返回控制输出
 * @param pid     指向 PID 控制器结构体的指针
 * @param target  目标值 (Target / Setpoint)
 * @param measure 当前测量值 (Measured / Process variable)
 * @param dt_s    控制周期时间差，单位：秒 (Delta time in seconds)
 * @return float  PID 计算后的控制输出值 (Control output)
 */
float Pid_Update(PidController_t *pid, float target, float measure, float dt_s);

#endif
