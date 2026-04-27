#include "ARCH/control/pid.h"

/**
 * @brief 内部辅助函数：对浮点数进行限幅
 * @param value     要限制的值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return float    限幅后的结果
 */
static float Pid_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void Pid_Init(PidController_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    if (pid == 0) {
        return;
    }

    /* 初始化 PID 控制器参数 */
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->integral   = 0.0f;     /* 初始积分为 0 */
    pid->prev_error = 0.0f;     /* 初始历史误差为 0 */
    pid->out_min    = out_min;  /* 设置最小输出限幅 */
    pid->out_max    = out_max;  /* 设置最大输出限幅 */
}

void Pid_Reset(PidController_t *pid)
{
    if (pid == 0) {
        return;
    }

    /* 清除积分累计和上次的误差，通常在系统停止或切换目标时调用 */
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float Pid_Update(PidController_t *pid, float target, float measure, float dt_s)
{
    float error;
    float derivative;
    float output;

    /* 指针为空或时间间隔不合法时，直接返回 0 以免导致除 0 异常 */
    if ((pid == 0) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    /* 计算当前误差 */
    error = target - measure;
    
    /* 积分累加：误差乘以时间，此实现未加积分抗饱和限幅，若遇到大幅突变可能产生积分风暴 */
    pid->integral += error * dt_s;
    
    /* 微分计算：当前误差减去上次误差再除以时间（误差变化率） */
    derivative = (error - pid->prev_error) / dt_s;
    
    /* 计算 PID 总输出：P + I + D */
    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    
    /* 对输出进行上下限幅 */
    output = Pid_Clamp(output, pid->out_min, pid->out_max);
    
    /* 记录本次误差，供下一次微分计算使用 */
    pid->prev_error = error;
    
    return output;
}
