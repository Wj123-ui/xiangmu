#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器的参数与运行状态。
 *
 * integral 保存已经乘过 Ki 的积分项；derivative 保存经过低通滤波的测量值微分。
 * 调用 PID_Update() 时应保持固定采样周期，因为 ki、kd 按离散周期使用。
 */
typedef struct
{
    /* 比例、积分、微分系数。 */
    float kp;
    float ki;
    float kd;

    /* 目标值与最近一次限幅后的输出值。 */
    float setpoint;
    float output;

    /* 总输出上下限。 */
    float output_min;
    float output_max;
    /* 积分项上下限，用于防止积分饱和。 */
    float integral_min;
    float integral_max;

    /* 微分滤波系数：0 表示不保留历史值，越接近 1 滤波越强。 */
    float derivative_filter;

    /* 当前积分项、滤波后的测量微分、上次测量值及当前误差。 */
    float integral;
    float derivative;
    float previous_measurement;
    float error;

    /* enabled 控制是否输出；initialized 标记是否已有有效历史测量值。 */
    bool enabled;
    bool initialized;
} PID_Controller;

/**
 * @brief 初始化 PID 控制器。
 *
 * @param pid          PID 控制器对象。
 * @param kp           比例系数。
 * @param ki           离散积分系数。
 * @param kd           离散微分系数。
 * @param output_min   输出下限。
 * @param output_max   输出上限。
 */
void PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

/** @brief 修改比例、积分和微分系数，不清空当前运行状态。 */
void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);

/** @brief 设置控制目标值。 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint);

/** @brief 设置总输出上下限，并限制当前输出。 */
void PID_SetOutputLimits(PID_Controller *pid, float output_min, float output_max);

/** @brief 设置积分项上下限，并限制当前积分项。 */
void PID_SetIntegralLimits(PID_Controller *pid, float integral_min, float integral_max);

/** @brief 设置 0～1 范围内的微分低通滤波系数。 */
void PID_SetDerivativeFilter(PID_Controller *pid, float filter);

/** @brief 启用或禁用控制器；禁用时自动复位运行状态。 */
void PID_Enable(PID_Controller *pid, bool enabled);

/** @brief 清空 PID 运行状态，但保留系数、目标值和限幅配置。 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief 执行一次 PID 计算。
 *
 * 应以固定周期调用。微分项根据测量值而不是误差计算，
 * 可减小目标值突然改变时产生的微分冲击。
 *
 * @param pid          PID 控制器对象。
 * @param measurement  当前反馈测量值。
 * @return             经过限幅后的 PID 输出。
 */
float PID_Update(PID_Controller *pid, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
