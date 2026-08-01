/**
 * @file PID.c
 * @brief 通用离散 PID 控制器实现。
 *
 * 调用者应以固定周期执行 PID_Update()。本实现包含输出限幅、积分限幅、
 * 条件积分抗饱和以及微分低通滤波，微分项基于测量值计算以减小设定值突变冲击。
 */
#include "PID.h"

/**
 * @brief 将数值限制在指定的最小值与最大值之间。
 */
static float PID_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

/**
 * @brief 确保上下限顺序正确；若传入顺序相反则交换两者。
 */
static void PID_OrderLimits(float *minimum, float *maximum)
{
    /* 交换上下限时使用的临时变量。 */
    float temporary;

    if (*minimum > *maximum)
    {
        temporary = *minimum;
        *minimum = *maximum;
        *maximum = temporary;
    }
}

/**
 * @brief 初始化一个 PID 控制器及其全部运行状态。
 * @param pid PID 控制器对象指针。
 * @param kp 比例系数。
 * @param ki 离散积分系数，已经包含采样周期影响。
 * @param kd 离散微分系数，已经包含采样周期影响。
 * @param output_min 输出下限。
 * @param output_max 输出上限。
 */
void PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max)
{
    if (pid == 0)
    {
        return;
    }

    PID_OrderLimits(&output_min, &output_max);

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->output = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;

    /* 默认将积分项限制在输出范围内，防止积分项无限累积。 */
    pid->integral_min = output_min;
    pid->integral_max = output_max;

    /* 默认采用较强的微分滤波，适合抑制编码器反馈中的高频抖动。 */
    pid->derivative_filter = 0.8f;

    /* 清空历史状态；第一次更新时再建立有效的上次测量值。 */
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->error = 0.0f;
    pid->enabled = true;
    pid->initialized = false;
}

/**
 * @brief 在线修改 PID 的比例、积分和微分系数。
 */
void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置 PID 目标值。
 */
void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    if (pid == 0)
    {
        return;
    }

    pid->setpoint = setpoint;
}

/**
 * @brief 设置输出上下限，并将当前输出同步限制到新范围内。
 */
void PID_SetOutputLimits(PID_Controller *pid, float output_min, float output_max)
{
    if (pid == 0)
    {
        return;
    }

    PID_OrderLimits(&output_min, &output_max);
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->output = PID_Clamp(pid->output, output_min, output_max);
}

/**
 * @brief 设置积分项上下限，并将当前积分项同步限制到新范围内。
 */
void PID_SetIntegralLimits(PID_Controller *pid, float integral_min, float integral_max)
{
    if (pid == 0)
    {
        return;
    }

    PID_OrderLimits(&integral_min, &integral_max);
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->integral = PID_Clamp(pid->integral, integral_min, integral_max);
}

/**
 * @brief 设置微分低通滤波系数。
 * @param filter 取值限制为 0～1；越接近 1，滤波越强、响应越平缓。
 */
void PID_SetDerivativeFilter(PID_Controller *pid, float filter)
{
    if (pid == 0)
    {
        return;
    }

    pid->derivative_filter = PID_Clamp(filter, 0.0f, 1.0f);
}

/**
 * @brief 启用或禁用 PID；禁用时会同时清空控制器运行状态。
 */
void PID_Enable(PID_Controller *pid, bool enabled)
{
    if (pid == 0)
    {
        return;
    }

    pid->enabled = enabled;
    if (!enabled)
    {
        PID_Reset(pid);
    }
}

/**
 * @brief 清空输出、积分、微分和历史测量值，保留已配置的参数与限幅。
 */
void PID_Reset(PID_Controller *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->output = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->error = 0.0f;
    pid->initialized = false;
}

/**
 * @brief 执行一次离散 PID 计算。
 * @param pid PID 控制器对象指针。
 * @param measurement 当前反馈测量值。
 * @return 经过抗饱和与输出限幅后的控制量。
 * @note 必须以固定周期调用，ki、kd 按离散控制周期使用。
 */
float PID_Update(PID_Controller *pid, float measurement)
{
    /* 比例项。 */
    float proportional;
    /* 本次计算得到的候选积分项，确认未向饱和方向累积后才写回。 */
    float integral_candidate;
    /* 未滤波的本次测量值变化量。 */
    float derivative_raw;
    /* 限幅前的候选总输出。 */
    float output_candidate;
    /* 输出是否超过上限或下限。 */
    bool saturated_high;
    bool saturated_low;

    if ((pid == 0) || (!pid->enabled))
    {
        return 0.0f;
    }

    /* 误差 = 目标值 - 当前测量值；比例项直接响应当前误差。 */
    pid->error = pid->setpoint - measurement;
    proportional = pid->kp * pid->error;

    if (!pid->initialized)
    {
        /* 首次运行没有历史测量值，仅记录当前值，不计算微分。 */
        pid->previous_measurement = measurement;
        pid->derivative = 0.0f;
        pid->initialized = true;
    }
    else
    {
        /* 对测量值做微分，避免设定值突变产生微分冲击。 */
        derivative_raw = measurement - pid->previous_measurement;
        /* 一阶低通滤波：保留历史微分并混合本次原始变化量。 */
        pid->derivative =
            pid->derivative_filter * pid->derivative +
            (1.0f - pid->derivative_filter) * derivative_raw;
    }
    pid->previous_measurement = measurement;

    /* 离散积分累加，并先限制候选积分项，避免积分自身越界。 */
    integral_candidate = pid->integral + pid->ki * pid->error;
    integral_candidate = PID_Clamp(integral_candidate,
                                   pid->integral_min,
                                   pid->integral_max);

    output_candidate =
        proportional + integral_candidate - pid->kd * pid->derivative;

    /*
     * 条件积分抗饱和：
     * 只有当误差会推动已经饱和的输出继续远离有效范围时，才暂停写入积分项；
     * 当误差方向有助于输出退出饱和状态时，仍然允许继续积分。
     */
    saturated_high = output_candidate > pid->output_max;
    saturated_low = output_candidate < pid->output_min;
    if (!((saturated_high && (pid->error > 0.0f)) ||
          (saturated_low && (pid->error < 0.0f))))
    {
        pid->integral = integral_candidate;
    }

    /* 使用最终积分项重新计算总输出，并执行输出限幅。 */
    output_candidate =
        proportional + pid->integral - pid->kd * pid->derivative;
    pid->output = PID_Clamp(output_candidate,
                            pid->output_min,
                            pid->output_max);

    return pid->output;
}
