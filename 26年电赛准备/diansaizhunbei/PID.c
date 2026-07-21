#include "PID.h"

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

static void PID_OrderLimits(float *minimum, float *maximum)
{
    float temporary;

    if (*minimum > *maximum)
    {
        temporary = *minimum;
        *minimum = *maximum;
        *maximum = temporary;
    }
}

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

    /* The I-term is limited to the output range by default. */
    pid->integral_min = output_min;
    pid->integral_max = output_max;

    /* A small amount of filtering is a useful default for encoder feedback. */
    pid->derivative_filter = 0.8f;

    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->previous_measurement = 0.0f;
    pid->error = 0.0f;
    pid->enabled = true;
    pid->initialized = false;
}

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

void PID_SetSetpoint(PID_Controller *pid, float setpoint)
{
    if (pid == 0)
    {
        return;
    }

    pid->setpoint = setpoint;
}

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

void PID_SetDerivativeFilter(PID_Controller *pid, float filter)
{
    if (pid == 0)
    {
        return;
    }

    pid->derivative_filter = PID_Clamp(filter, 0.0f, 1.0f);
}

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

float PID_Update(PID_Controller *pid, float measurement)
{
    float proportional;
    float integral_candidate;
    float derivative_raw;
    float output_candidate;
    bool saturated_high;
    bool saturated_low;

    if ((pid == 0) || (!pid->enabled))
    {
        return 0.0f;
    }

    pid->error = pid->setpoint - measurement;
    proportional = pid->kp * pid->error;

    if (!pid->initialized)
    {
        pid->previous_measurement = measurement;
        pid->derivative = 0.0f;
        pid->initialized = true;
    }
    else
    {
        derivative_raw = measurement - pid->previous_measurement;
        pid->derivative =
            pid->derivative_filter * pid->derivative +
            (1.0f - pid->derivative_filter) * derivative_raw;
    }
    pid->previous_measurement = measurement;

    integral_candidate = pid->integral + pid->ki * pid->error;
    integral_candidate = PID_Clamp(integral_candidate,
                                   pid->integral_min,
                                   pid->integral_max);

    output_candidate =
        proportional + integral_candidate - pid->kd * pid->derivative;

    /*
     * Conditional integration anti-windup:
     * stop integrating only when the error would push an already saturated
     * output farther into saturation. Integration in the recovery direction
     * is still allowed.
     */
    saturated_high = output_candidate > pid->output_max;
    saturated_low = output_candidate < pid->output_min;
    if (!((saturated_high && (pid->error > 0.0f)) ||
          (saturated_low && (pid->error < 0.0f))))
    {
        pid->integral = integral_candidate;
    }

    output_candidate =
        proportional + pid->integral - pid->kd * pid->derivative;
    pid->output = PID_Clamp(output_candidate,
                            pid->output_min,
                            pid->output_max);

    return pid->output;
}
