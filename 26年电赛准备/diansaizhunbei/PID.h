#ifndef PID_H
#define PID_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID controller object.
 *
 * integral stores the I-term itself (Ki * integral(error dt)).
 * derivative stores the filtered derivative of the measurement.
 */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float setpoint;
    float output;

    float output_min;
    float output_max;
    float integral_min;
    float integral_max;

    /* 0.0f: no filtering; closer to 1.0f: stronger filtering. */
    float derivative_filter;

    float integral;
    float derivative;
    float previous_measurement;
    float error;

    bool enabled;
    bool initialized;
} PID_Controller;

/**
 * @brief Initialize a PID controller.
 *
 * @param pid          Controller object.
 * @param kp           Proportional gain.
 * @param ki           Integral gain.
 * @param kd           Derivative gain.
 * @param output_min   Minimum controller output.
 * @param output_max   Maximum controller output.
 */
void PID_Init(PID_Controller *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max);

void PID_SetTunings(PID_Controller *pid, float kp, float ki, float kd);
void PID_SetSetpoint(PID_Controller *pid, float setpoint);
void PID_SetOutputLimits(PID_Controller *pid, float output_min, float output_max);
void PID_SetIntegralLimits(PID_Controller *pid, float integral_min, float integral_max);
void PID_SetDerivativeFilter(PID_Controller *pid, float filter);
void PID_Enable(PID_Controller *pid, bool enabled);
void PID_Reset(PID_Controller *pid);

/**
 * @brief Run one PID calculation.
 *
 * Call this function at a fixed period. Ki and Kd are discrete-time gains, so
 * the interrupt period must remain constant. The derivative is calculated from
 * the measurement instead of the error to avoid derivative kick when the
 * setpoint changes.
 *
 * @param measurement  Current feedback value.
 * @return             Limited PID output.
 */
float PID_Update(PID_Controller *pid, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
