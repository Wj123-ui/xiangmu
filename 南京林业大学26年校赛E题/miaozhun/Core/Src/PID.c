#include "PID.h"
#include <string.h>

void PID_Init(PID_T *pid, float kp, float ki, float kd, float outmax, float outmin)
{
    memset(pid, 0, sizeof(PID_T));
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->Outmax = outmax;
    pid->Outmin = outmin;
}


void PID_Update(PID_T *pid)
{
    pid->error0 = pid->target - pid->actual;

    pid->Out += pid->KP * (pid->error0 - pid->error1)
              + pid->KI *  pid->error0
              + pid->KD * (pid->error0 - 2.0f * pid->error1 + pid->error2);

    if (pid->Out > pid->Outmax) pid->Out = pid->Outmax;
    if (pid->Out < pid->Outmin) pid->Out = pid->Outmin;

    pid->error2 = pid->error1;
    pid->error1 = pid->error0;
}