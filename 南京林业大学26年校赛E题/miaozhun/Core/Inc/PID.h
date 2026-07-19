#ifndef _PID_H_
#define _PID_H_
typedef struct
{
    float KP;
    float KI;
    float KD;

    float actual;
    float target;
    float Out;

    float error0;
    float error1;
    float error2;

    float Outmax;
    float Outmin;
} PID_T;

void PID_Init(PID_T *pid, float kp, float ki, float kd, float outmax, float outmin);
void PID_Update(PID_T *pid);

#endif
