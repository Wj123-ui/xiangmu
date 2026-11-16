#ifndef _PWM_H_
#define _PWM_H_
#include "main.h"
extern float avepwm;
extern float leftpwm;
extern float rightpwm;
extern float difpwm;
void WJ_Set_rightPWM(float PWM);
void WJ_Set_leftPWM(float PWM);
#endif
