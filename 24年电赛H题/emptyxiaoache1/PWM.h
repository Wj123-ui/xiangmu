#ifndef _PWM_H_
#define _PWM_H_
extern int8_t xiaozhun;
float getencoderleft(void);
void Set_Pwm(float motor_left,float motor_right);//控制左右轮
float getencoderright(void);
void Mode1(float Yaw);
void Mode2(float Yaw);
void Mode3(float Yaw);
void Mode4(float Yaw); 

#endif