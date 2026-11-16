#include "PWM.h"
float avepwm;
float leftpwm;
float rightpwm;
float difpwm;
void WJ_Set_rightPWM(float PWM)
{
	int lf=(int)PWM;
	if(lf>=1000)
	{
		lf=1000;
	}
	if(lf<=-1000)
	{
		lf=-1000;
	}
	if(lf>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000-lf);
	}
	else if(lf<0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000+lf);
	}
}
void WJ_Set_leftPWM(float PWM)
{
	int ri=(int)PWM;
	if(ri>=1000)
	{
		ri=1000;
	}
	if(ri<=-1000)
	{
		ri=-1000;
	}
	if(ri>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000-ri);
	}
	else if(ri<0)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,1000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000+ri);
	}
}
