#include "A4950.h"
void WJ_Set_leftspeed(float a)
{
    if(a>=0)
    {
        DL_TimerG_setCaptureCompareValue(PWM_left_INST ,(int)a, DL_TIMER_CC_0_INDEX);
        DL_TimerG_setCaptureCompareValue(PWM_left_INST ,1000, DL_TIMER_CC_1_INDEX);
    }
    else 
    {
         DL_TimerG_setCaptureCompareValue(PWM_left_INST ,1000, DL_TIMER_CC_0_INDEX);
        DL_TimerG_setCaptureCompareValue(PWM_left_INST ,(int)(-a), DL_TIMER_CC_1_INDEX);
    }
}
void WJ_Set_rightspeed(float a)
{
    if(a>=0)
    {
        DL_TimerG_setCaptureCompareValue(PWM_right_INST ,(int)a, DL_TIMER_CC_0_INDEX);
        DL_TimerG_setCaptureCompareValue(PWM_right_INST,1000, DL_TIMER_CC_1_INDEX);
    }
    else 
    {
         DL_TimerG_setCaptureCompareValue(PWM_right_INST,1000, DL_TIMER_CC_0_INDEX);
        DL_TimerG_setCaptureCompareValue(PWM_right_INST,(int)(-a), DL_TIMER_CC_1_INDEX);
    }
}