#include "Timer.h"
uint16_t counter=0;
int16_t left_speed_0=0,right_speed_0 =0;
int16_t left_speed=0,right_speed=0;
volatile uint8_t speed_update_ready=0;//PID更新计数时间
void TIMER_12_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_12_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            counter++;
            if(xunji_yanshi_flag==1)
            {xunji_yanshi++;}
            if(xunji_yanshi>=200)
            {xunji_yanshi=200;}
            if(counter==20)
            {
                counter=0;
                left_speed=(int16_t)((float)(left_speed_0)*0.5+(float)(WJ_read_Encoder_left())*0.5);
                right_speed=(int16_t)((float)(right_speed_0)*0.5+(float)(WJ_read_Encoder_right())*0.5);
                left_speed_0=WJ_read_Encoder_left();
                right_speed_0=WJ_read_Encoder_right();
                WJ_Reset_Encoder();
                speed_update_ready=1U;
            }
            break;
        default:
            break;
    }
}
int16_t WJ_get_left_speed(void)
{
    return (int16_t)(left_speed*15*0.3771);
}
int16_t WJ_get_right_speed(void)
{
    return (int16_t)(right_speed*15*0.3771);
}
