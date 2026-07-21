#include "Encoder.h"
#include "ti_msp_dl_config.h"
int16_t gEncoderCount_left=0,gEncoderCount_right=0;
extern volatile bool gIcmDataReady;

void GROUP1_IRQHandler(void)
{
    uint32_t gpioStatus = DL_GPIO_getEnabledInterruptStatus(
        GPIO_Encoder_PORT,
        ICM_INT_ICM_DRDY_PIN |
        GPIO_Encoder_PIN_left_A_PIN |
        GPIO_Encoder_PIN_left_B_PIN |
        GPIO_Encoder_PIN_right_A_PIN |
        GPIO_Encoder_PIN_right_B_PIN);

    if ((gpioStatus & ICM_INT_ICM_DRDY_PIN) != 0U)
    {
        gIcmDataReady = true;
        DL_GPIO_clearInterruptStatus(ICM_INT_PORT, ICM_INT_ICM_DRDY_PIN);
    }

    if((gpioStatus & GPIO_Encoder_PIN_left_A_PIN)==GPIO_Encoder_PIN_left_A_PIN)
    { 
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_left_B_PIN))
            {
                gEncoderCount_left++;
            }
            else 
            {
                gEncoderCount_left--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_left_A_PIN);
    }
    if((gpioStatus & GPIO_Encoder_PIN_left_B_PIN)==GPIO_Encoder_PIN_left_B_PIN)
    {
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_left_A_PIN))
            {
                gEncoderCount_left--;
            }
            else 
            {
                gEncoderCount_left++;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_left_B_PIN);    
}
    
    if((gpioStatus & GPIO_Encoder_PIN_right_A_PIN)==GPIO_Encoder_PIN_right_A_PIN)
    { 
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_right_B_PIN))
            {
                gEncoderCount_right--;
            }
            else 
            {
                gEncoderCount_right++;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_right_A_PIN);
    }
    if((gpioStatus & GPIO_Encoder_PIN_right_B_PIN)==GPIO_Encoder_PIN_right_B_PIN)
    {
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_right_A_PIN))
            {
                gEncoderCount_right++;
            }
            else 
            {
                gEncoderCount_right--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_right_B_PIN);    
}
}
void WJ_Reset_Encoder(void)//左右电机计数值置0
{
    gEncoderCount_left=0;
    gEncoderCount_right=0;
}
int16_t WJ_read_Encoder_left(void)//返回左电机速度
{
    return gEncoderCount_left;
}
int16_t WJ_read_Encoder_right(void)//返回右电机速度
{
    return gEncoderCount_right;
}