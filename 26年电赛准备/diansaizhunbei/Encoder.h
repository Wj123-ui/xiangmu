#ifndef _Encoder_H_
#define _Encoder_H_
#include "ti_msp_dl_config.h"
void WJ_Reset_Encoder(void);
int16_t WJ_read_Encoder_left(void);//返回左电机速度
int16_t WJ_read_Encoder_right(void);//返回右电机速度
#endif