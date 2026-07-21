#ifndef _xunji_H_
#define _xunji_H_
#include "ti_msp_dl_config.h"
float WJ_xunji_fangxing(void);
extern uint8_t xunji_flag;
void xunji_all(void);
extern volatile uint16_t xunji_yanshi;//转弯延时计数
extern volatile uint8_t xunji_yanshi_flag;//转弯开始标志位,1为执行计数
#endif
