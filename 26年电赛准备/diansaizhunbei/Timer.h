#ifndef _Timer_H_
#define _Timer_H_
#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include "xunji.h"
int16_t WJ_get_left_speed(void);
int16_t WJ_get_right_speed(void);
extern volatile uint8_t speed_update_ready;//PID更新计数时间

#endif
