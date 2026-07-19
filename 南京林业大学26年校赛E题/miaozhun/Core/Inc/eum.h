#ifndef _eum_H_
#define _eum_H_
#include "main.h"
void show_eum(void);
void show_uart_xy(void);         /* 屏幕底部显示 K230 实际解析到的 X/Y */
void eum_task_dispatch(void);   /* 任务调度: level 1 时分发 OpenMV 数据到任务 */
extern uint8_t key_num, eumlevel;
extern uint8_t num1, num2, num3;
#endif
