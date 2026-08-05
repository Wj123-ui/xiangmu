#ifndef XUNJI_H
#define XUNJI_H

#include "main.h"

/*
 * 七路循迹传感器：
 * 从左至右的 P1~P7 对应 PC0~PC6，高电平表示检测到赛道。
 */
/* 返回差速修正量，单位 RPM；负值左转、正值右转。 */
float WJ_xunji_fangxing(void);
/* TASK4～6 共用差速修正；幅度参数位于 xunji.c 顶部，和 TASK2 相互独立。 */
float WJ_xunji_task4_fangxing(void);
/* TASK4～6 每次启动时清空循迹低通和上次转向，防止沿用上一轮输出。 */
void WJ_xunji_task4_reset(void);
/* 七路全高返回 0，否则返回 1；该返回约定沿用参考工程。 */
uint8_t WJ_xunji_quanhei(void);
#endif
