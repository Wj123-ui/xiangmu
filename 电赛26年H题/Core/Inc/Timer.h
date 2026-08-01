#ifndef TIMER_H
#define TIMER_H

#include "main.h"
#include <stdint.h>

/* 清零调度状态并启动 TIM2；TIM2 每 1 ms 进入一次更新中断。 */
void Timer_Init(void);
/* TASK2 每次启动前清空编码器计数、速度滤波和相关周期标志。 */
void WJ_Reset_Speed(void);

/*
 * 20 ms 编码器速度刷新完成标志：中断置 1，主循环可清 0。
 * 速度 PID 已在 TIM2 中断内紧跟采样执行，不依赖该标志。
 */
extern volatile uint8_t speed_update_ready;

/* 100 ms OLED 刷新标志：中断置 1，主循环显示完成后清 0。 */
extern volatile uint8_t display_update_ready;

/*
 * 获取最近一次滤波后的车轮转速，单位 RPM（转/分钟）。
 * 返回值正负表示车辆前进/后退方向。
 */
int16_t WJ_get_left_speed(void);
int16_t WJ_get_right_speed(void);

#endif
