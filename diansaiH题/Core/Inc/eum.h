#ifndef EUM_H
#define EUM_H

#include "main.h"
#include <stdint.h>

/*
 * 与 emptyxiaoache1 菜单一致：
 * Mode=0 为菜单，Mode=1~6 对应 TASK1~TASK6。
 */
extern volatile uint8_t Mode;

/* 初始化菜单、光标和按键扫描状态，上电时调用一次。 */
void Menu_Init(void);
/* 在主循环调用，处理按键事件；TASK6 双击取点时使用最新 K230 坐标。 */
void Menu_Update(int16_t k230_x, uint8_t ball_found);
/* 由 TIM2 的 1 ms 中断调用；KEY1/2/3/4 分别为上、下、运行、总复位。 */
void tick(void);
/*
 * 刷新当前菜单或任务页面：
 * k230_x：小球相对画面中心的横向像素偏差；
 * front_back_accel_mg / left_right_accel_mg：平面加速度，单位 mg，当前预留；
 * tmc_ok：TMC2209 初始化是否成功，1 表示成功。
 */
void eumshow(int16_t k230_x,
             int32_t front_back_accel_mg,
             int32_t left_right_accel_mg,
             uint8_t tmc_ok);

#endif
