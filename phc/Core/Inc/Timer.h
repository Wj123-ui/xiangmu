#ifndef _Timer_H_
#define _Timer_H_
#include "main.h"

/* 机械平衡点(度): 角度环 Actual = Pitch - BALANCE_ANGLE。
 * 显示和控制必须用同一个值, 否则屏幕上看到的 0 不是控制器认为的 0。
 * 校准: 车稳住时读屏幕上的角度偏差, 把它补偿到这里, 使平衡时读数为 0。 */
#define BALANCE_ANGLE   16.4f

extern uint16_t count2;
extern float leftspeed;
extern float rightspeed;
#endif
