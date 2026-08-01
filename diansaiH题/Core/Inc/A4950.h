#ifndef A4950_H
#define A4950_H

#include "main.h"

/* 电机控制量的最大绝对值，对应 100% PWM 驱动强度。 */
#define A4950_COMMAND_MAX 1000.0f

/** @brief 初始化 TIM3 四路 PWM，并使左右电机进入制动状态。 */
void A4950_Init(void);

/** @brief 立即主动制动左右两台直流电机。 */
void A4950_Stop(void);

/**
 * @brief 设置左电机驱动控制量。
 * @param command 正负号表示方向，绝对值表示驱动力大小。
 */
void WJ_Set_leftspeed(float command);

/**
 * @brief 设置右电机驱动控制量。
 * @param command 正负号表示方向，绝对值表示驱动力大小。
 */
void WJ_Set_rightspeed(float command);

#endif
