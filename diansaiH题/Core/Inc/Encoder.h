#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>

/** @brief 启动 TIM1/TIM4 编码器接口并清零计数器。 */
void Encoder_Init(void);

/** @brief 同时清零左右编码器计数器。 */
void WJ_Reset_Encoder(void);

/**
 * @brief 读取左编码器本采样周期内的计数增量。
 * @return 有符号脉冲计数，符号表示旋转方向。
 */
int16_t WJ_read_Encoder_left(void);

/**
 * @brief 读取右编码器本采样周期内的计数增量。
 * @return 已统一前进方向符号的有符号脉冲计数。
 */
int16_t WJ_read_Encoder_right(void);

#endif
