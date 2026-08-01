/**
 * @file Encoder.c
 * @brief 左右直流电机的正交编码器计数读取。
 *
 * 左编码器使用 TIM1，右编码器使用 TIM4。定时器工作在编码器模式，
 * 本模块只负责启动、清零和读取计数，转速换算由上层定时任务完成。
 */
#include "Encoder.h"
#include "stm32f4xx_ll_tim.h"

/* 两个编码器定时器句柄均由 CubeMX 生成。 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/**
 * @brief 启动左右编码器接口并将两个计数器清零。
 * @note 任意编码器启动失败都会进入 Error_Handler()。
 */
void Encoder_Init(void)
{
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK ||
        HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    WJ_Reset_Encoder();
}

/**
 * @brief 同时清零左右编码器硬件计数器。
 * @note 转速采样通常在读取本周期增量后调用此函数开始下一周期计数。
 */
void WJ_Reset_Encoder(void)
{
    LL_TIM_SetCounter(TIM1, 0U);
    LL_TIM_SetCounter(TIM4, 0U);
}

/**
 * @brief 读取左编码器自上次清零后的有符号计数增量。
 * @return TIM1 当前计数值，按 int16_t 解释以保留正反转方向。
 */
int16_t WJ_read_Encoder_left(void)
{
    return (int16_t)LL_TIM_GetCounter(TIM1);
}

/**
 * @brief 读取右编码器自上次清零后的有符号计数增量。
 * @return 经过方向统一后的 TIM4 计数值。
 */
int16_t WJ_read_Encoder_right(void)
{
    /* 右编码器安装方向与左侧相反，在读取层取反以统一车辆前进方向。 */
    return -(int16_t)LL_TIM_GetCounter(TIM4);
}
