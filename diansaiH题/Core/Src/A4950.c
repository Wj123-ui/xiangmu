/**
 * @file A4950.c
 * @brief A4950 双路直流电机驱动。
 *
 * TIM3 的 CH1/CH2 控制左电机，CH3/CH4 控制右电机。
 * 上层给出的 command 范围为 -A4950_COMMAND_MAX～A4950_COMMAND_MAX：
 * 正负号表示转动方向，绝对值表示驱动力大小。
 */
#include "A4950.h"
#include "stm32f4xx_ll_tim.h"

/* TIM3 由 CubeMX 生成并在 main.c 中完成初始化。 */
extern TIM_HandleTypeDef htim3;
//-------------------------------------------------------------------
/**
 * @brief 将电机控制量的绝对值换算成 TIM3 的比较值跨度。
 * @param magnitude 电机控制量，允许传入正数或负数。
 * @return 与控制量绝对值对应的计数值，范围为 0～TIM3 周期。
 */
static uint32_t A4950_Compare(float magnitude)
{
    /* ARR 从 0 开始计数，因此一个完整 PWM 周期的计数数为 ARR+1。 */
    uint32_t period = LL_TIM_GetAutoReload(TIM3) + 1U;

    /* 方向由 A4950_SetPair() 处理，这里只换算控制量大小。 */
    if (magnitude < 0.0f) {
        magnitude = -magnitude;
    }
    /* 防止上层控制量超过允许范围而产生越界比较值。 */
    if (magnitude > A4950_COMMAND_MAX) {
        magnitude = A4950_COMMAND_MAX;
    }
    /* 将 0～A4950_COMMAND_MAX 线性映射到 0～period。 */
    return (uint32_t)((magnitude * (float)period) / A4950_COMMAND_MAX);
}

/**
 * @brief 设置一台 A4950 电机对应的两个 PWM 输入通道。
 * @param channel_a 电机 IN1 对应的 TIM3 通道。
 * @param channel_b 电机 IN2 对应的 TIM3 通道。
 * @param command 电机控制量；正数正转，负数反转，0 主动制动。
 */
static void A4950_SetPair(uint32_t channel_a, uint32_t channel_b, float command)
{
    /* period 是一个 PWM 周期的总计数数。 */
    uint32_t period = LL_TIM_GetAutoReload(TIM3) + 1U;
    /* duty 表示控制量对应的有效驱动计数跨度。 */
    uint32_t duty = A4950_Compare(command);
    /* 本工程采用高电平制动，因此实际写入值使用 period-duty。 */
    uint32_t pwm_compare = period - duty;

    if (command > 0.0f) {
        /*
         * 与原 MSPM0 工程保持一致：
         * IN1 恒高，IN2 在“高电平制动 / 低电平驱动”之间切换。
         */
        __HAL_TIM_SET_COMPARE(&htim3, channel_a, period);
        __HAL_TIM_SET_COMPARE(&htim3, channel_b, pwm_compare);
    } else if (command < 0.0f) {
        /* 交换恒高端与 PWM 端即可反转电流方向。 */
        __HAL_TIM_SET_COMPARE(&htim3, channel_a, pwm_compare);
        __HAL_TIM_SET_COMPARE(&htim3, channel_b, period);
    } else {
        /* A4950 的 IN1=1、IN2=1 为制动，行为与原工程一致。 */
        __HAL_TIM_SET_COMPARE(&htim3, channel_a, period);
        __HAL_TIM_SET_COMPARE(&htim3, channel_b, period);
    }
}

/**
 * @brief 启动左右电机使用的 4 路 PWM，并让两台电机保持制动。
 * @note 任意一路 PWM 启动失败都会进入 Error_Handler()。
 */
void A4950_Init(void)
{
    if ((HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)) {
        Error_Handler();
    }
    A4950_Stop();
}

/**
 * @brief 立即将左右电机的 A4950 输入置为主动制动状态。
 */
void A4950_Stop(void)
{
    A4950_SetPair(TIM_CHANNEL_1, TIM_CHANNEL_2, 0.0f);
    A4950_SetPair(TIM_CHANNEL_3, TIM_CHANNEL_4, 0.0f);
}
//----------------------------------------------------------------
/**
 * @brief 设置左电机控制量。
 * @param command 范围建议为 -1000～1000，符号表示方向。
 */
void WJ_Set_leftspeed(float command)
{
    /* 左电机安装方向与右电机相反，统一在驱动层修正方向。 */
    A4950_SetPair(TIM_CHANNEL_1, TIM_CHANNEL_2, -command);
}

/**
 * @brief 设置右电机控制量。
 * @param command 范围建议为 -1000～1000，符号表示方向。
 */
void WJ_Set_rightspeed(float command)
{
    A4950_SetPair(TIM_CHANNEL_3, TIM_CHANNEL_4, command);
}
