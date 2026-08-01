#include "board.h"

/*
 * 板级阻塞延时工具。
 * 微秒延时使用 Cortex-M4 的 DWT 周期计数器，毫秒延时使用 HAL_Delay()。
 * 这些函数仅供外设初始化和短时序使用，不应放入周期控制或中断回调。
 */

/* 记录 DWT 周期计数器是否已经启用，避免每次延时重复初始化。 */
static uint8_t dwt_ready;

static void Board_DWT_Init(void)
{
    /* 已启用时直接返回。 */
    if (dwt_ready != 0U) {
        return;
    }
    /* 打开内核跟踪模块并使能 CYCCNT 自由运行计数器。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_ready = 1U;
}

void Board_DelayUs(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    Board_DWT_Init();
    /* 把微秒数换算成 CPU 周期数，并忙等待到目标周期数。 */
    start = DWT->CYCCNT;
    ticks = (SystemCoreClock / 1000000U) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    }
}

void delay_us(int us)
{
    /* 兼容移植库的有符号参数；非正延时直接忽略。 */
    if (us > 0) {
        Board_DelayUs((uint32_t)us);
    }
}

void delay_ms(int ms)
{
    /* HAL_Delay() 为阻塞延时，依赖 SysTick 正常运行。 */
    if (ms > 0) {
        HAL_Delay((uint32_t)ms);
    }
}

/* 兼容参考工程保留的旧接口名称。 */
void delay_1us(int us) { delay_us(us); }
void delay_1ms(int ms) { delay_ms(ms); }
