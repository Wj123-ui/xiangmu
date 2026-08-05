#include "xunji.h"

/*
 * 七路数字循迹模块。
 * 从车头朝前看，P1～P7 由左到右排列，分别接 PC0～PC6；高电平表示检测到赛道。
 * 本模块只计算左右轮的差速修正量，基础速度及速度闭环由 TASK2/main.c 完成。
 */

/* 用宏直接读取七个 GPIO，便于下方按偏离程度依次判断。 */
#define P1 HAL_GPIO_ReadPin(XUNJI_1_GPIO_Port, XUNJI_1_Pin)
#define P2 HAL_GPIO_ReadPin(XUNJI_2_GPIO_Port, XUNJI_2_Pin)
#define P3 HAL_GPIO_ReadPin(XUNJI_3_GPIO_Port, XUNJI_3_Pin)
#define P4 HAL_GPIO_ReadPin(XUNJI_4_GPIO_Port, XUNJI_4_Pin)
#define P5 HAL_GPIO_ReadPin(XUNJI_5_GPIO_Port, XUNJI_5_Pin)
#define P6 HAL_GPIO_ReadPin(XUNJI_6_GPIO_Port, XUNJI_6_Pin)
#define P7 HAL_GPIO_ReadPin(XUNJI_7_GPIO_Port, XUNJI_7_Pin)

/* 最近一次有效转向修正量，单位 RPM；七路瞬时丢线时继续沿原方向寻找赛道。 */
static float last_correction;
static float last_correction_jao;
/*
 * P4 是中心；P1~P3 在左侧；P5~P7 在右侧。
 * 返回负数向左修正，返回正数向右修正。
 * 修正量绝对值越大，表示检测位置离中心越远，左右轮差速也越大。
 */
float WJ_xunji_fangxing(void)
{
    if (P4 != GPIO_PIN_RESET) {
        /* 中心探头命中时直行；保留 last_correction 供下一次瞬时丢线使用。 */
        return 0.0f;
    } else if (P3 != GPIO_PIN_RESET) {
        /* 靠近中心的左侧探头：小幅向左修正，单位 RPM。 */
        last_correction = -20.0f;
    } else if (P5 != GPIO_PIN_RESET) {
        /* 靠近中心的右侧探头：小幅向右修正，单位 RPM。 */
        last_correction = 20.0f;
    } else if (P2 != GPIO_PIN_RESET) {
        /* 中等左偏：增大向左修正量。 */
        last_correction = -35.0f;
    } else if (P6 != GPIO_PIN_RESET) {
        /* 中等右偏：增大向右修正量。 */
        last_correction = 35.0f;
    } else if (P1 != GPIO_PIN_RESET) {
        /* 最外侧左探头命中：采用最大向左修正量。 */
        last_correction = -55.0f;
    } else if (P7 != GPIO_PIN_RESET) {
        /* 最外侧右探头命中：采用最大向右修正量。 */
        last_correction = 55.0f;
    } else {
        /* 弯道瞬间丢线时保持上一次转向，不再把修正量突然清零直行。 */
        return last_correction;
    }

    return last_correction;
}

/*
 * TASK4～6 专用循迹参数：当前基准速度为 70～75 RPM，修正量不能再沿用高速 TASK2，
 * 否则一侧车轮会落入低速死区并产生左右摆动。只需要修改下面三个数即可调节：
 *   P3/P5：靠近中心的小幅修正；
 *   P2/P6：中等修正；
 *   P1/P7：外侧的大幅修正。
 * 正值向右修正，负值向左修正；单位都是 RPM。
 */
#define TASK4_XUNJI_NEAR_CORRECTION_RPM     6.0f
#define TASK4_XUNJI_MIDDLE_CORRECTION_RPM  12.0f
#define TASK4_XUNJI_OUTER_CORRECTION_RPM   20.0f
/* 每 20 ms 保留 88% 上次输出，传感器跨区时不会瞬间反向。 */
#define TASK4_XUNJI_FILTER_KEEP              0.88f

/* TASK4～6 单独保存原始方向和滤波输出，避免影响 TASK2。 */
static float task4_last_raw_correction;
static float task4_filtered_correction;

uint8_t WJ_xunji_task4_right_raw_active(void)
{
    return ((P5 != GPIO_PIN_RESET) ||
            (P6 != GPIO_PIN_RESET) ||
            (P7 != GPIO_PIN_RESET)) ? 1U : 0U;
}

uint8_t WJ_xunji_task4_any_raw_active(void)
{
    return ((P1 != GPIO_PIN_RESET) ||
            (P2 != GPIO_PIN_RESET) ||
            (P3 != GPIO_PIN_RESET) ||
            (P4 != GPIO_PIN_RESET) ||
            (P5 != GPIO_PIN_RESET) ||
            (P6 != GPIO_PIN_RESET) ||
            (P7 != GPIO_PIN_RESET)) ? 1U : 0U;
}

void WJ_xunji_task4_reset(void)
{
    task4_last_raw_correction = 0.0f;
    task4_filtered_correction = 0.0f;
    last_correction_jao = 0.0f;
}

/*
 * TASK4 专用循迹函数，判断顺序与 WJ_xunji_fangxing() 完全一致：
 * P4 居中直行，P3/P2/P1 向左修正，P5/P6/P7 向右修正。
 */
float WJ_xunji_task4_fangxing(void)
{
    float raw_correction;

    if (P4 != GPIO_PIN_RESET) {
        raw_correction = 0.0f;
    } else if (P3 != GPIO_PIN_RESET) {
        raw_correction = -TASK4_XUNJI_NEAR_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else if (P5 != GPIO_PIN_RESET) {
        raw_correction = TASK4_XUNJI_NEAR_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else if (P2 != GPIO_PIN_RESET) {
        raw_correction = -TASK4_XUNJI_MIDDLE_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else if (P6 != GPIO_PIN_RESET) {
        raw_correction = TASK4_XUNJI_MIDDLE_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else if (P1 != GPIO_PIN_RESET) {
        raw_correction = -TASK4_XUNJI_OUTER_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else if (P7 != GPIO_PIN_RESET) {
        raw_correction = TASK4_XUNJI_OUTER_CORRECTION_RPM;
        task4_last_raw_correction = raw_correction;
    } else {
        /* 瞬时丢线时继续沿最近方向搜索，但仍通过低通平滑输出。 */
        raw_correction = task4_last_raw_correction;
    }

    task4_filtered_correction =
        TASK4_XUNJI_FILTER_KEEP * task4_filtered_correction +
        (1.0f - TASK4_XUNJI_FILTER_KEEP) * raw_correction;
    if ((task4_filtered_correction > -0.1f) &&
        (task4_filtered_correction < 0.1f)) {
        task4_filtered_correction = 0.0f;
    }
    return task4_filtered_correction;
}

/*
 * 保留参考工程原有的返回约定：
 * 七路全部为高电平时返回 0，只要有一路不是高电平就返回 1。
 * 注意该语义与函数名不完全直观，调用处应按这里的返回值判断。
 */
uint8_t WJ_xunji_quanhei(void)
{
    if (P1 != GPIO_PIN_RESET && P2 != GPIO_PIN_RESET &&
        P3 != GPIO_PIN_RESET && P4 != GPIO_PIN_RESET &&
        P5 != GPIO_PIN_RESET && P6 != GPIO_PIN_RESET &&
        P7 != GPIO_PIN_RESET) {
        return 0;
    } else {
        return 1;
    }
}
float WJ_xunji_task5_buchangjiao(void)
{
	if (P4 != GPIO_PIN_RESET) {
        /* 中心探头命中时本周期不补偿；历史值仅供随后瞬时丢线时使用。 */
        return 0.0f;
    } else if (P3 != GPIO_PIN_RESET) {
        /* 顺时针赛道转弯统一施加负角，偏离越大补偿越强。 */
        last_correction_jao = -0.5f;
    } else if (P5 != GPIO_PIN_RESET) {
        last_correction_jao = -0.5f;
    } else if (P2 != GPIO_PIN_RESET) {
        last_correction_jao = -0.9f;
    } else if (P6 != GPIO_PIN_RESET) {
        last_correction_jao = -0.9f;
    } else if (P1 != GPIO_PIN_RESET) {
        last_correction_jao = -1.3f;
    } else if (P7 != GPIO_PIN_RESET) {
        last_correction_jao = -1.3f;
    } else {
        /* 弯道瞬间丢线时保持上一补偿角；P4 命中周期直接返回 0°。 */
        return last_correction_jao;
    }

    return last_correction_jao;
}
