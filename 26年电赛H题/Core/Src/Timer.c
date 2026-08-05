#include "Timer.h"
#include "A4950.h"
#include "Encoder.h"
#include "eum.h"
#include "task.h"
#include "tmc2209.h"

/*
 * TIM2 软件调度器：TIM2 每 1 ms 进入一次中断，本文件把该基础节拍分频为
 * 编码器/PID、OLED、TASK2、TASK3 和步进电机速度斜坡所需的不同周期。
 * PID 与 TASK2 距离/停车控制在编码器采样完成后立即执行；OLED 刷新仍在主循环完成。
 */

/* TIM2 句柄由 CubeMX 生成。 */
extern TIM_HandleTypeDef htim2;

/* 速度 PID 在 TIM2 的 20 ms 节拍中执行，避免主循环阻塞导致控制延迟。 */
extern void PID_all_update(void);

/* 编码器采样以及直流电机速度 PID 的周期，单位 ms。 */
#define SPEED_UPDATE_TIME_MS    20U
/* OLED 刷新的周期，单位 ms。 */
#define DISPLAY_UPDATE_TIME_MS 100U
/* TASK1/TASK3/TASK4～6 的钢球位置控制周期，单位 ms。 */
#define BALL_CONTROL_UPDATE_TIME_MS 20U
/* TMC2209 角速度加减速斜坡的更新周期，单位 ms。 */
#define TMC_RAMP_UPDATE_TIME_MS  2U

static uint16_t speed_counter;       /* 编码器/PID 分频计数器。 */
static uint16_t display_counter;     /* OLED 分频计数器。 */
static uint8_t task2_counter;        /* TASK2 距离控制分频计数器。 */
static uint8_t ball_control_counter; /* TASK1/TASK3/TASK4 钢球控制分频计数器。 */
static uint8_t tmc_ramp_counter;     /* 步进电机斜坡分频计数器。 */

static int16_t left_speed_previous;  /* 左轮上一周期的原始编码器增量。 */
static int16_t right_speed_previous; /* 右轮上一周期的原始编码器增量。 */
static volatile int16_t left_speed;  /* 左轮两点均值滤波后的增量。 */
static volatile int16_t right_speed; /* 右轮两点均值滤波后的增量。 */

/* 以下 ready 标志由 TIM2 中断置 1；PID 和 TASK1～TASK4 控制在中断内完成。 */
volatile uint8_t speed_update_ready;   /* 编码器新速度可用。 */
volatile uint8_t display_update_ready; /* 到达 OLED 刷新时刻。 */

//----------------------------------------------------------------
void Timer_Init(void)
{
    /* 清空所有软件分频器、速度缓存和待处理标志。 */
    speed_counter = 0U;
    display_counter = 0U;
    task2_counter = 0U;
    ball_control_counter = 0U;
    tmc_ramp_counter = 0U;
    left_speed_previous = 0;
    right_speed_previous = 0;
    left_speed = 0;
    right_speed = 0;
    speed_update_ready = 0U;
    display_update_ready = 0U;

    /* 启动 TIM2 的 1 ms 更新中断，启动失败则交给统一错误处理。 */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }
}
//----------------------------------------------------------------
void WJ_Reset_Speed(void)
{
    /* 保存进入函数前的中断状态，避免把调用者原本关闭的中断错误打开。 */
    uint32_t interrupt_state = __get_PRIMASK();

    /* 与 TIM2 共享的速度数据必须在临界区内整体清零。 */
    __disable_irq();
    speed_counter = 0U;
    task2_counter = 0U;
    left_speed_previous = 0;
    right_speed_previous = 0;
    left_speed = 0;
    right_speed = 0;
    speed_update_ready = 0U;
    WJ_Reset_Encoder();
    /* 只有进入函数前允许中断时才恢复中断。 */
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}
//----------------------------------------------------------------
/*
 * TIM2 的 1 ms 扫描函数。
 *
 * 写法与参考工程 Timer.c 一致：
 * 1. 计数达到 20 ms 后清零；
 * 2. 刷新编码器速度；
 * 3. 在中断内完成速度 PID 或主动制动；
 * 4. 将 speed_update_ready 置 1，供主循环做调试同步。
 */
static void Timer_Scan1ms(void)
{
    int16_t left_now;
    int16_t right_now;

    /* TASK2 的比赛时间与主循环无关，启动后每 1 ms 精确累计一次。 */
    WJ_task_2_time_tick_1ms();

    /* 保留旧相对角度接口的 1 ms 计时；当前 TASK1/TASK3/TASK4 使用下面的绝对位置模式。 */
    TMC2209_AngleMoveTick1ms();
    /* TASK1/TASK3/TASK4～6 的绝对位置模式在此等待 DIR 建立时间；实际步数由 TIM9_CC1 逐脉冲累计。 */
    TMC2209_PositionMoveTick1ms();

    speed_counter++;
    if (speed_counter >= SPEED_UPDATE_TIME_MS) {
        speed_counter = 0U;

        left_now = WJ_read_Encoder_left();
        right_now = WJ_read_Encoder_right();

        /* 当前值与上一采样值各占 50%，抑制单周期编码器计数抖动。 */
        left_speed = (int16_t)((float)left_speed_previous * 0.5f +
                               (float)left_now * 0.5f);
        right_speed = (int16_t)((float)right_speed_previous * 0.5f +
                                (float)right_now * 0.5f);

        /* 保存原始值并清计数器，从零开始累计下一个 20 ms 周期。 */
        left_speed_previous = left_now;
        right_speed_previous = right_now;
        WJ_Reset_Encoder();

        /*
         * 速度闭环必须跟采样严格同周期，不能等主循环处理 ready 标志。
         * TASK2 未运行时由中断直接保持 A4950 主动制动，避免旧 PWM 残留。
         */
        if (((Mode == 2U) &&
             (WJ_task_2_get_state() == TASK2_RUNNING)) ||
            (((Mode >= 4U) && (Mode <= 6U)) &&
             (WJ_task_4_is_running() != 0U))) {
            PID_all_update();
        } else {
            A4950_Stop();
        }

        /* 保留该标志供调试/显示使用；PID 已在上面完成，不再由主循环计算。 */
        speed_update_ready = 1U;
    }
//----------------------------------------------------------------
    /* 每 100 ms 通知主循环刷新一次 OLED。 */
    display_counter++;
    if (display_counter >= DISPLAY_UPDATE_TIME_MS) {
        display_counter = 0U;
        display_update_ready = 1U;
    }
//----------------------------------------------------------------
    /*
     * TASK2 距离积分、循迹目标更新和停车判断使用固定 20 ms 周期。
     * 直接在 TIM2 中执行，OLED 的阻塞 I2C 刷新不会再拖慢停车点。
     */
    task2_counter++;
    if (task2_counter >= SPEED_UPDATE_TIME_MS) {
        task2_counter = 0U;
        if (Mode == 2U) {
            WJ_task_2();
        }
    }
//----------------------------------------------------------------
    /* 直接照参考工程，在定时器节拍中完成非阻塞按键扫描。 */
    tick();
//----------------------------------------------------------------
    /*
     * TASK1/TASK3/TASK4～6 的位置 PID 直接在 TIM2 的固定 20 ms 周期内执行。
     * 主循环仅缓存视觉数据，因此 OLED/I2C 刷新不会改变控制周期。
     */
    ball_control_counter++;
    if (ball_control_counter >= BALL_CONTROL_UPDATE_TIME_MS) {
        ball_control_counter = 0U;
        if (Mode == 1U) {
            WJ_task_1_time_tick_20ms();
        } else if (Mode == 3U) {
            WJ_task_3_time_tick_20ms();
        } else if ((Mode >= 4U) && (Mode <= 6U)) {
            WJ_task_4_time_tick_20ms();
        }
    }
//----------------------------------------------------------------
    /* 步进电机每 2 ms 逼近设定角速度，实现非阻塞加减速。 */
    tmc_ramp_counter++;
    if (tmc_ramp_counter >= TMC_RAMP_UPDATE_TIME_MS) {
        tmc_ramp_counter = 0U;
        TMC2209_SpeedRamp_Update();
    }
//----------------------------------------------------------------
}
//----------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 工程可能还有其他定时器回调，这里只响应 TIM2 更新中断。 */
    if (htim->Instance == TIM2) {
        Timer_Scan1ms();
    }
}
//----------------------------------------------------------------
/* TIM9_CH1 每完成一个 STEP PWM 高电平时进入；只转交给 TMC 的逐脉冲位置计数。 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9) {
        TMC2209_PositionPulseFinished();
    }
}
//----------------------------------------------------------------
int16_t WJ_get_left_speed(void)
{
    /*
     * 7.5：把当前 20 ms 编码器增量换算为编码器轴 RPM；
     * 0.25641：参考电机减速比换算到车轮轴。最终单位为 RPM。
     */
    return (int16_t)((float)left_speed * 7.5f * 0.25641f);
}

int16_t WJ_get_right_speed(void)
{
    /* 与左轮采用相同换算；正值表示小车前进方向，负值表示反转。 */
    return (int16_t)((float)right_speed * 7.5f * 0.25641f);
}
