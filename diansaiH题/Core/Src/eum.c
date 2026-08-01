#include "eum.h"
#include "oled.h"
#include "task.h"
#include "Timer.h"

/*
 * OLED 菜单与按键事件模块：
 * - TIM2 中断每 1 ms 调用 tick()，每 20 ms 采样一次按键；
 * - 按键松开后只产生一个事件，由主循环中的 Menu_Update() 处理；
 * - 菜单页面只负责选择任务，真正的任务代码仍由 main.c 按 Mode 调用。
 */

/* 菜单共有 TASK1～TASK6 六项。 */
#define EUM_ITEM_COUNT       6U
/* 按键采样周期，单位 ms；用于简单去抖且不阻塞主循环。 */
#define EUM_KEY_SCAN_TIME   20U
/* TASK6 的 KEY2 双击窗口，按键事件在松手时产生。 */
#define EUM_KEY2_DOUBLE_CLICK_TIME_MS 500U

/*
 * 与参考工程保持同样的思路：
 * eum1 选择菜单项，Mode 决定主函数执行哪个任务。
 */
volatile uint8_t Mode;

static uint8_t eum1;                 /* 当前菜单光标，范围 0～5。 */
static uint8_t clear;                /* 非 0 时，下次显示前先清空 OLED。 */
static volatile uint8_t key_num;     /* 中断写、主循环读的待处理按键事件。 */
static uint8_t tick_count;           /* 1 ms 节拍累计值。 */
static uint8_t current_key;          /* 本次扫描到的按键编号，0 表示未按。 */
static uint8_t previous_key;         /* 上一次扫描到的按键编号。 */
static uint8_t task6_configuring;     /* 1=TASK6 二级菜单正在现场记录目标。 */
static volatile uint8_t task6_key2_waiting_second;
static volatile uint16_t task6_key2_double_click_remaining_ms;
static uint8_t task6_target_captured;
static uint8_t task6_capture_failed;

/*
 * 四个按键均为低电平有效。
 * 如果多个按键同时按下，按 KEY1、KEY2、KEY3、KEY4 的顺序返回第一个。
 */
static uint8_t Key_GetStatus(void)
{
    if (HAL_GPIO_ReadPin(key_1_GPIO_Port, key_1_Pin) == GPIO_PIN_RESET) {
        return 1U;
    }
    if (HAL_GPIO_ReadPin(key_2_GPIO_Port, key_2_Pin) == GPIO_PIN_RESET) {
        return 2U;
    }
    if (HAL_GPIO_ReadPin(key_3_GPIO_Port, key_3_Pin) == GPIO_PIN_RESET) {
        return 3U;
    }
    if (HAL_GPIO_ReadPin(key_4_GPIO_Port, key_4_Pin) == GPIO_PIN_RESET) {
        return 4U;
    }
    return 0U;
}

/*
 * TIM2 每 1 ms 调用一次；每 20 ms 读取按键，松手时产生一次事件。
 * 本函数在中断中只采样 GPIO 和写事件，不执行任务、不刷新 OLED，也不延时。
 */
void tick(void)
{
    tick_count++;
    if (tick_count < EUM_KEY_SCAN_TIME) {
        return;
    }
    tick_count = 0U;

    if (task6_key2_double_click_remaining_ms > EUM_KEY_SCAN_TIME) {
        task6_key2_double_click_remaining_ms -= EUM_KEY_SCAN_TIME;
    } else if (task6_key2_double_click_remaining_ms != 0U) {
        task6_key2_double_click_remaining_ms = 0U;
        task6_key2_waiting_second = 0U;
    }

    previous_key = current_key;
    current_key = Key_GetStatus();
    if ((current_key == 0U) &&
        (previous_key != 0U) &&
        (key_num == 0U)) {
        key_num = previous_key;
    }
}

static void Menu_RunSelected(void)
{
    /* 切换任务前停止旧任务，防止直流电机或步进电机保留上次输出。 */
    WJ_task_all_reset();

    /* eum1 从 0 开始，Mode 从 1 开始，与参考工程一致。 */
    Mode = eum1 + 1U;
    /* TASK1～TASK5 直接启动；TASK6 第一次 KEY3 只进入目标设置页。 */
    if (Mode == 1U) {
        WJ_task_1_start();
    } else if (Mode == 2U) {
        WJ_task_2_start();
    } else if (Mode == 3U) {
        WJ_task_3_start();
    } else if (Mode == 4U) {
        WJ_task_4_start();
    } else if (Mode == 5U) {
        WJ_task_5_start();
    } else if (Mode == 6U) {
        WJ_task_6_prepare();
        task6_configuring = 1U;
        task6_key2_waiting_second = 0U;
        task6_key2_double_click_remaining_ms = 0U;
        task6_target_captured = 0U;
        task6_capture_failed = 0U;
    }
    clear = 1U;
}

void Menu_Init(void)
{
    /* 上电默认停留在 TASK1 菜单项，不自动运行任何任务。 */
    Mode = 0U;
    eum1 = 0U;
    clear = 1U;
    key_num = 0U;
    tick_count = 0U;
    current_key = 0U;
    previous_key = 0U;
    task6_configuring = 0U;
    task6_key2_waiting_second = 0U;
    task6_key2_double_click_remaining_ms = 0U;
    task6_target_captured = 0U;
    task6_capture_failed = 0U;
}

/*
 * 主循环中的菜单事件处理函数：
 * 主菜单 KEY1/KEY2 选择、KEY3 进入；TASK6 设置页双击 KEY2 取点、KEY3 启动。
 * tick() 只记录按键；任务启动和复位均在此处执行，不占用定时器中断。
 */
void Menu_Update(int16_t k230_x, uint8_t ball_found)
{
    uint8_t key_event = key_num;

    if (key_event == 0U) {
        return;
    }
    key_num = 0U;

    if (key_event == 4U) {
        /* KEY4：任何页面都立即停止并重新初始化全部任务，回到主菜单。 */
        WJ_task_all_reset();
        Mode = 0U;
        eum1 = 0U;
        task6_configuring = 0U;
        task6_key2_waiting_second = 0U;
        task6_key2_double_click_remaining_ms = 0U;
        task6_target_captured = 0U;
        task6_capture_failed = 0U;
        clear = 1U;
    } else if ((Mode == 6U) && (task6_configuring != 0U)) {
        if (key_event == 2U) {
            if ((task6_key2_waiting_second != 0U) &&
                (task6_key2_double_click_remaining_ms != 0U)) {
                /* 第二次 KEY2 松手：记录当前识别到的小球像素。 */
                task6_key2_waiting_second = 0U;
                task6_key2_double_click_remaining_ms = 0U;
                if (WJ_task_6_capture_target(k230_x, ball_found) != 0U) {
                    task6_target_captured = 1U;
                    task6_capture_failed = 0U;
                } else {
                    task6_target_captured = 0U;
                    task6_capture_failed = 1U;
                }
            } else {
                /* 第一次 KEY2 松手：打开 500 ms 双击等待窗口。 */
                task6_key2_waiting_second = 1U;
                task6_key2_double_click_remaining_ms =
                    EUM_KEY2_DOUBLE_CLICK_TIME_MS;
                task6_capture_failed = 0U;
            }
            clear = 1U;
        } else if (key_event == 3U) {
            /* 只有双击 KEY2 成功记录目标后，KEY3 才正式启动 TASK6。 */
            if ((task6_target_captured != 0U) &&
                (task6_key2_waiting_second == 0U)) {
                task6_configuring = 0U;
                task6_key2_double_click_remaining_ms = 0U;
                WJ_task_6_start();
            }
            clear = 1U;
        }
    } else if ((key_event == 1U) && (Mode == 0U)) {
        /* key1：向上选择。 */
        if (eum1 == 0U) {
            eum1 = EUM_ITEM_COUNT - 1U;
        } else {
            eum1--;
        }
        clear = 1U;
    } else if ((key_event == 2U) && (Mode == 0U)) {
        /* key2：向下选择。 */
        eum1++;
        if (eum1 >= EUM_ITEM_COUNT) {
            eum1 = 0U;
        }
        clear = 1U;
    } else if ((key_event == 3U) && (Mode == 0U)) {
        /* key3：直接执行当前选中的 TASK/显示页。 */
        Menu_RunSelected();
    }
}

static void eumshow_main(void)
{
    /* 主菜单：星号标出 eum1 当前选择的任务。 */
    OLED_ShowString(0, 0,
                    (u8 *)((eum1 == 0U) ? "*TASK1" : " TASK1"), 8, 1);
    OLED_ShowString(0, 12,
                    (u8 *)((eum1 == 1U) ? "*TASK2" : " TASK2"), 8, 1);
    OLED_ShowString(0, 24,
                    (u8 *)((eum1 == 2U) ? "*TASK3" : " TASK3"), 8, 1);
    OLED_ShowString(0, 36,
                    (u8 *)((eum1 == 3U) ? "*TASK4" : " TASK4"), 8, 1);
    OLED_ShowString(66, 0,
                    (u8 *)((eum1 == 4U) ? "*TASK5" : " TASK5"), 8, 1);
    OLED_ShowString(66, 12,
                    (u8 *)((eum1 == 5U) ? "*TASK6" : " TASK6"), 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K1 UP K2 DN K3 RUN", 8, 1);
}

static void eumshow_task2(void)
{
    uint32_t elapsed_ms = WJ_task_2_get_time_ms();
    uint32_t elapsed_seconds = elapsed_ms / 1000U;
    uint32_t elapsed_minutes = elapsed_seconds / 60U;
    uint32_t elapsed_second_part = elapsed_seconds % 60U;
    uint32_t elapsed_tenth = (elapsed_ms / 100U) % 10U;

    /* TASK2 调试页：显示状态、实时计时、累计距离和左右轮实测 RPM。 */
    OLED_ShowString(0, 0, (u8 *)"TASK2", 8, 1);
    /*
     * 时间格式 TIME:MM:SS.t。
     * 本 OLED 的 8 号字体每个字符占 6 像素，以下坐标均按 6 像素步进，
     * 否则冒号、数字会重叠。
     */
    OLED_ShowString(36, 0, (u8 *)"TIME:", 8, 1);
    OLED_ShowNum(66, 0, elapsed_minutes % 100U, 2, 8, 1);
    OLED_ShowString(78, 0, (u8 *)":", 8, 1);
    OLED_ShowNum(84, 0, elapsed_second_part, 2, 8, 1);
    OLED_ShowString(96, 0, (u8 *)".", 8, 1);
    OLED_ShowNum(102, 0, elapsed_tenth, 1, 8, 1);
    OLED_ShowString(108, 0, (u8 *)"s", 8, 1);
    OLED_ShowString(0, 12, (u8 *)"STATE:", 8, 1);
    OLED_ShowNum(42, 12, (u32)WJ_task_2_get_state(), 1, 8, 1);
    OLED_ShowString(0, 24, (u8 *)"DIST:", 8, 1);
    OLED_ShowNum(36, 24,
                 (u32)(WJ_task_2_get_distance_mm() / 10.0f),
                 4, 8, 1);
    OLED_ShowString(66, 24, (u8 *)"cm", 8, 1);
    OLED_ShowString(0, 36, (u8 *)"L/R:", 8, 1);
    OLED_ShowSignedNum(24, 36, WJ_get_left_speed(), 3, 8, 1);
    OLED_ShowSignedNum(54, 36, WJ_get_right_speed(), 3, 8, 1);
    OLED_ShowString(84, 36, (u8 *)"RPM", 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K4 RESET", 8, 1);
}

static void eumshow_task1(int16_t k230_x, uint8_t tmc_ok)
{
    /* TASK1 调试页：只测“小球回 O 点”，不让循迹小车行驶。 */
    OLED_ShowString(0, 0, (u8 *)"BALL TEST", 8, 1);
    OLED_ShowString(72, 0, (u8 *)"TMC:", 8, 1);
    OLED_ShowNum(102, 0, tmc_ok, 1, 8, 1);
    OLED_ShowString(0, 12, (u8 *)"RUN:", 8, 1);
    OLED_ShowNum(30, 12, WJ_task_1_is_running(), 1, 8, 1);
    OLED_ShowString(0, 24, (u8 *)"TARGET: O=0", 8, 1);
    OLED_ShowString(0, 36, (u8 *)"K230 X:", 8, 1);
    OLED_ShowSignedNum(42, 36, k230_x, 4, 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K4 RESET", 8, 1);
}

static void eumshow_task3(int16_t k230_x, uint8_t tmc_ok)
{
    uint16_t elapsed_ms = WJ_task_3_get_time_ms();
    uint16_t elapsed_second = elapsed_ms / 1000U;
    uint16_t elapsed_tenth = (elapsed_ms / 100U) % 10U;

    /* TASK3 调试页：显示状态、用时、K230 横向偏差及步进驱动初始化结果。 */
    OLED_ShowString(0, 0, (u8 *)"TASK3", 8, 1);
    OLED_ShowString(72, 0, (u8 *)"TMC:", 8, 1);
    OLED_ShowNum(102, 0, tmc_ok, 1, 8, 1);
    OLED_ShowString(0, 12, (u8 *)"STATE:", 8, 1);
    OLED_ShowNum(42, 12, (u32)WJ_task_3_get_state(), 1, 8, 1);
    /* 题目限时 5 s：显示为 0.0/5s，进入 -5 cm 保持状态后该数字冻结。 */
    OLED_ShowString(0, 24, (u8 *)"TIME:", 8, 1);
    OLED_ShowNum(36, 24, elapsed_second, 1, 8, 1);
    OLED_ShowString(42, 24, (u8 *)".", 8, 1);
    OLED_ShowNum(48, 24, elapsed_tenth, 1, 8, 1);
    OLED_ShowString(54, 24, (u8 *)"/5s", 8, 1);
    OLED_ShowString(0, 36, (u8 *)"K230 X:", 8, 1);
    OLED_ShowSignedNum(42, 36, k230_x, 4, 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K4 RESET", 8, 1);
}

static void eumshow_task4(int16_t k230_x,
                          int32_t front_back_accel_mg,
                          uint8_t tmc_ok)
{
    /* TASK4 调试页：同时显示 K230 横向偏差和前后向加速度，便于校验前馈方向。 */
    OLED_ShowString(0, 0, (u8 *)"TASK4", 8, 1);
    OLED_ShowString(72, 0, (u8 *)"TMC:", 8, 1);
    OLED_ShowNum(102, 0, tmc_ok, 1, 8, 1);
    OLED_ShowString(0, 12, (u8 *)"RUN:", 8, 1);
    OLED_ShowNum(30, 12, WJ_task_4_is_running(), 1, 8, 1);
    OLED_ShowString(0, 24, (u8 *)"K230 X:", 8, 1);
    OLED_ShowSignedNum(42, 24, k230_x, 4, 8, 1);
    OLED_ShowString(0, 36, (u8 *)"FB:", 8, 1);
    OLED_ShowSignedNum(18, 36, front_back_accel_mg, 5, 8, 1);
    OLED_ShowString(54, 36, (u8 *)"mg", 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K4 RESET", 8, 1);
}

static void eumshow_task5(int16_t k230_x, uint8_t tmc_ok)
{
    uint32_t elapsed_ms = WJ_task_5_get_time_ms();
    uint32_t elapsed_second = elapsed_ms / 1000U;
    uint32_t elapsed_tenth = (elapsed_ms / 100U) % 10U;
    uint8_t diagnostic_count = WJ_task_5_get_diagnostic_count();
    uint8_t diagnostic_index;
    Task5_TurnDiagnostic diagnostic;

    OLED_ShowString(0, 0, (u8 *)"TASK5 A", 8, 1);
    OLED_ShowString(48, 0, (u8 *)"D:", 8, 1);
    if (diagnostic_count > 0U) {
        /* 每 2 s 轮换显示已锁存的右转记录，最多 D1～D4。 */
        diagnostic_index =
            (uint8_t)((elapsed_ms / 2000U) % diagnostic_count);
        OLED_ShowNum(60, 0, diagnostic_index + 1U, 1, 8, 1);
    } else {
        diagnostic_index = 0U;
        OLED_ShowNum(60, 0, 0U, 1, 8, 1);
    }
    OLED_ShowString(72, 0, (u8 *)"/", 8, 1);
    OLED_ShowNum(78, 0, diagnostic_count, 1, 8, 1);
    OLED_ShowString(90, 0, (u8 *)"T:", 8, 1);
    OLED_ShowNum(102, 0, tmc_ok, 1, 8, 1);

    OLED_ShowString(0, 12, (u8 *)"RUN:", 8, 1);
    OLED_ShowNum(30, 12, WJ_task_5_is_running(), 1, 8, 1);
    OLED_ShowString(48, 12, (u8 *)"T:", 8, 1);
    OLED_ShowNum(60, 12, elapsed_second % 100U, 2, 8, 1);
    OLED_ShowString(72, 12, (u8 *)".", 8, 1);
    OLED_ShowNum(78, 12, elapsed_tenth, 1, 8, 1);
    OLED_ShowString(84, 12, (u8 *)"s", 8, 1);

    OLED_ShowString(0, 24, (u8 *)"X:", 8, 1);
    OLED_ShowSignedNum(12, 24, k230_x, 4, 8, 1);

    if (WJ_task_5_get_turn_diagnostic(diagnostic_index,
                                      &diagnostic) != 0U) {
        OLED_ShowString(54, 24, (u8 *)"D:", 8, 1);
        OLED_ShowNum(66, 24,
                     (u32)diagnostic.control_exit_delay_ms,
                     4, 8, 1);
        OLED_ShowString(96, 24, (u8 *)"ms", 8, 1);

        /* IN/OUT 各显示有符号最大值和最小值，不取绝对值。 */
        OLED_ShowString(0, 36, (u8 *)"IN:", 8, 1);
        OLED_ShowSignedNum(18, 36, diagnostic.turn_max_x, 4, 8, 1);
        OLED_ShowString(48, 36, (u8 *)"/", 8, 1);
        OLED_ShowSignedNum(54, 36, diagnostic.turn_min_x, 4, 8, 1);

        OLED_ShowString(0, 52, (u8 *)"OUT:", 8, 1);
        OLED_ShowSignedNum(24, 52, diagnostic.exit_max_x, 4, 8, 1);
        OLED_ShowString(54, 52, (u8 *)"/", 8, 1);
        OLED_ShowSignedNum(60, 52, diagnostic.exit_min_x, 4, 8, 1);
    } else {
        OLED_ShowString(54, 24, (u8 *)"D:----ms", 8, 1);
        OLED_ShowString(0, 36, (u8 *)"WAIT RIGHT TURN", 8, 1);
        OLED_ShowString(0, 52, (u8 *)"A=1.5deg/10dps", 8, 1);
    }
}

static void eumshow_task6(int16_t k230_x, uint8_t tmc_ok)
{
    int16_t target_pixel = WJ_task_6_get_target_pixel();
    uint32_t elapsed_ms = WJ_task_6_get_time_ms();
    uint32_t elapsed_second = elapsed_ms / 1000U;
    uint32_t elapsed_tenth = (elapsed_ms / 100U) % 10U;

    if (task6_configuring != 0U) {
        OLED_ShowString(0, 0, (u8 *)"TASK6 SET TARGET", 8, 1);
        OLED_ShowString(0, 12, (u8 *)"TARGET:", 8, 1);
        OLED_ShowSignedNum(42, 12, target_pixel, 4, 8, 1);
        OLED_ShowString(72, 12, (u8 *)"px", 8, 1);
        OLED_ShowString(0, 24, (u8 *)"K230 X:", 8, 1);
        OLED_ShowSignedNum(42, 24, k230_x, 4, 8, 1);
        if (task6_key2_waiting_second != 0U) {
            OLED_ShowString(0, 38, (u8 *)"K2 AGAIN TO SAVE    ", 8, 1);
        } else if (task6_capture_failed != 0U) {
            OLED_ShowString(0, 38, (u8 *)"NO BALL - RETRY     ", 8, 1);
        } else if (task6_target_captured != 0U) {
            OLED_ShowString(0, 38, (u8 *)"TARGET RECORDED     ", 8, 1);
        } else {
            OLED_ShowString(0, 38, (u8 *)"K2 DOUBLE CLICK     ", 8, 1);
        }
        OLED_ShowString(0, 52,
                        (u8 *)((task6_target_captured != 0U) ?
                               "K3 RUN  K4 BACK" : "K3 WAIT K4 BACK"),
                        8, 1);
        return;
    }

    OLED_ShowString(0, 0, (u8 *)"TASK6", 8, 1);
    OLED_ShowString(72, 0, (u8 *)"TMC:", 8, 1);
    OLED_ShowNum(102, 0, tmc_ok, 1, 8, 1);
    OLED_ShowString(0, 12, (u8 *)"RUN:", 8, 1);
    OLED_ShowNum(30, 12, WJ_task_6_is_running(), 1, 8, 1);
    OLED_ShowString(48, 12, (u8 *)"TGT:", 8, 1);
    OLED_ShowSignedNum(72, 12, target_pixel, 4, 8, 1);
    OLED_ShowString(0, 24, (u8 *)"K230 X:", 8, 1);
    OLED_ShowSignedNum(42, 24, k230_x, 4, 8, 1);
    OLED_ShowString(0, 36, (u8 *)"TIME:", 8, 1);
    OLED_ShowNum(36, 36, elapsed_second % 100U, 2, 8, 1);
    OLED_ShowString(48, 36, (u8 *)".", 8, 1);
    OLED_ShowNum(54, 36, elapsed_tenth, 1, 8, 1);
    OLED_ShowString(60, 36, (u8 *)"/30s", 8, 1);
    OLED_ShowString(0, 52, (u8 *)"K4 RESET", 8, 1);
}

void eumshow(int16_t k230_x,
             int32_t front_back_accel_mg,
             int32_t left_right_accel_mg,
             uint8_t tmc_ok)
{
    /* 左右向加速度暂不占用菜单行；前后向加速度在 TASK4 页面显示。 */
    (void)left_right_accel_mg;

    /* 仅在页面切换时清屏，避免每 100 ms 全屏闪烁。 */
    if (clear != 0U) {
        OLED_Clear();
        clear = 0U;
    }

    /* Mode=0 显示菜单；Mode=1～6 显示对应任务页面。 */
    if (Mode == 0U) {
        eumshow_main();
    } else if (Mode == 1U) {
        eumshow_task1(k230_x, tmc_ok);
    } else if (Mode == 2U) {
        eumshow_task2();
    } else if (Mode == 3U) {
        eumshow_task3(k230_x, tmc_ok);
    } else if (Mode == 4U) {
        eumshow_task4(k230_x, front_back_accel_mg, tmc_ok);
    } else if (Mode == 5U) {
        eumshow_task5(k230_x, tmc_ok);
    } else {
        eumshow_task6(k230_x, tmc_ok);
    }
    OLED_Refresh();
}
