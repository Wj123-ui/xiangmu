/*
 * eum.c — 菜单系统 + 任务调度 (按键/LCD/任务分发)
 *
 * 硬件:
 *   4 个按键 (GPIOE 0~3): key1=上/加, key2=下/减, key3=进入/启动/退出, key4=编辑
 *   LCD: ILI9341 240×320, SPI1
 *
 * 菜单层级:
 *   level 0 — 任务选择: key1/key2 移动光标, key3 进入/启动
 *   level 1 — 任务运行: 普通任务 key3 退出，task4 快速双击 key3 退出
 *   level 2 — task4 参数页: key1/key2 选择或调节, key4 进入/退出编辑,
 *             快速双击 key3 启动; 单击或双击超时退出 task4
 *
 * 任务调度:
 *   level 1 时, 主循环调用 eum_task_dispatch(),
 *   读取 OpenMV 数据并分发到对应 task 函数.
 */
#include "eum.h"
#include "main.h"
#include "ili9341.h"
#include "control.h"
#include "task.h"
#include "opmv.h"

/* ========== 按键定义 (GPIOE 0~3, 低电平有效) ========== */
#define key1 (LL_GPIO_ReadInputPort(GPIOE) & LL_GPIO_PIN_0)
#define key2 (LL_GPIO_ReadInputPort(GPIOE) & LL_GPIO_PIN_1)
#define key3 (LL_GPIO_ReadInputPort(GPIOE) & LL_GPIO_PIN_2)
#define key4 (LL_GPIO_ReadInputPort(GPIOE) & LL_GPIO_PIN_3)

/* ========== 全局变量 ========== */
uint8_t key_num, eumlevel;       /* eumlevel: 0=菜单 1=运行 2=参数 */
uint8_t num1, num2, num3, shuaxin = 1;

/* 中断只投递事件, 菜单和任务切换全部在主循环执行 */
static volatile uint8_t key_event = 0;

/* task4 参数页状态 */
static uint8_t param_selected = 0;      /* 0=幅值 A, 1=相位 ψ */
static uint8_t param_editing = 0;       /* 0=选择参数, 1=调节当前参数 */
static uint8_t task4_key3_pending = 0;  /* 等待第二次 key3 */
static uint32_t task4_key3_deadline = 0;
static uint8_t task4_run_key3_pending = 0; /* Task4 运行中等待双击退出 */
static uint32_t task4_run_key3_deadline = 0;

#define KEY_SCAN_DIVIDER          5U    /* TIM1 为 1ms, 每 5ms 采样一次 */
#define KEY_DEBOUNCE_SAMPLES      4U    /* 连续稳定 20ms */
#define TASK4_DOUBLE_CLICK_MS   400U

/* task4 参数表 (与 task.c 中一致) */
static const float amp_tbl[] = {3.0f, 5.0f, 7.5f, 10.0f};
static const float pha_tbl[] = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f};

/* ========== 任务名称 (LCD 显示用) ========== */
static const char *task_names[] = {
    "1.Aim 2s",        /* task1: 2s瞄准中心 */
    "2.Draw sin",      /* task2: 画sin(x) */
    "3.Aim 4s",        /* task3: 4s瞄准中心 */
    "4.Draw A*sin",    /* task4: 画Asin(x+ψ) */
    "5.Scan rect",     /* task5: 扫描矩形轮廓 */
    "6.Digit write",   /* task6: 数字识别+书写 */
    "7.Copy line",     /* task7: 线条复绘 */
};

static const char *task_desc[] = {
    "Aim center 2s",   "Draw sin(x)",    "Aim center 4s",
    "Draw A*sin(x+p)", "Scan contour",   "Recognize+write",
    "Copy handdraw",
};

/* 底部串口数据行在数值或页面变化时才刷新，避免 LCD 拖慢主循环。 */
static uint8_t uart_xy_force_redraw = 1;

static uint8_t AppendSigned5(char *buf, uint8_t pos, int16_t value)
{
    uint16_t magnitude;

    if (value < 0) {
        buf[pos++] = '-';
        magnitude = (uint16_t)(-(int32_t)value);
    } else {
        buf[pos++] = '+';
        magnitude = (uint16_t)value;
    }

    buf[pos++] = (char)('0' + (magnitude / 10000U) % 10U);
    buf[pos++] = (char)('0' + (magnitude / 1000U)  % 10U);
    buf[pos++] = (char)('0' + (magnitude / 100U)   % 10U);
    buf[pos++] = (char)('0' + (magnitude / 10U)    % 10U);
    buf[pos++] = (char)('0' + magnitude % 10U);
    return pos;
}

static uint8_t AppendUnsigned4(char *buf, uint8_t pos, uint32_t value)
{
    value %= 10000U;
    buf[pos++] = (char)('0' + (value / 1000U) % 10U);
    buf[pos++] = (char)('0' + (value / 100U)  % 10U);
    buf[pos++] = (char)('0' + (value / 10U)   % 10U);
    buf[pos++] = (char)('0' + value % 10U);
    return pos;
}

/* ========== 按键读取 (低电平有效) ========== */
static uint8_t Key_GetStatus(void)
{
    if (key1 == 0) return 1;
    if (key2 == 0) return 2;
    if (key3 == 0) return 3;
    if (key4 == 0) return 4;
    return 0;
}

/* ========== 按键扫描 (TIM1 中断中调用, 仅产生释放事件) ========== */
void tick(void)
{
    static uint8_t scan_divider = 0;
    static uint8_t raw_last = 0;
    static uint8_t stable_key = 0;
    static uint8_t stable_samples = 0;
    uint8_t raw;

    if (++scan_divider < KEY_SCAN_DIVIDER) return;
    scan_divider = 0;

    raw = Key_GetStatus();
    if (raw != raw_last) {
        raw_last = raw;
        stable_samples = 1;
        return;
    }
    if (stable_samples < KEY_DEBOUNCE_SAMPLES) stable_samples++;
    if (stable_samples < KEY_DEBOUNCE_SAMPLES || raw == stable_key) return;

    /* 释放时产生一次事件; 主循环足够快, 单槽队列不会积压正常按键 */
    {
        uint8_t released_key = stable_key;
        stable_key = raw;
        if (raw == 0 && released_key != 0 && key_event == 0) {
            key_event = released_key;
        }
    }
}

static uint8_t TimeReached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static void ReturnToMenu(void)
{
    Task_Reset();
    K230_SendTaskMode(0);
    task4_key3_pending = 0;
    task4_run_key3_pending = 0;
    param_editing = 0;
    eumlevel = 0;
    shuaxin = 1;
}

static void StartSelectedTask(void)
{
    Task_Reset();
    K230_SendTaskMode(num1 + 1);
    eumlevel = 1;
    shuaxin = 1;
}

static void EnterTask4Setup(void)
{
    Task_Reset();
    K230_SendTaskMode(0);
    param_selected = 0;
    param_editing = 0;
    task4_key3_pending = 0;
    task4_run_key3_pending = 0;
    eumlevel = 2;
    shuaxin = 1;
}

static void StartTask4(void)
{
    Task_Reset();
    task4_state = T4_FIND;
    K230_SendTaskMode(4);
    task4_key3_pending = 0;
    task4_run_key3_pending = 0;
    param_editing = 0;
    eumlevel = 1;
    shuaxin = 1;
}

/* ========== 菜单状态机 (仅在主循环调用) ========== */
static void EUM_Process(void)
{
    uint8_t event;
    uint32_t now = HAL_GetTick();
    uint32_t primask;

    /* key3 单击超过双击窗口: 退出 task4 参数页 */
    if (eumlevel == 2 && task4_key3_pending &&
        TimeReached(now, task4_key3_deadline)) {
        key_event = 0;
        ReturnToMenu();
        return;
    }

    /* Task4 运行中的单击不退出；超时后只取消等待第二击。 */
    if (eumlevel == 1 && num1 == 3 && task4_run_key3_pending &&
        TimeReached(now, task4_run_key3_deadline)) {
        task4_run_key3_pending = 0;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    event = key_event;
    key_event = 0;
    if (primask == 0U) __enable_irq();
    if (event == 0) return;
    key_num = event;

    switch (eumlevel) {
    case 0: /* 任务选择 */
        if (event == 1) {
            num1 = (num1 == 0) ? 6 : (uint8_t)(num1 - 1);
            shuaxin = 1;
        } else if (event == 2) {
            num1 = (uint8_t)((num1 + 1) % 7);
            shuaxin = 1;
        } else if (event == 3) {
            if (num1 == 3) EnterTask4Setup();
            else StartSelectedTask();
        }
        break;

    case 1: /* 任务运行 */
        if (event == 3) {
            if (num1 == 3) {
                if (!task4_run_key3_pending) {
                    task4_run_key3_pending = 1;
                    task4_run_key3_deadline = now + TASK4_DOUBLE_CLICK_MS;
                } else {
                    ReturnToMenu();
                }
            } else {
                ReturnToMenu();
            }
        }
        break;

    case 2: /* task4 参数页 */
        /* 第一次 key3 后只接受第二次 key3; 其他键不能取消“单击退出” */
        if (task4_key3_pending && event != 3) break;
        if (event == 1 || event == 2) {
            if (!param_editing) {
                /* 只有两个参数: key1 选 A, key2 选 ψ */
                param_selected = (event == 1) ? 0 : 1;
            } else if (param_selected == 0) {
                if (event == 1 && task4_amp_idx < 3) task4_amp_idx++;
                if (event == 2 && task4_amp_idx > 0) task4_amp_idx--;
                task4_amplitude = amp_tbl[task4_amp_idx];
            } else {
                if (event == 1 && task4_pha_idx < 6) task4_pha_idx++;
                if (event == 2 && task4_pha_idx > 0) task4_pha_idx--;
                task4_phase = pha_tbl[task4_pha_idx];
            }
            shuaxin = 1;
        } else if (event == 4) {
            param_editing = !param_editing;
            shuaxin = 1;
        } else if (event == 3) {
            if (!task4_key3_pending) {
                task4_key3_pending = 1;
                task4_key3_deadline = now + TASK4_DOUBLE_CLICK_MS;
                shuaxin = 1;
            } else {
                StartTask4();
            }
        }
        break;
    }
}

/* ========== LCD 显示 (主循环调用) ========== */
void show_eum(void)
{
    if (shuaxin == 0) return;
    shuaxin = 0;
    ILI9341_FillScreen(ILI9341_COLOR_BLACK);
    uart_xy_force_redraw = 1;

    switch (eumlevel) {
    /* ----- Level 0: 任务列表 ----- */
    case 0:
        ILI9341_DrawString(0, 0, "== E-Task Menu ==",
            ILI9341_COLOR_CYAN, ILI9341_COLOR_BLACK, 2);
        for (uint8_t i = 0; i < 7; i++) {
            uint16_t color = (i == num1) ? ILI9341_COLOR_BLACK : ILI9341_COLOR_WHITE;
            uint16_t bg    = (i == num1) ? ILI9341_COLOR_GREEN : ILI9341_COLOR_BLACK;
            ILI9341_DrawString(10, 30 + i * 24, task_names[i], color, bg, 2);
            /* 选中行显示描述 */
            if (i == num1) {
                ILI9341_DrawString(130, 30 + i * 24, "<--",
                    ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, 2);
            }
        }
        ILI9341_DrawString(0, 250, "1/2:Sel 3:Enter/Run",
            ILI9341_COLOR_GRAY, ILI9341_COLOR_BLACK, 1);
        break;

    /* ----- Level 1: 任务运行中 ----- */
    case 1:
        ILI9341_DrawString(0, 0, "Running:",
            ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, 2);
        if (num1 < 7) {
            ILI9341_DrawString(10, 25, task_names[num1],
                ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, 2);
            ILI9341_DrawString(10, 50, task_desc[num1],
                ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 1);
        }
        ILI9341_DrawString(0, 250,
            (num1 == 3) ?
                (task4_run_key3_pending ? "3 again: STOP" : "Double 3:Stop") :
                "3:Stop",
            ILI9341_COLOR_GRAY, ILI9341_COLOR_BLACK, 1);
        /* task1/3 显示运行时间 */
        if (num1 == 0 || num1 == 2) {
            ILI9341_DrawString(10, 80, "Aiming...",
                ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK, 2);
        }
        if (num1 == 3) {
            ILI9341_DrawString(10, 80, "Aiming, then draw...",
                ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK, 1);
        }
        break;

    /* ----- Level 2: task4 参数选择/编辑 ----- */
    case 2:
        ILI9341_DrawString(0, 0, "Task4 Params",
            ILI9341_COLOR_CYAN, ILI9341_COLOR_BLACK, 2);

        /* 幅值 A */
        {
            char buf[20];
            int a10 = (int)(task4_amplitude * 10.0f + 0.5f);
            uint16_t color = (param_selected == 0) ?
                (param_editing ? ILI9341_COLOR_YELLOW : ILI9341_COLOR_GREEN) : ILI9341_COLOR_WHITE;
            buf[0]='A'; buf[1]='=';
            if (a10 >= 100) {
                buf[2]='1'; buf[3]='0'; buf[4]='.'; buf[5]='0';
                buf[6]='c'; buf[7]='m'; buf[8]=0;
            } else {
                buf[2]='0'+(a10/10); buf[3]='.'; buf[4]='0'+(a10%10);
                buf[5]='c'; buf[6]='m'; buf[7]=0;
            }
            ILI9341_DrawString(10, 50, buf, color, ILI9341_COLOR_BLACK, 2);
            if (param_selected == 0)
                ILI9341_DrawString(150, 50, param_editing ? "EDIT" : "<--",
                    ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, 1);
        }
        /* 相位 ψ */
        {
            char buf[20];
            uint16_t color = (param_selected == 1) ?
                (param_editing ? ILI9341_COLOR_YELLOW : ILI9341_COLOR_GREEN) : ILI9341_COLOR_WHITE;
            int p = (int)task4_phase;
            buf[0]='P'; buf[1]='='; buf[2]='0'+(p/100); buf[3]='0'+((p/10)%10);
            buf[4]='0'+(p%10); buf[5]='d'; buf[6]='e'; buf[7]='g'; buf[8]=0;
            ILI9341_DrawString(10, 90, buf, color, ILI9341_COLOR_BLACK, 2);
            if (param_selected == 1)
                ILI9341_DrawString(150, 90, param_editing ? "EDIT" : "<--",
                    ILI9341_COLOR_RED, ILI9341_COLOR_BLACK, 1);
        }
        if (param_editing) {
            ILI9341_DrawString(0, 150, "1:+ 2:- 4:Done",
                ILI9341_COLOR_GRAY, ILI9341_COLOR_BLACK, 1);
        } else {
            ILI9341_DrawString(0, 150, "1/2:Select 4:Edit",
                ILI9341_COLOR_GRAY, ILI9341_COLOR_BLACK, 1);
        }
        ILI9341_DrawString(0, 175,
            task4_key3_pending ? "3 again: RUN" : "Double 3:Run  Single:Exit",
            task4_key3_pending ? ILI9341_COLOR_YELLOW : ILI9341_COLOR_GRAY,
            ILI9341_COLOR_BLACK, 1);
        break;
    }
}

/* 屏幕底部显示原始帧、解析结果和接收统计。 */
void show_uart_xy(void)
{
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static uint8_t last_type = 0;
    static uint8_t last_status = 0;
    static uint32_t last_byte_count = 0;
    static uint32_t last_frame_count = 0;
    static uint32_t last_error_count = 0;
    static uint32_t last_draw_tick = 0;
    uint32_t now = HAL_GetTick();
    uint32_t byte_count = uart_rx_byte_count;
    uint32_t frame_count = uart_rx_frame_count;
    uint32_t error_count = uart_rx_error_count;
    int16_t x = uart_data.err_x;
    int16_t y = uart_data.err_y;
    uint8_t type = uart_data.type;
    char buf[40];
    char raw[32];
    uint8_t raw_len;
    uint8_t i;
    uint8_t pos = 0;

    /* 数据行最快20 Hz刷新；解析和电机控制不受此显示限速影响。 */
    if (!uart_xy_force_redraw && (uint32_t)(now - last_draw_tick) < 50U) {
        return;
    }
    if (!uart_xy_force_redraw && x == last_x && y == last_y && type == last_type &&
        uart_data.status == last_status &&
        byte_count == last_byte_count && frame_count == last_frame_count &&
        error_count == last_error_count) {
        return;
    }
    uart_xy_force_redraw = 0;
    last_draw_tick = now;
    last_x = x;
    last_y = y;
    last_type = type;
    last_status = uart_data.status;
    last_byte_count = byte_count;
    last_frame_count = frame_count;
    last_error_count = error_count;

    /* 先复制中断正在更新的原始预览，长度受限且始终补 '\0'。 */
    raw_len = uart_rx_preview_len;
    if (raw_len > sizeof(raw) - 1U) raw_len = sizeof(raw) - 1U;
    for (i = 0; i < raw_len; i++) raw[i] = (char)uart_rx_preview[i];
    raw[raw_len] = '\0';

    ILI9341_FillRect(0, 268, ILI9341_WIDTH, 52, ILI9341_COLOR_BLACK);
    pos = 0;
    buf[pos++] = 'R'; buf[pos++] = 'A'; buf[pos++] = 'W'; buf[pos++] = ':';
    for (i = 0; i < raw_len && pos < sizeof(buf) - 1U; i++) {
        char ch = raw[i];
        buf[pos++] = (ch >= ' ' && ch <= '~') ? ch : '.';
    }
    buf[pos] = '\0';
    ILI9341_DrawString(0, 270, buf,
        ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 1);

    pos = 0;
    buf[pos++] = 'R'; buf[pos++] = 'X'; buf[pos++] = ':';
    buf[pos++] = type ? (char)type : '-';
    buf[pos++] = ' '; buf[pos++] = 'X'; buf[pos++] = ':';
    pos = AppendSigned5(buf, pos, x);
    buf[pos++] = ' '; buf[pos++] = 'Y'; buf[pos++] = ':';
    pos = AppendSigned5(buf, pos, y);
    buf[pos++] = ' '; buf[pos++] = 'Z'; buf[pos++] = ':';
    buf[pos++] = uart_data.status ? '1' : '0';
    buf[pos] = '\0';

    ILI9341_DrawString(0, 286, buf,
        type == 'A' ? ILI9341_COLOR_GREEN : ILI9341_COLOR_YELLOW,
        ILI9341_COLOR_BLACK, 1);

    pos = 0;
    buf[pos++] = 'B'; buf[pos++] = ':';
    pos = AppendUnsigned4(buf, pos, byte_count);
    buf[pos++] = ' '; buf[pos++] = 'F'; buf[pos++] = ':';
    pos = AppendUnsigned4(buf, pos, frame_count);
    buf[pos++] = ' '; buf[pos++] = 'E'; buf[pos++] = ':';
    pos = AppendUnsigned4(buf, pos, error_count);
    buf[pos] = '\0';
    ILI9341_DrawString(0, 302, buf,
        error_count ? ILI9341_COLOR_RED : ILI9341_COLOR_CYAN,
        ILI9341_COLOR_BLACK, 1);
}

/* ========== 任务调度 (主循环调用) ========== */
/**
  * @brief  在 level 1 时读取 OpenMV 数据并分发到对应任务
  * @note   非任务运行时 (level != 1) 不执行任何操作
  *         uart_data.err_x/err_y 持久保存上次接收值, 无新数据时沿用
  */
void eum_task_dispatch(void)
{
    uint8_t has_new_data;
    uint8_t drawing_open_loop;

    EUM_Process();
    if (eumlevel != 1) return;

    /* uart_data 中的 err_x/err_y 持久保存, 即使无新数据也可用 */
    int x = uart_data.err_x;
    int y = uart_data.err_y;
    has_new_data = uart_data.updated;
    if (has_new_data) {
        uart_data.updated = 0;
    }

    /*
     * task1~4 依赖 $A 瞄准误差。不能用上电默认 (0,0) 或 $R 就绪帧
     * 启动任务，否则 task1 会立即判定已对准，且 2 s 超时会过早起计。
     */
    /* Task2/4 进入移笔或绘图后为开环轨迹，不能因 K230 偶发 Z=0 而暂停。 */
    drawing_open_loop =
        ((num1 == 1U && Task_IsRunning() && task2_state != T2_FIND) ||
         (num1 == 3U && Task_IsRunning() &&
          task4_state != T4_INPUT && task4_state != T4_FIND)) ? 1U : 0U;
    /* Task4 已启动后允许 Z 短暂丢失，使 task4() 的矄准超时仍能执行。 */
    if (num1 <= 3U && !drawing_open_loop &&
        !(num1 == 3U && Task_IsRunning() && task4_state == T4_FIND) &&
        (uart_data.type != 'A' || uart_data.status == 0U)) {
        return;
    }

    /* 第一次必须是刚收到的有效 $A 帧；启动后可沿用最后一帧。 */
    if (num1 <= 3U && !Task_IsRunning() && !has_new_data) {
        return;
    }

    /* 分发到选中任务 (num1: 0~6 → task 1~7) */
    Task_Run(num1 + 1, x, y);
}

/* ========== TIM1 中断 (20ms 周期) ========== */
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
        tick();                      /* 按键处理 */
        TMC2209_SpeedRamp_Update();  /* 电机斜坡更新 (task_direct_control=1时自动跳过) */
        LL_TIM_ClearFlag_UPDATE(TIM1);
    }
}
