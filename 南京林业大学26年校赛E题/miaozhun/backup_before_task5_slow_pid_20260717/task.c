/*
 * task.c — 竞赛任务实现 (2026南林大电赛 E题: 自动瞄准及绘图装置)
 *
 * 控制架构:
 *   OpenMV(UART4) → (err_x, err_y) → task_x() → TMC2209 电机控制
 *
 * 瞄准任务 (task1/3): PID 闭环, OpenMV 误差驱动云台对准靶心
 * 绘图任务 (task2/4): 开环轨迹生成, 两路 TMC2209 VACTUAL 独立调速
 * 发挥任务 (task5-7): OpenMV 提供目标点/识别结果, STM32 执行运动
 *
 * VACTUAL 双轴独立调速原理:
 *   TMC2209 的 VACTUAL 寄存器可内部生成 STEP 脉冲, 无需 TIM3.
 *   绘图时两轴分别通过各自 UART 设置 VACTUAL，频率完全独立，
 *   解决 TIM3 两通道共享 ARR 的限制.
 */
#include "task.h"
#include "PID.h"
#include "control.h"
#include "ili9341.h"
#include "opmv.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif

/* ========== PID 控制器 ==========
 * Yaw 的 KP/KI 为负: OpenMV x 误差正向与 Yaw 电机转向相反
 * Outmax/Outmin = ±50: 限制角速度 ±50°/s, 防止云台甩过头 */
PID_T yuntaix = {.KP=0.10f, .KI=0.0f, .KD=0.05, .Outmax=50, .Outmin=-50};
PID_T yuntaiy = {.KP= 0.10f, .KI=0.0f, .KD=0.05, .Outmax=50, .Outmin=-50};

/* ========== 任务运行状态 ========== */
static uint8_t     task_running    = 0;       /* 当前是否有任务运行 */
static uint8_t     current_task_id = 0;       /* 当前任务编号 1~7 */
static uint32_t    task_start_tick = 0;       /* 任务启动时刻 (HAL_GetTick) */
static uint32_t    draw_start_tick = 0;       /* 绘图启动时刻 */
static uint32_t    task2_motion_deadline = 0; /* Task2 移到起点的结束时刻 */
static uint32_t    task4_motion_deadline = 0; /* Task4 移到参数波形起点的结束时刻 */
static uint8_t     laser_on        = 0;       /* 激光开关状态 */

volatile uint8_t task_direct_control = 0;      /* 1=任务直接控电机, 斜坡跳过 */

/* LCD 波形绘制状态 (task4 用) */
static uint8_t lcd_wave_initialized = 0;
static uint16_t lcd_wave_last_x = 20, lcd_wave_last_y = 160;

/* task5-7 完成标志 (需在 Task_Reset 中重置) */
static uint8_t task5_done = 0;
static uint8_t task6_done = 0;
static uint8_t task7_done = 0;

typedef enum {
    T6_IDLE = 0,
    T6_AIM,
    T6_NEXT_SEGMENT,
    T6_BLANK_BEFORE_MOVE,
    T6_MOVE_TO_START,
    T6_SETTLE_AT_START,
    T6_DRAW_SEGMENT,
    T6_DONE,
} Task6Phase_t;

uint8_t task6_digit = TASK6_DEFAULT_DIGIT;
static Task6Phase_t task6_phase = T6_IDLE;
static uint8_t task6_segment_index = 0;
static uint32_t task6_motion_deadline = 0;
static float task6_current_x_cm = 0.0f;
static float task6_current_y_cm = 0.0f;
static float task6_target_x_cm = 0.0f;
static float task6_target_y_cm = 0.0f;
static uint8_t task6_lcd_initialized = 0;

/* task2 状态 */
Task2State_t task2_state = T2_FIND;
/* task4 状态 */
Task4State_t task4_state = T4_INPUT;

/* ========== task4 可调参数 ========== */
float   task4_amplitude = 7.5f;
float   task4_phase     = 0.0f;
uint8_t task4_amp_idx   = 9;  /* 3.0 + 9×0.5 = 默认 7.5cm */
uint8_t task4_pha_idx   = 0;  /* 默认 0° */

/* ========== 内部辅助函数声明 ========== */
static void aim_pid(int x, int y);
static uint8_t is_centered(int x, int y);
static void lock_motors(void);
static void sine_stop_motion(void);
static void draw_sine_wave(float amplitude, float phase_deg, uint8_t show_lcd);
static void lcd_draw_waveform(float amplitude, float phase_deg);
static uint8_t append_amplitude(char *buf, uint8_t pos, float amplitude);

/* 将 3.0~10.0 正确写入字符串，兼容两位整数部分 */
static uint8_t append_amplitude(char *buf, uint8_t pos, float amplitude)
{
    int a10 = (int)(amplitude * 10.0f + 0.5f);

    if (a10 >= 100) {
        buf[pos++] = '0' + ((a10 / 100) % 10);
        buf[pos++] = '0' + ((a10 / 10) % 10);
    } else {
        buf[pos++] = '0' + ((a10 / 10) % 10);
    }
    buf[pos++] = '.';
    buf[pos++] = '0' + (a10 % 10);
    return pos;
}

/* ========================================================================== */
/*                           任务管理                                          */
/* ========================================================================== */

/**
  * @brief  重置所有任务状态, 供切换任务时调用
  */
void Task_Reset(void)
{
    task_running       = 0;
    /* 先冻结斜坡，防止清理过程中旧目标速度重新驱动电机。 */
    task_direct_control = 1;
    laser_on           = 0;
    Laser_Set(0);

    /* 清除斜坡控制器的旧目标；大加速度让下一次更新立即归零。 */
    TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
    TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);

    /* 停止所有电机输出 */
    TMC2209_StopVACTUAL(AXIS_YAW);
    TMC2209_StopVACTUAL(AXIS_PITCH);
    TMC2209_Stop(AXIS_YAW);
    TMC2209_Stop(AXIS_PITCH);
    TMC2209_Disable(AXIS_YAW);
    TMC2209_Disable(AXIS_PITCH);
    task_direct_control = 0;

    /* 重置 PID 累积 */
    yuntaix.Out = 0; yuntaix.error0 = 0; yuntaix.error1 = 0; yuntaix.error2 = 0;
    yuntaiy.Out = 0; yuntaiy.error0 = 0; yuntaiy.error1 = 0; yuntaiy.error2 = 0;

    /* 重置状态机 */
    task2_state = T2_FIND;
    task2_motion_deadline = 0;
    task4_state = T4_INPUT;
    task4_motion_deadline = 0;

    /* 重置 LCD 波形绘制状态 */
    lcd_wave_initialized = 0;

    /* 重置 task5-7 完成标志 */
    task5_done = 0;
    task6_done = 0;
    task7_done = 0;
    task6_phase = T6_IDLE;
    task6_segment_index = 0;
    task6_motion_deadline = 0;
    task6_current_x_cm = 0.0f;
    task6_current_y_cm = 0.0f;
    task6_target_x_cm = 0.0f;
    task6_target_y_cm = 0.0f;
    task6_lcd_initialized = 0;
}

/**
  * @brief  调度执行指定任务
  * @param  task_id  任务编号 1~7
  * @param  x        OpenMV 水平误差 (像素)
  * @param  y        OpenMV 垂直误差 (像素)
  */
void Task_Run(uint8_t task_id, int x, int y)
{
    if (task_id < 1 || task_id > 7) return;

    /* 首次调用: 记录启动时刻 */
    if (!task_running || current_task_id != task_id) {
        current_task_id = task_id;
        task_running    = 1;
        task_start_tick = HAL_GetTick();
    }

    switch (task_id) {
        case 1: task1(x, y); break;
        case 2: task2(x, y); break;
        case 3: task3(x, y); break;
        case 4: task4(x, y); break;
        case 5: task5(x, y); break;
        case 6: task6(x, y); break;
        case 7: task7(x, y); break;
    }
}

uint8_t Task_IsRunning(void)
{
    return task_running;
}

/* ========================================================================== */
/*                           内部辅助函数                                      */
/* ========================================================================== */

/**
  * @brief  PID 瞄准靶心: 将 OpenMV 误差驱动到 0
  */
static void aim_pid(int x, int y)
{
    yuntaix.target = 0;
    yuntaiy.target = 0;
    yuntaix.actual = (float)x;
    yuntaiy.actual = (float)y;
    PID_Update(&yuntaix);
    PID_Update(&yuntaiy);

    /* PID 输出 = 角速度 (°/s), 通过斜坡控制电机 */
    TMC2209_SetAngleSpeed(AXIS_PITCH, yuntaix.Out, 50);
    TMC2209_SetAngleSpeed(AXIS_YAW,   yuntaiy.Out, 50);
}

/**
  * @brief  判断激光是否已对准靶心 (误差在阈值内)
  */
static uint8_t is_centered(int x, int y)
{
    /* Z=2 只是部分黑框的搜索引导，只有 Z=1 才是真实靶心。 */
    if (uart_data.type != 'A' || uart_data.status != 1U) return 0;
    return (x >= -AIM_THRESHOLD_PIXEL && x <= AIM_THRESHOLD_PIXEL &&
            y >= -AIM_THRESHOLD_PIXEL && y <= AIM_THRESHOLD_PIXEL);
}

/**
  * @brief  锁定电机: 停止运动, 使能保持电流锁定位置
  */
static void lock_motors(void)
{
    TMC2209_SetAngleSpeed(AXIS_YAW,   0, 500);
    TMC2209_SetAngleSpeed(AXIS_PITCH, 0, 500);
    TMC2209_Enable(AXIS_YAW);
    TMC2209_Enable(AXIS_PITCH);
}

/* 正弦绘图使用两路 TMC2209 内部脉冲，完全避开 TIM3 共用 ARR。 */
static void sine_stop_motion(void)
{
    TMC2209_StopVACTUAL(AXIS_YAW);
    TMC2209_StopVACTUAL(AXIS_PITCH);
    TMC2209_Stop(AXIS_YAW);
    TMC2209_Stop(AXIS_PITCH);
}

/**
  * @brief  绘制正弦波 (核心绘图函数)
  *
  *         轨迹: x 从 -11cm 线性扫到 +11cm, y = A*sin(2π*x/11 + ψ)
  *         实际接线为屏幕 x→PITCH、屏幕 y→YAW，两轴均用 VACTUAL
  *         两轴独立调速，不受 TIM3 共享 ARR 限制
  *
  *         调用前提: task_direct_control=1, 斜坡已跳过
  *
  * @param  amplitude  幅值 A (cm)
  * @param  phase_deg  相位 ψ (度)
  * @param  show_lcd   是否在 LCD 上同步绘制波形
  */
static void draw_sine_wave(float amplitude, float phase_deg, uint8_t show_lcd)
{
    uint32_t elapsed = HAL_GetTick() - draw_start_tick;
    float t = (float)elapsed / (float)DRAW_DURATION_MS;  /* 归一化时间 0~1 */
    float draw_sec;
    float phase_rad;
    float x_screen;
    float y_screen;
    float vx;
    float vy;
    float x_axis_vel;
    float y_axis_vel;

    if (t >= 1.0f) {
        /* 绘图完成: 停止输出 */
        sine_stop_motion();
        return;
    }

    /* --- 屏幕坐标轨迹 (cm) ---
     * x: -11 → +11 线性扫描 (总长 22cm)
     * y: A * sin(2π * 2周期 * t + ψ)
     *   波长 = 22/2 = 11cm, 空间频率 = 2π/11
     */
    x_screen = SINE_X_SCALE *
               (-SINE_LENGTH_CM / 2.0f + SINE_LENGTH_CM * t);
    phase_rad = phase_deg * (float)M_PI / 180.0f;
    y_screen = SINE_Y_SCALE * amplitude *
               sinf(2.0f * (float)M_PI * SINE_PERIODS * t + phase_rad);

    /* --- 角速度计算 (°/s) ---
     * θ=atan(s/D)，所以 dθ/dt=D/(D²+s²)·ds/dt。
     * 比 v/D 小角度近似更准确，能减小波形两端的尺寸误差。
     */
    draw_sec = (float)DRAW_DURATION_MS / 1000.0f;
    vx = SINE_X_SCALE * SINE_LENGTH_CM / draw_sec;
    vy = SINE_Y_SCALE * amplitude * 2.0f * (float)M_PI *
         (float)SINE_PERIODS / draw_sec *
         cosf(2.0f * (float)M_PI * SINE_PERIODS * t + phase_rad);
    x_axis_vel = 57.29578f * SCREEN_DISTANCE_CM * vx /
                 (SCREEN_DISTANCE_CM * SCREEN_DISTANCE_CM + x_screen * x_screen);
    y_axis_vel = 57.29578f * SCREEN_DISTANCE_CM * vy /
                 (SCREEN_DISTANCE_CM * SCREEN_DISTANCE_CM + y_screen * y_screen);

    /* 保持工程现有真实映射：x→PITCH，y→YAW；不改变量或轴的命名。 */
    TMC2209_SetVACTUAL(AXIS_PITCH, SINE_X_MOTOR_SIGN * x_axis_vel);
    TMC2209_SetVACTUAL(AXIS_YAW,   SINE_Y_MOTOR_SIGN * y_axis_vel);

    /* --- LCD 同步绘制波形 (task4 用) --- */
    if (show_lcd) {
        lcd_draw_waveform(amplitude, phase_deg);
    }
}

/**
  * @brief  在 LCD 上绘制正弦波形和函数表达式 (task4 用)
  *
  *         LCD 240×320, 横屏使用 240(宽)×320(高)
  *         波形区域: x=[20,220], y=[80,240], 中心线 y=160
  */
static void lcd_draw_waveform(float amplitude, float phase_deg)
{
    uint16_t i;
    float phase_rad;
    float lcd_amp_px;
    char buf[32];
    int p_int;
    uint8_t pos;

    /* LCD 只画一次完整图形，不在电机轨迹中反复刷 SPI。 */
    if (lcd_wave_initialized) return;

    ILI9341_FillScreen(ILI9341_COLOR_BLACK);
    ILI9341_DrawHLine(10, 160, 220, ILI9341_COLOR_GRAY);
    ILI9341_DrawVLine(120, 80, 160, ILI9341_COLOR_GRAY);

    p_int = (int)phase_deg;
    pos = 0;
    buf[pos++] = 'y'; buf[pos++] = '=';
    pos = append_amplitude(buf, pos, amplitude);
    buf[pos++] = 's'; buf[pos++] = 'i'; buf[pos++] = 'n'; buf[pos++] = '(';
    buf[pos++] = 'x'; buf[pos++] = '+';
    buf[pos++] = '0' + (p_int / 100);
    buf[pos++] = '0' + ((p_int / 10) % 10);
    buf[pos++] = '0' + (p_int % 10);
    buf[pos++] = ')'; buf[pos] = 0;
    ILI9341_DrawString(10, 10, buf, ILI9341_COLOR_YELLOW, ILI9341_COLOR_BLACK, 2);
    ILI9341_DrawString(10, 40, "2 periods", ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, 2);
    ILI9341_DrawString(10, 65, "Double 3:Stop", ILI9341_COLOR_GRAY, ILI9341_COLOR_BLACK, 1);

    phase_rad = phase_deg * (float)M_PI / 180.0f;
    /* A=10 cm 对应 LCD 60 px，其他幅值按比例显示，图形与参数 A 一致。 */
    lcd_amp_px = amplitude * 6.0f;
    if (lcd_amp_px > 60.0f) lcd_amp_px = 60.0f;
    lcd_wave_last_x = 20;
    lcd_wave_last_y = 160 - (int16_t)(sinf(phase_rad) * lcd_amp_px);
    for (i = 1; i <= 200U; i++) {
        float t = (float)i / 200.0f;
        float sin_val = sinf(2.0f * (float)M_PI * SINE_PERIODS * t + phase_rad);
        uint16_t x_lcd = (uint16_t)(20U + i);
        uint16_t y_lcd = (uint16_t)(160 - (int16_t)(sin_val * lcd_amp_px));
        ILI9341_DrawLine(lcd_wave_last_x, lcd_wave_last_y,
                         x_lcd, y_lcd, ILI9341_COLOR_CYAN);
        lcd_wave_last_x = x_lcd;
        lcd_wave_last_y = y_lcd;
    }
    lcd_wave_initialized = 1;
}

/* ========================================================================== */
/*                           基本要求 1: 2s 瞄准中心                          */
/* ========================================================================== */

/**
  * @brief  task1 — 启动后 2s 内激光打到中心, 光斑在靶心 2cm 圆圈内
  *
  *         流程: 开激光 → PID 瞄准 → 到位锁定
  *         超时 2s 未到位也锁定 (尽力而为)
  */
void task1(int x, int y)
{
    /* 开激光 */
    if (!laser_on) {
        Laser_Set(1);
        laser_on = 1;
    }

    uint32_t elapsed = HAL_GetTick() - task_start_tick;

    if (is_centered(x, y)) {
        /* 到位: 锁定电机 */
        lock_motors();
    } else if (elapsed < TASK1_TIMEOUT_MS) {
        /* 未到位且未超时: PID 瞄准 */
        aim_pid(x, y);
    } else {
        /* 超时: 锁定当前位置 (尽力而为) */
        lock_motors();
    }
}

/* ========================================================================== */
/*                           基本要求 2: 画 sin(x)                            */
/* ========================================================================== */

/**
  * @brief  task2 — 画 y=sin(x), 峰峰值15cm, 2周期, 长度22cm, 原点在靶心
  *
  *         流程: 瞄准靶心 → 移到起点(-11,0) → 绘制正弦波 → 完成
  */
void task2(int x, int y)
{
    uint32_t now = HAL_GetTick();

    switch (task2_state) {
    case T2_FIND: {
        /* 阶段1: PID 瞄准靶心 */
        if (laser_on) {
            Laser_Set(0);
            laser_on = 0;
        }
        aim_pid(x, y);

        if (uart_data.type == 'A' && uart_data.status == 1U &&
            x >= -TASK2_AIM_THRESHOLD_PIXEL && x <= TASK2_AIM_THRESHOLD_PIXEL &&
            y >= -TASK2_AIM_THRESHOLD_PIXEL && y <= TASK2_AIM_THRESHOLD_PIXEL) {
            float move_distance = SINE_X_SCALE * SINE_LENGTH_CM / 2.0f;
            float move_seconds = move_distance / SINE_MOVE_SPEED_CM_S;
            float move_angle = CM_TO_DEG(-move_distance);

            /* 冻结 PID，关激光，从靶心移动到正弦波左端 (-11cm, 0)。 */
            task_direct_control = 1;
            TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
            TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
            sine_stop_motion();
            TMC2209_Enable(AXIS_YAW);
            TMC2209_Enable(AXIS_PITCH);
            Laser_Set(0);
            laser_on = 0;
            TMC2209_SetVACTUAL(AXIS_PITCH,
                              SINE_X_MOTOR_SIGN * move_angle / move_seconds);
            task2_motion_deadline = now +
                (uint32_t)(move_seconds * 1000.0f + 0.5f);
            task2_state = T2_MOVE_START;
        }
        break;
    }

    case T2_MOVE_START:
        /* 阶段2: 到达左端后打开激光；此后不再依赖 K230 的 Z 状态。 */
        if ((int32_t)(now - task2_motion_deadline) >= 0) {
            sine_stop_motion();
            Laser_Set(1);
            laser_on = 1;
            draw_start_tick = now;
            task2_state = T2_DRAW;
        }
        break;

    case T2_DRAW:
        /* 阶段3: 激光绘制正弦波 (A=7.5cm, ψ=0)。 */
        draw_sine_wave(SINE_AMPLITUDE_CM, 0.0f, 0);
        {
            uint32_t elapsed = now - draw_start_tick;
            if (elapsed >= DRAW_DURATION_MS) {
                sine_stop_motion();
                Laser_Set(0);
                laser_on = 0;
                /* 清掉进入开环前遗留的 PID 目标，防止恢复斜坡后再次转动。 */
                TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
                TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
                task_direct_control = 0;
                task2_state = T2_DONE;
            }
        }
        break;

    case T2_DONE:
        /* 完成: 保持停止 */
        break;
    }
}

/* ========================================================================== */
/*                           基本要求 3: 4s 瞄准中心                          */
/* ========================================================================== */

/**
  * @brief  task3 — 指定位置启动后 4s 内激光打到中心
  *
  *         与 task1 逻辑相同, 超时改为 4s (指定位置更难, 给更多时间)
  */
void task3(int x, int y)
{
    if (!laser_on) {
        Laser_Set(1);
        laser_on = 1;
    }

    uint32_t elapsed = HAL_GetTick() - task_start_tick;

    if (is_centered(x, y)) {
        lock_motors();
    } else if (elapsed < TASK3_TIMEOUT_MS) {
        aim_pid(x, y);
    } else {
        lock_motors();
    }
}

/* ========================================================================== */
/*                           基本要求 4: 画 Asin(x+ψ)                         */
/* ========================================================================== */

/**
  * @brief  task4 — 画 y=Asin(x+ψ), A/ψ 可调, LCD 同步显示波形+表达式
  *
  *         流程: 参数输入(菜单) → 瞄准靶心 → 移到参数波形起点
  *               → 绘制参数化正弦波 → 完成
  *
  * @note   参数 A/ψ 在 eum.c 菜单中调节, 通过 task4_amplitude/task4_phase 传入
  */
void task4(int x, int y)
{
    uint32_t now = HAL_GetTick();

    switch (task4_state) {
    case T4_INPUT:
        /* 等待菜单确认参数 (eum.c 中 key4 确认后切到 T4_FIND) */
        if (laser_on) {
            Laser_Set(0);
            laser_on = 0;
        }
        /* 显示当前参数 */
        {
            char buf[32];
            int p_int = (int)task4_phase;
            uint8_t pos = 0;
            buf[pos++]='A'; buf[pos++]='=';
            pos = append_amplitude(buf, pos, task4_amplitude);
            buf[pos++]=' '; buf[pos++]='P'; buf[pos++]='=';
            buf[pos++]='0'+(p_int/100);
            buf[pos++]='0'+((p_int/10)%10);
            buf[pos++]='0'+(p_int%10); buf[pos]=0;
            ILI9341_DrawString(10, 250, buf, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK, 2);
        }
        break;

    case T4_FIND:
        /* PID 瞄准靶心 */
        if (laser_on) {
            Laser_Set(0);
            laser_on = 0;
        }
        aim_pid(x, y);
        /* K230 误差有量化跳变，放宽到 ±10 像素；超时从当前最佳位置继续。 */
        if ((uart_data.type == 'A' && uart_data.status == 1U &&
             x >= -TASK4_AIM_THRESHOLD_PIXEL && x <= TASK4_AIM_THRESHOLD_PIXEL &&
             y >= -TASK4_AIM_THRESHOLD_PIXEL && y <= TASK4_AIM_THRESHOLD_PIXEL) ||
            (uint32_t)(now - task_start_tick) >= TASK4_AIM_TIMEOUT_MS) {
            float phase_rad = task4_phase * (float)M_PI / 180.0f;
            float start_x = -SINE_X_SCALE * SINE_LENGTH_CM / 2.0f;
            float start_y = SINE_Y_SCALE * task4_amplitude * sinf(phase_rad);
            float move_distance = sqrtf(start_x * start_x + start_y * start_y);
            float move_seconds = move_distance / SINE_MOVE_SPEED_CM_S;
            float x_angle;
            float y_angle;

            if (move_seconds < 0.05f) {
                move_seconds = 0.05f;
            }
            x_angle = CM_TO_DEG(start_x);
            y_angle = CM_TO_DEG(start_y);

            /* 冻结 PID，并从靶心同时移动到 (-L/2, A*sin(psi))。 */
            task_direct_control = 1;
            TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
            TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
            sine_stop_motion();
            TMC2209_Enable(AXIS_YAW);
            TMC2209_Enable(AXIS_PITCH);
            Laser_Set(0);
            laser_on = 0;
            TMC2209_SetVACTUAL(AXIS_PITCH,
                              SINE_X_MOTOR_SIGN * x_angle / move_seconds);
            TMC2209_SetVACTUAL(AXIS_YAW,
                              SINE_Y_MOTOR_SIGN * y_angle / move_seconds);
            task4_motion_deadline = now +
                (uint32_t)(move_seconds * 1000.0f + 0.5f);
            task4_state = T4_MOVE_START;
        }
        break;

    case T4_MOVE_START:
        /* 到达波形左端后再开激光，避免画出靶心到起点的连线。 */
        if ((int32_t)(now - task4_motion_deadline) >= 0) {
            sine_stop_motion();
            lcd_wave_initialized = 0;
            task4_state = T4_DRAW;
            /*
             * 先初始化 LCD，再开始计算 5 s 轨迹时间。
             * 否则整屏清屏/字符绘制的耗时会被算入轨迹，
             * 慢 LCD 上下一轮可能直接判定绘图结束。
             */
            draw_start_tick = HAL_GetTick();
            lcd_draw_waveform(task4_amplitude, task4_phase);
            Laser_Set(1);
            laser_on = 1;
            draw_start_tick = HAL_GetTick();
        }
        break;

    case T4_DRAW:
        /*
         * 与 Task2 完全使用同一套开环轨迹。
         * LCD 已在进入本状态前完整画好，运动期间不再访问 SPI/LCD。
         */
        draw_sine_wave(task4_amplitude, task4_phase, 0);
        {
            uint32_t elapsed = now - draw_start_tick;
            if (elapsed >= DRAW_DURATION_MS) {
                sine_stop_motion();
                Laser_Set(0);
                laser_on = 0;
                TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
                TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
                task_direct_control = 0;
                task4_state = T4_DONE;
            }
        }
        break;

    case T4_DONE:
        break;
    }
}

/* ========================================================================== */
/*                           发挥部分 1: 扫描矩形轮廓                         */
/* ========================================================================== */

/**
  * @brief  task5 — 3s 内激光沿黑色矩形边缘 1-2cm 扫描轮廓
  *
  *         OpenMV 检测黑色矩形, 发送边缘跟踪点 (err_x, err_y) 给 STM32.
  *         STM32 用 PID 跟随边缘点, 同时记录最大/最小坐标计算边长和面积.
  *
  *         status 字段约定:
  *           0 = 正在跟踪边缘
  *           1 = 跟踪完成 (回到起点)
  *
  * @note   边长面积测量: 记录扫描过程中 x/y 的最大最小值, 差值即边长
  */
void task5(int x, int y)
{
    uint32_t elapsed = HAL_GetTick() - task_start_tick;

    if (task5_done) return;

    /* C2=矩形丢失：立即关激光并停住，等待K230重新检测。 */
    if (uart_data.type != 'C' || uart_data.status == 2U) {
        Laser_Set(0);
        laser_on = 0;
        TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
        TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
        return;
    }

    /* C3=先关光移动到左上角；C0=到位后开光沿13个边缘点转一圈。 */
    if (uart_data.status == 3U) {
        Laser_Set(0);
        laser_on = 0;
        aim_pid(x, y);
    } else if (uart_data.status == 0U) {
        Laser_Set(1);
        laser_on = 1;
        aim_pid(x, y);
    }

    /* C1=回到起点完成；20秒仅作安全超时。 */
    if (uart_data.status == 1U || elapsed >= 20000U) {
        task5_done = 1;
        Laser_Set(0);
        laser_on = 0;
        lock_motors();
        ILI9341_DrawString(10, 280, "Rect trace done",
                           ILI9341_COLOR_GREEN, ILI9341_COLOR_BLACK, 2);
    }
}

/* ========================================================================== */
/*                           发挥部分 2: 数字识别 + 书写                      */
/* ========================================================================== */

/* 数字 0-9 使用七段式轨迹。每一段独立抬笔/落笔，避免段间连线。 */
typedef struct {
    int8_t x1, y1, x2, y2;
} DigitSegment_t;

enum {
    DIGIT_SEG_A = 0, /* 顶部横线 */
    DIGIT_SEG_B,     /* 右上竖线 */
    DIGIT_SEG_C,     /* 右下竖线 */
    DIGIT_SEG_D,     /* 底部横线 */
    DIGIT_SEG_E,     /* 左下竖线 */
    DIGIT_SEG_F,     /* 左上竖线 */
    DIGIT_SEG_G,     /* 中间横线 */
    DIGIT_SEG_COUNT
};

static const DigitSegment_t digit_segments[DIGIT_SEG_COUNT] = {
    {-1,  1,  1,  1}, /* A */
    { 1,  1,  1,  0}, /* B */
    { 1,  0,  1, -1}, /* C */
    { 1, -1, -1, -1}, /* D */
    {-1, -1, -1,  0}, /* E */
    {-1,  0, -1,  1}, /* F */
    {-1,  0,  1,  0}, /* G */
};

#define DIGIT_SEG_BIT(n) (1U << (n))
static const uint8_t digit_segment_masks[10] = {
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_B) |
    DIGIT_SEG_BIT(DIGIT_SEG_C) | DIGIT_SEG_BIT(DIGIT_SEG_D) |
    DIGIT_SEG_BIT(DIGIT_SEG_E) | DIGIT_SEG_BIT(DIGIT_SEG_F), /* 0 */
    DIGIT_SEG_BIT(DIGIT_SEG_B) | DIGIT_SEG_BIT(DIGIT_SEG_C), /* 1 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_B) |
    DIGIT_SEG_BIT(DIGIT_SEG_G) | DIGIT_SEG_BIT(DIGIT_SEG_E) |
    DIGIT_SEG_BIT(DIGIT_SEG_D),                              /* 2 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_B) |
    DIGIT_SEG_BIT(DIGIT_SEG_C) | DIGIT_SEG_BIT(DIGIT_SEG_D) |
    DIGIT_SEG_BIT(DIGIT_SEG_G),                              /* 3 */
    DIGIT_SEG_BIT(DIGIT_SEG_F) | DIGIT_SEG_BIT(DIGIT_SEG_G) |
    DIGIT_SEG_BIT(DIGIT_SEG_B) | DIGIT_SEG_BIT(DIGIT_SEG_C), /* 4 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_F) |
    DIGIT_SEG_BIT(DIGIT_SEG_G) | DIGIT_SEG_BIT(DIGIT_SEG_C) |
    DIGIT_SEG_BIT(DIGIT_SEG_D),                              /* 5 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_F) |
    DIGIT_SEG_BIT(DIGIT_SEG_E) | DIGIT_SEG_BIT(DIGIT_SEG_D) |
    DIGIT_SEG_BIT(DIGIT_SEG_C) | DIGIT_SEG_BIT(DIGIT_SEG_G), /* 6 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_B) |
    DIGIT_SEG_BIT(DIGIT_SEG_C),                              /* 7 */
    0x7FU,                                                   /* 8 */
    DIGIT_SEG_BIT(DIGIT_SEG_A) | DIGIT_SEG_BIT(DIGIT_SEG_B) |
    DIGIT_SEG_BIT(DIGIT_SEG_C) | DIGIT_SEG_BIT(DIGIT_SEG_D) |
    DIGIT_SEG_BIT(DIGIT_SEG_F) | DIGIT_SEG_BIT(DIGIT_SEG_G), /* 9 */
};

static uint8_t task6_time_reached(uint32_t now, uint32_t deadline)
{
    return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static void task6_segment_point_cm(int8_t x, int8_t y, float *x_cm, float *y_cm)
{
    *x_cm = TASK6_X_OFFSET_CM + (float)x * TASK6_HALF_WIDTH_CM;
    *y_cm = (float)y * TASK6_HALF_HEIGHT_CM;
}

static void task6_stop_motion(void)
{
    TMC2209_StopVACTUAL(AXIS_YAW);
    TMC2209_StopVACTUAL(AXIS_PITCH);
}

/* 从当前逻辑位置匀速移动到目标点，返回本段所需毫秒数。 */
static uint32_t task6_start_motion(float target_x_cm, float target_y_cm,
                                   float speed_cm_s, uint8_t draw)
{
    float dx = target_x_cm - task6_current_x_cm;
    float dy = target_y_cm - task6_current_y_cm;
    float distance = sqrtf(dx * dx + dy * dy);
    uint32_t duration_ms;
    float duration_s;
    float x_angle_delta;
    float y_angle_delta;

    Laser_Set(draw);
    laser_on = draw;
    task6_target_x_cm = target_x_cm;
    task6_target_y_cm = target_y_cm;

    if (distance < 0.01f) {
        task6_stop_motion();
        return 0;
    }

    duration_ms = (uint32_t)(distance * 1000.0f / speed_cm_s);
    if (duration_ms < TASK6_MIN_MOTION_MS) duration_ms = TASK6_MIN_MOTION_MS;
    duration_s = (float)duration_ms / 1000.0f;

    /* 用起止角之差计算恒定角速度，比小角度速度近似更稳定。 */
    x_angle_delta = CM_TO_DEG(target_x_cm) - CM_TO_DEG(task6_current_x_cm);
    y_angle_delta = CM_TO_DEG(target_y_cm) - CM_TO_DEG(task6_current_y_cm);

    /*
     * 当前相机/云台安装方向与常规Yaw/Pitch坐标旋转了90°：
     * 视觉X误差由Pitch轴修正，视觉Y误差由Yaw轴修正（与aim_pid一致）。
     * 原先按X->Yaw、Y->Pitch绘制会把数字旋转90°，并把右移量变成上移量。
     */
    TMC2209_SetVACTUAL(AXIS_PITCH,
                       TASK6_PITCH_SIGN * x_angle_delta / duration_s);
    TMC2209_SetVACTUAL(AXIS_YAW,
                       TASK6_YAW_SIGN * y_angle_delta / duration_s);
    return duration_ms;
}

static void task6_finish_motion(void)
{
    task6_stop_motion();
    task6_current_x_cm = task6_target_x_cm;
    task6_current_y_cm = task6_target_y_cm;
}

/**
  * @brief  task6 — 识别手写数字, LCD 显示, 激光写出对应数字
  *
 *         当前版本先忽略 K230 通信，直接绘制 task6_digit 指定的数字。
 *         默认 task6_digit=6，支持 0~9；后续识别完成后只需给该变量赋值。
 *
 * @note   先用K230黑框误差重新瞄准靶心，再在靶纸右侧绘制。
 */
void task6(int x, int y)
{
    uint32_t now = HAL_GetTick();
    uint8_t mask;
    const DigitSegment_t *segment;
    float target_x;
    float target_y;
    uint32_t duration;

    (void)x;
    (void)y;
    if (task6_digit > 9U) task6_digit = TASK6_DEFAULT_DIGIT;
    mask = digit_segment_masks[task6_digit];

    if (!task6_lcd_initialized) {
        char buf[16] = "Open-loop D:0";
        buf[12] = '0' + task6_digit;
        ILI9341_FillScreen(ILI9341_COLOR_BLACK);
        ILI9341_DrawString(10, 10, buf, ILI9341_COLOR_GREEN,
                           ILI9341_COLOR_BLACK, 2);
        ILI9341_DrawString(10, 40, "Laser writing...", ILI9341_COLOR_YELLOW,
                           ILI9341_COLOR_BLACK, 1);
        task6_lcd_initialized = 1;
    }

    switch (task6_phase) {
    case T6_IDLE:
        Laser_Set(0);
        laser_on = 0;
        TMC2209_Enable(AXIS_YAW);
        TMC2209_Enable(AXIS_PITCH);
        task6_segment_index = 0;
        task6_current_x_cm = 0.0f; /* 进入任务时以靶心为逻辑原点 */
        task6_current_y_cm = 0.0f;
        task6_phase = T6_AIM;
        break;

    case T6_AIM:
        Laser_Set(0);
        laser_on = 0;
        if (uart_data.status == 0U) {
            TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
            TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
            break;
        }
        aim_pid(x, y);
        if (uart_data.status == 1U && is_centered(x, y)) {
            TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
            TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
            TMC2209_Stop(AXIS_YAW);
            TMC2209_Stop(AXIS_PITCH);
            task_direct_control = 1;
            task6_phase = T6_NEXT_SEGMENT;
        }
        break;

    case T6_NEXT_SEGMENT:
        Laser_Set(0);
        laser_on = 0;
        while (task6_segment_index < DIGIT_SEG_COUNT &&
               (mask & DIGIT_SEG_BIT(task6_segment_index)) == 0U) {
            task6_segment_index++;
        }
        if (task6_segment_index >= DIGIT_SEG_COUNT) {
            task6_phase = T6_DONE;
            break;
        }
        /* 先确保激光真正关闭，再开始向下一段起点移笔。 */
        task6_motion_deadline = now + TASK6_LASER_BLANK_MS;
        task6_phase = T6_BLANK_BEFORE_MOVE;
        break;

    case T6_BLANK_BEFORE_MOVE:
        Laser_Set(0);
        laser_on = 0;
        if (!task6_time_reached(now, task6_motion_deadline)) break;
        segment = &digit_segments[task6_segment_index];
        task6_segment_point_cm(segment->x1, segment->y1, &target_x, &target_y);
        duration = task6_start_motion(target_x, target_y, TASK6_MOVE_SPEED_CM_S, 0);
        task6_motion_deadline = now + duration;
        task6_phase = T6_MOVE_TO_START;
        break;

    case T6_MOVE_TO_START:
        /* 移笔全程重复强制关光，防止留下段间连接线。 */
        Laser_Set(0);
        laser_on = 0;
        if (!task6_time_reached(now, task6_motion_deadline)) break;
        task6_finish_motion();
        task6_motion_deadline = now + TASK6_START_SETTLE_MS;
        task6_phase = T6_SETTLE_AT_START;
        break;

    case T6_SETTLE_AT_START:
        Laser_Set(0);
        laser_on = 0;
        if (!task6_time_reached(now, task6_motion_deadline)) break;
        segment = &digit_segments[task6_segment_index];
        task6_segment_point_cm(segment->x2, segment->y2, &target_x, &target_y);
        duration = task6_start_motion(target_x, target_y, TASK6_DRAW_SPEED_CM_S, 1);
        task6_motion_deadline = now + duration;
        task6_phase = T6_DRAW_SEGMENT;
        break;

    case T6_DRAW_SEGMENT:
        if (!task6_time_reached(now, task6_motion_deadline)) break;
        task6_finish_motion();
        Laser_Set(0);
        laser_on = 0;
        task6_segment_index++;
        task6_phase = T6_NEXT_SEGMENT;
        break;

    case T6_DONE:
        if (!task6_done) {
            task6_stop_motion();
            Laser_Set(0);
            laser_on = 0;
            TMC2209_SetAngleSpeed(AXIS_YAW, 0.0f, 1000000.0f);
            TMC2209_SetAngleSpeed(AXIS_PITCH, 0.0f, 1000000.0f);
            task_direct_control = 0;
            task6_done = 1;
            ILI9341_DrawString(10, 65, "Done", ILI9341_COLOR_CYAN,
                               ILI9341_COLOR_BLACK, 2);
        }
        break;
    }
}

/* ========================================================================== */
/*                           发挥部分 3: 线条复绘                             */
/* ========================================================================== */

/**
  * @brief  task7 — 20s 内激光在屏幕右侧复绘左侧手绘线条图
  *
  *         OpenMV 采集左侧线条的点集, 逐点发送给 STM32 (err_x, err_y).
  *         STM32 跟随点集在右侧绘制.
  *
  *         status 字段约定:
  *           0 = 正在发送点
  *           1 = 所有点发送完毕
  *
  * @note   右侧绘制: x 坐标偏移 +10cm, y 坐标不变
  */
void task7(int x, int y)
{
    if (!laser_on) {
        Laser_Set(1);
        laser_on = 1;
    }

    uint32_t elapsed = HAL_GetTick() - task_start_tick;

    /* K230 发送 (x, y) = 激光斑与当前线条目标点的误差
     * PID 驱动误差到 0, 激光跟随线条轨迹 */
    if (!task7_done && elapsed < 20000) {
        aim_pid(x, y);
    }

    /* 20s 超时或 K230 报告线条完成 (type='L', status=1): 停止 */
    if (!task7_done && (elapsed >= 20000 || (uart_data.type == 'L' && uart_data.status == 1))) {
        task7_done = 1;
        lock_motors();
    }
}
