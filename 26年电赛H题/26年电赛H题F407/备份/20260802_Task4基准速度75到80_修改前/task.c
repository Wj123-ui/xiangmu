#include "task.h"
#include "A4950.h"
#include "xunji.h"
#include "PID.h"
#include "Timer.h"
#include "tmc2209.h"

/*
 * 任务层说明：
 * 1. main.c 根据菜单 Mode 只调用当前选中的任务；
 * 2. TASK1/TASK2/TASK3/TASK4 都是非阻塞控制，真正更新周期由 Timer.c 的 20 ms 节拍直接调用；
 * 3. KEY3 只负责调用 start，KEY4 统一调用 WJ_task_all_reset。
 */

/* 下列函数定义在 main.c，用于修改直流电机速度 PID 的目标值。 */
extern void xunji_set_speed(float base_speed, float correction);
extern void xunji_set_stop(void);
extern void PID_all_init(void);

/*-------------------------- 第一题：钢球回 O 点测试 --------------------------*/
/* TASK1 只测试滚球闭环，小车的 A4950 始终主动刹车。 */
#define TASK1_CONTROL_TIME_MS       20U
/* K230 超过 300 ms 不更新时，停止闭环并让摆杆回到本轮机械零位。 */
#define TASK1_FRAME_TIMEOUT_MS     300U

/*
 * PID 输出就是“相对 TASK1 启动零点的绝对目标角度”，单位 °。
 * P、I 使用通用 PID；D 改为按真实小球速度（像素/秒）计算，避免 K230 帧率
 * 变化时同一个 KD 产生完全不同的制动效果。
 */
/*
 * 首轮防过冲参数：球位于最右端时，先用较小 P 值避免一开始把摆杆打到大角度。
 * 若球后来停在 O 点之前，再每次只加 0.005f。
 */
#define TASK1_KP                     0.035f
/*
 * I 项用于消除机械初始斜度、摩擦等造成的固定像素残差。
 * 本 PID 只在一帧新 K230 图像到达时更新；0.0010f 是当前每帧积分量。
 */
#define TASK1_KI                     0.0008f
/*
 * 积分滞回：|误差| 进入 5 px 时冻结已学到的补偿角；再次漂出 20 px 才恢复积分。
 * 这样既能消除机械静态偏差，又不会在 O 点附近被 K230 的微小噪声长期推偏。
 */
#define TASK1_INTEGRAL_FREEZE_PIXEL   5.0f
#define TASK1_INTEGRAL_RESUME_PIXEL  20.0f
/* 积分本身最多提供 ±5° 补偿，防止长时间异常画面把摆杆推到软限位。 */
#define TASK1_INTEGRAL_LIMIT_DEG      5.0f
/* 新视觉比例下的低速判据，单位 px/s。 */
#define TASK1_ZERO_CAPTURE_SPEED_PX_S 26.0f
#define TASK1_ZERO_CAPTURE_COUNT      5U
/* D 项单位：° / (像素/秒)，用于在小球快速经过 O 点时提前反向倾斜。 */
#define TASK1_KD                      0.055f
/* TASK1 摆杆绝对目标的安全软限位，当前允许范围为 -20°～+20°。 */
#define TASK1_MAX_TARGET_ANGLE_DEG  20.0f
/*
 * 步进电机追踪绝对目标的角速度，单位 °/s。
 * 绝对位置模式目前没有独立的加速度斜坡，120°/s 会让电机从静止直接跳到
 * 约 1066 Hz，实车会抖动或丢步；45°/s 约 400 Hz，先保证能够稳定起步。
 */
#define TASK1_ANGLE_MOVE_SPEED       45.0f
/* 速度的一阶低通：保留 60% 历史值，抑制 K230 单帧像素抖动。 */
#define TASK1_SPEED_FILTER_KEEP       0.60f
/* TASK1 和 TASK3 使用同一机械方向；若回中方向反了，只改这一行。 */
#define TASK1_MOTOR_SIGN             TASK3_MOTOR_SIGN

static PID_Controller task1_position_pid; /* 小球 x 像素到绝对摆杆角度的 PID。 */
/* 以下 K230 缓存由主循环写入、TIM2 的 20 ms 节拍读取。 */
static volatile int16_t task1_ball_x_pixel;
static volatile uint8_t task1_ball_found;
static volatile uint8_t task1_new_frame_ready;
static volatile uint8_t task1_running;
static uint16_t task1_frame_age_ms;
static int16_t task1_previous_ball_x_pixel;
static float task1_ball_speed_pixel_per_second;
static uint8_t task1_previous_frame_valid;
static uint8_t task1_integral_enabled;

static float Task1_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Task1_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

/* 消费一帧新的视觉数据，并由固定的 TIM2 节拍累计“最后一帧的年龄”。 */
static uint8_t Task1_UpdateVision(void)
{
    uint16_t frame_interval_ms;
    float raw_speed;

    if (task1_new_frame_ready != 0U) {
        task1_new_frame_ready = 0U;

        /* 上次有效帧到本帧的实际时间：每次本函数调用相隔固定 20 ms。 */
        frame_interval_ms = task1_frame_age_ms + TASK1_CONTROL_TIME_MS;
        if ((frame_interval_ms > TASK1_FRAME_TIMEOUT_MS) ||
            (task1_previous_frame_valid == 0U)) {
            /* 首帧或长时间丢帧后不计算速度，防止产生虚假的超大 D 输出。 */
            task1_ball_speed_pixel_per_second = 0.0f;
            task1_previous_frame_valid = 1U;
        } else {
            raw_speed = ((float)task1_ball_x_pixel -
                         (float)task1_previous_ball_x_pixel) * 1000.0f /
                        (float)frame_interval_ms;
            task1_ball_speed_pixel_per_second =
                TASK1_SPEED_FILTER_KEEP * task1_ball_speed_pixel_per_second +
                (1.0f - TASK1_SPEED_FILTER_KEEP) * raw_speed;
        }
        task1_previous_ball_x_pixel = task1_ball_x_pixel;
        task1_frame_age_ms = 0U;
        return 1U;
    }

    if (task1_frame_age_ms < TASK1_FRAME_TIMEOUT_MS) {
        task1_frame_age_ms += TASK1_CONTROL_TIME_MS;
        if (task1_frame_age_ms > TASK1_FRAME_TIMEOUT_MS) {
            task1_frame_age_ms = TASK1_FRAME_TIMEOUT_MS;
        }
    }
    if (task1_frame_age_ms >= TASK1_FRAME_TIMEOUT_MS) {
        task1_previous_frame_valid = 0U;
        task1_ball_speed_pixel_per_second = 0.0f;
    }
    return 0U;
}

/* 只在有新 K230 图像时调用一次，避免主循环快慢影响 PID 更新频率。 */
static void Task1_RunPositionPID(void)
{
    float error_pixel;
    float target_angle_deg;

    /* PID 输出不是“这次再转多少”，而是新的绝对目标角度。 */
    error_pixel = task1_position_pid.setpoint - (float)task1_ball_x_pixel;

    /*
     * 启动时的杆角只作为相对 0°；I 项自动学习真实平衡角。
     * 接近 O 点后冻结已有积分值（不清零），让它持续补偿起始角度和摩擦；
     * 漂出更大的 20 px 范围才重新积分。
     */
    if (task1_integral_enabled != 0U) {
        if (Task1_Abs(error_pixel) <= TASK1_INTEGRAL_FREEZE_PIXEL) {
            task1_integral_enabled = 0U;
        }
    } else if (Task1_Abs(error_pixel) >= TASK1_INTEGRAL_RESUME_PIXEL) {
        task1_integral_enabled = 1U;
    }
    PID_SetTunings(&task1_position_pid,
                   TASK1_KP,
                   (task1_integral_enabled != 0U) ? TASK1_KI : 0.0f,
                   0.0f);
    target_angle_deg = PID_Update(&task1_position_pid,
                                   (float)task1_ball_x_pixel);

    /*
     * 速度为负表示小球正向左滚。减去 Kd×速度后，摆杆会及时给出向右的制动倾角；
     * 反之亦然。这里的 D 使用像素/秒，K230 是 20/30/60 FPS 都不会改变其含义。
     */
    target_angle_deg -= TASK1_KD * task1_ball_speed_pixel_per_second;
    target_angle_deg = Task1_Clamp(target_angle_deg,
                                   TASK1_MAX_TARGET_ANGLE_DEG);

    (void)TMC2209_MoveToDegreesAsync(TASK1_MOTOR_SIGN * target_angle_deg,
                                     TASK1_ANGLE_MOVE_SPEED);
}

void WJ_task_1_init(void)
{
    PID_Init(&task1_position_pid,
             TASK1_KP, TASK1_KI, 0.0f,
             -TASK1_MAX_TARGET_ANGLE_DEG,
             TASK1_MAX_TARGET_ANGLE_DEG);
    PID_SetIntegralLimits(&task1_position_pid,
                          -TASK1_INTEGRAL_LIMIT_DEG,
                          TASK1_INTEGRAL_LIMIT_DEG);
    /* 通用 PID 内的 D 关闭；TASK1_KD 在上面按真实像素/秒单独计算。 */
    PID_SetDerivativeFilter(&task1_position_pid, 0.0f);

    task1_ball_x_pixel = 0;
    task1_ball_found = 0U;
    task1_new_frame_ready = 0U;
    task1_running = 0U;
    task1_frame_age_ms = TASK1_FRAME_TIMEOUT_MS;
    task1_previous_ball_x_pixel = 0;
    task1_ball_speed_pixel_per_second = 0.0f;
    task1_previous_frame_valid = 0U;
    task1_integral_enabled = 1U;
}

void WJ_task_1_start(void)
{
    /* TASK1 只做小球平衡测试，启动时立即让小车停车。 */
    xunji_set_stop();
    A4950_Stop();

    PID_Reset(&task1_position_pid);
    PID_SetSetpoint(&task1_position_pid, 0.0f);
    /* 当前摆杆姿态定义为本轮测试的机械 0°，并设置双重 ±40° 软限位。 */
    TMC2209_Enable();
    TMC2209_CancelAngleMove();
    TMC2209_SetCurrentPositionSteps(0);
    TMC2209_SetSoftLimitsDegrees(-TASK1_MAX_TARGET_ANGLE_DEG,
                                  TASK1_MAX_TARGET_ANGLE_DEG);

    /* 必须等 KEY3 之后的新视觉帧，不能把上次任务残留的球坐标拿来控制。 */
    task1_ball_found = 0U;
    task1_new_frame_ready = 0U;
    task1_frame_age_ms = TASK1_FRAME_TIMEOUT_MS;
    task1_previous_ball_x_pixel = 0;
    task1_ball_speed_pixel_per_second = 0.0f;
    task1_previous_frame_valid = 0U;
    task1_integral_enabled = 1U;
    task1_running = 1U;
}

/* main.c 主循环只负责把完整的新 K230 帧交给 TASK1。 */
void WJ_task_1(int16_t ball_x_pixel, uint8_t ball_found, bool new_frame)
{
    if (new_frame) {
        task1_ball_x_pixel = ball_x_pixel;
        task1_ball_found = ball_found;
        /* 数据全部写完后最后置位，TIM2 下个节拍便会读取完整的一帧。 */
        task1_new_frame_ready = 1U;
    }
}

uint8_t WJ_task_1_is_running(void)
{
    return task1_running;
}

/* 只由 Timer.c 的 TIM2 每 20 ms 调用：小车不走，钢球闭环回到 K230 x=0。 */
void WJ_task_1_time_tick_20ms(void)
{
    const TMC2209_State *tmc_state;
    uint8_t fresh_frame;

    if (task1_running == 0U) {
        return;
    }

    /* 即使误选到 TASK1，也始终保持直流电机主动制动。 */
    xunji_set_stop();
    A4950_Stop();

    fresh_frame = Task1_UpdateVision();
    tmc_state = TMC2209_GetState();
    if ((tmc_state != 0) && tmc_state->stalled) {
        /* 堵转后锁定停止，等待 KEY4 总复位并人工检查机械结构。 */
        task1_running = 0U;
        TMC2209_Disable();
        return;
    }

    if ((task1_ball_found == 0U) ||
        (task1_frame_age_ms >= TASK1_FRAME_TIMEOUT_MS)) {
        /* 看不到球时不盲目继续追旧偏差；先安全回到摆杆 0°。 */
        PID_Reset(&task1_position_pid);
        task1_integral_enabled = 1U;
        (void)TMC2209_MoveToDegreesAsync(0.0f,
                                         TASK1_ANGLE_MOVE_SPEED);
        return;
    }

    /* 同一张图最多参与一次 PID 计算；无新帧时电机继续追踪上一次目标。 */
    if (fresh_frame != 0U) {
        Task1_RunPositionPID();
    }
}

/*-------------------------- 第二题参数/状态 --------------------------*/
/* TASK2 每 20 ms 积分一次距离，即 0.02 s。 */
#define TASK2_CONTROL_TIME_S       0.02f
/* 为避免依赖数学库中的 M_PI，在本文件直接使用单精度圆周率。 */
#define TASK2_PI                  3.1415926f

/* 当前 TASK2 状态会被 TIM2 中断读取，因此声明为 volatile。 */
static volatile Task2_State task2_state = TASK2_IDLE;
/* 从本次启动开始累计的平均轮缘距离，由 TIM2 的 20 ms 控制节拍更新。 */
static volatile float task2_distance_mm;
/* TASK2 比赛计时，TIM2 每 1 ms 累加；状态变为 DONE/IDLE 后保持当前值。 */
static volatile uint32_t task2_elapsed_ms;

/* 局部绝对值函数：距离积分只关心轮子转过的路程，不关心编码器正负号。 */
static float Task2_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Task2_IntegrateDistance(void)
{
    /* 左右轮速度来自 Timer.c，单位均为 RPM。 */
    float left_rpm = (float)WJ_get_left_speed();
    float right_rpm = (float)WJ_get_right_speed();
    float average_rpm =
        (Task2_Abs(left_rpm) + Task2_Abs(right_rpm)) * 0.5f;
    float wheel_circumference_mm =
        TASK2_PI * TASK2_WHEEL_DIAMETER_MM;

    /*
     * RPM 是“转/分钟”：
     * 本周期距离 = RPM × 轮周长 × 0.02 s ÷ 60 s。
     * 左右轮绝对速度取平均，可近似表示差速小车中心走过的距离。
     */
    task2_distance_mm += average_rpm * wheel_circumference_mm *
                         TASK2_CONTROL_TIME_S / 60.0f;
}

/*-------------------------- 第三题参数/状态 --------------------------*/
/* TASK3 的固定控制周期。 */
#define TASK3_CONTROL_TIME_MS       20U
/* 超过 300 ms 没收到 K230 新帧就按丢球处理，先停止摆杆。 */
#define TASK3_FRAME_TIMEOUT_MS     300U
/* 计时在 -5 cm 容差窗口内停止，但仍继续追踪精确目标。 */
#define TASK3_NEGATIVE_STOP_MIN_PIXEL (-155.0f)
#define TASK3_NEGATIVE_STOP_MAX_PIXEL (-135.0f)
/* 只有更接近 -140 px 且速度稳定时，才切换到柔和保持 PID。 */
#define TASK3_NEGATIVE_HOLD_MIN_PIXEL (-145.0f)
#define TASK3_NEGATIVE_HOLD_MAX_PIXEL (-135.0f)
/* O 点仍需连续若干帧稳定后才记录平衡零角。 */
#define TASK3_CENTER_CONFIRM_COUNT    3U

/*
 * PID 输出定义为“相对 TASK3 启动零点的绝对目标角度”，单位 °。
 * K230 输入是像素，0.06 表示每 1 px 误差产生约 0.06° 目标倾角。
 * 该值由参考代码的 27.5 微步/cm、1/32 微步和 25.6 px/cm 近似换算而来。
 * 这与 Arduino 的 runToNewPosition(output) 相同，不再把每帧输出重复累加。
 */
/* TASK3 直接参照 TASK1 已实测的滚球参数；以后改 TASK1 时此处会同步跟随。 */
#define TASK3_KP                      0.040f
#define TASK3_KI                      0.00125f
/* 首次回 O 标定时，临时使用很小的 I 项补偿机械初始斜度；记录零点后关闭。 */
#define TASK3_CENTER_CALIBRATION_KI   0.0018f
#define TASK3_CENTER_INTEGRAL_LIMIT_DEG 3.6f  //积分限幅
/*
 * D 项负责给正在快速滚动的小球“提前刹车”，单位与 TASK1 相同：
 * ° / (像素/秒)。它在 PID_Update() 外按真实像素速度单独计算。
 */
#define TASK3_KD                      0.041f  //0.04
/* 仅恢复 TASK3 今日最早版本的有效 ±30° 限位，不改变 TASK1 当前的 ±20°。 */
#define TASK3_MAX_TARGET_ANGLE_DEG   25.0f
/* TASK3 独立摆杆跟随速度，不再引用 TASK1。 */
#define TASK3_ANGLE_MOVE_SPEED       45.0f
/*
 * 题目要求 O→+5 cm→-5 cm 在 5 s 内完成。位置模式没有加速度斜坡，45°/s
 * 反向制动偏慢；仅在两个行程段提高到 90°/s，避免影响 O 点和最终点的稳态。
 */
#define TASK3_TRAVEL_ANGLE_MOVE_SPEED 90.0f
/* +5 cm 折返和 -5 cm 停车均使用短时强制反向倾角，随后交回普通 PID。 */
#define TASK3_RETURN_ANGLE_MOVE_SPEED 105.0f
#define TASK3_POSITIVE_BRAKE_ANGLE_DEG TASK3_MAX_TARGET_ANGLE_DEG
#define TASK3_POSITIVE_BRAKE_TIME_MS  200U
/*
 * -5 cm 是最终停车点，不需要像 +5 cm 那样强制建立反向行程。
 * 使用较轻、较短的制动，且速度降到停稳阈值后立即交回 PID，避免向右反弹过远。
 */
#define TASK3_NEGATIVE_BRAKE_ANGLE_DEG 16.0f
#define TASK3_NEGATIVE_BRAKE_TIME_MS  25U
/* -5 cm 提前制动时保留部分左向积分，避免刹住后失去继续靠近目标的力量。 */
#define TASK3_NEGATIVE_BRAKE_INTEGRAL_RETAIN 0.60f        //过冲量//0.60
/* 负向制动时提前释放倾角，用剩余左向惯性抵消摆杆回正的右向推动。 */
#define TASK3_NEGATIVE_BRAKE_RELEASE_SPEED_PIXEL_PER_SECOND 35.0f
/* 与 TASK1 一样保留 60% 历史速度，抑制单帧 K230 像素抖动。 */
#define TASK3_SPEED_FILTER_KEEP      TASK1_SPEED_FILTER_KEEP

/*
 * 靠近目标时，视觉噪声会造成目标倾角来回变化。
 * |误差| 不超过 10 px 时直接命令摆杆回到 0°，同时保持 EN 和保持力矩。
 */
#define TASK3_DEADBAND_PIXEL          6.5f
/* 最终 -5 cm 目标周围 ±5 px 内暂停积分，离开窗口后自动恢复 KI。 */
#define TASK3_NEGATIVE_KI_DISABLE_PIXEL 5.0f
/*
 * 只有位置已进入死区且相邻 K230 新帧的变化很小时，才认为球真正停稳。
 * 若球高速穿过目标，仍必须保留 D 项输出进行反向刹车，不能直接把摆杆放平。
 */
#define TASK3_SETTLED_SPEED_PIXEL_PER_SECOND 0.0f  //26
/* O 点允许误差放宽到 20 像素；满足后记录实际步进位置为本轮软件 0°。 */
#define TASK3_ZERO_CAPTURE_PIXEL       13.0f
#define TASK3_ZERO_CAPTURE_SPEED_PIXEL_PER_SECOND 26.0f
#define TASK3_ZERO_CAPTURE_COUNT       TASK1_ZERO_CAPTURE_COUNT

static PID_Controller task3_position_pid; /* 小球像素位置闭环 PID。 */
static volatile Task3_State task3_state;  /* 当前 TASK3 状态；由 TIM2 更新。 */
/* 下面三个量由主循环写入、TIM2 读取；16 位和 8 位访问在 F407 上是原子的。 */
static volatile int16_t task3_ball_x_pixel;  /* K230 给出的横向中心偏差。 */
static volatile uint8_t task3_ball_found;    /* 1=当前视觉帧检测到小球。 */
static volatile uint8_t task3_new_frame_ready; /* 有尚未被 TIM2 使用的新视觉帧。 */
static volatile uint16_t task3_elapsed_ms;   /* 从离开 O 点开始累计的时间。 */
static uint8_t task3_timer_stopped;          /* 1=已在题目容差窗口内锁定完成时间。 */
static uint16_t task3_frame_age_ms;          /* 最近一帧 K230 数据的年龄。 */
static uint8_t task3_zero_captured;          /* 1=已记录实际平衡角为软件 0°。 */
static uint8_t task3_zero_capture_count;     /* O 点稳定的新帧计数。 */
static uint8_t task3_zero_capture_pending;   /* 1=已稳定，等待 TIM9 完全停脉冲后置零。 */
static int16_t task3_previous_ball_x_pixel;  /* 上一有效 K230 帧，用于求真实速度。 */
static float task3_ball_speed_pixel_per_second;
static uint8_t task3_previous_frame_valid;
/* 到 +5 cm 折返时的短时强制反向制动剩余时间。 */
static uint16_t task3_positive_brake_remaining_ms;
/* 接近最终 -5 cm 时只触发一次的提前制动，结束后继续闭环保持 -5 cm。 */
static uint16_t task3_negative_brake_remaining_ms;
static uint8_t task3_negative_brake_triggered;

/* TASK3 误差判断使用的局部浮点绝对值函数。 */
static float Task3_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Task3_SetTarget(float target_pixel)
{
    /* 每次切换 +5/-5 cm 都清空积分，避免上一段状态串入下一段。 */
    PID_Reset(&task3_position_pid);
    PID_SetSetpoint(&task3_position_pid, target_pixel);
}

static uint8_t Task3_NegativeTimerStopReached(void)
{
    /* 位置进入指定窗口且速度已稳定时，当前帧立即停止计时。 */
    if (((float)task3_ball_x_pixel >= TASK3_NEGATIVE_STOP_MIN_PIXEL) &&
        ((float)task3_ball_x_pixel <= TASK3_NEGATIVE_STOP_MAX_PIXEL) &&
        (Task3_Abs(task3_ball_speed_pixel_per_second) <=
         TASK3_SETTLED_SPEED_PIXEL_PER_SECOND)) {
        return 1U;
    }
    return 0U;
}

static uint8_t Task3_NegativeHoldReached(void)
{
    /* 计时停止后继续追踪 -140 px，进入更窄窗口才切换保持 PID。 */
    if (((float)task3_ball_x_pixel >= TASK3_NEGATIVE_HOLD_MIN_PIXEL) &&
        ((float)task3_ball_x_pixel <= TASK3_NEGATIVE_HOLD_MAX_PIXEL) &&
        (Task3_Abs(task3_ball_speed_pixel_per_second) <=
         TASK3_SETTLED_SPEED_PIXEL_PER_SECOND)) {
        return 1U;
    }
    return 0U;
}

/*
 * 按当前球速计算提前制动点。+5 cm 用它立即折返，-5 cm 用它提前减速；
 * 写成正负通用形式，K230 左右方向反标定时只需修改 task.h 的目标符号。
 */
static uint8_t Task3_AtApproachBrakePoint(float target_pixel)
{
    float forward_speed;
    float turn_lead_pixel;
    float base_lead_pixel;
    float response_time_s;
    float max_lead_pixel;

    /* 只把“朝 +5 cm 方向的速度”计入预测，反向滚动时不额外提前。 */
    forward_speed = (target_pixel >= 0.0f) ?
                        task3_ball_speed_pixel_per_second :
                        -task3_ball_speed_pixel_per_second;
    if (forward_speed < 0.0f) {
        forward_speed = 0.0f;
    }
    if (target_pixel >= 0.0f) {
        base_lead_pixel = TASK3_POSITIVE_RETURN_LEAD_PIXEL;
        response_time_s = TASK3_POSITIVE_RETURN_RESPONSE_TIME_S;
        max_lead_pixel = TASK3_POSITIVE_RETURN_MAX_LEAD_PIXEL;
    } else {
        base_lead_pixel = TASK3_NEGATIVE_BRAKE_LEAD_PIXEL;
        response_time_s = TASK3_NEGATIVE_BRAKE_RESPONSE_TIME_S;
        max_lead_pixel = TASK3_NEGATIVE_BRAKE_MAX_LEAD_PIXEL;
    }

    turn_lead_pixel = base_lead_pixel + forward_speed * response_time_s;
    if (turn_lead_pixel > max_lead_pixel) {
        turn_lead_pixel = max_lead_pixel;
    }

    if (target_pixel >= 0.0f) {
        return ((float)task3_ball_x_pixel >=
                (target_pixel - turn_lead_pixel)) ? 1U : 0U;
    }

    return ((float)task3_ball_x_pixel <=
            (target_pixel + turn_lead_pixel)) ? 1U : 0U;
}

/* 直接给目标运动方向相反的预设倾角，先压住惯性，再交回普通 PID。 */
static void Task3_ApplyApproachBrake(float target_pixel)
{
    float brake_angle_deg = (target_pixel >= 0.0f) ?
                                -TASK3_POSITIVE_BRAKE_ANGLE_DEG :
                                TASK3_NEGATIVE_BRAKE_ANGLE_DEG;

    (void)TMC2209_MoveToDegreesAsync(TASK3_MOTOR_SIGN * brake_angle_deg,
                                     TASK3_RETURN_ANGLE_MOVE_SPEED);
}

/*
 * 只在首次回 O 阶段确认视觉稳定。确认后先请求停止位置脉冲，下一次 20 ms
 * 节拍确认 TIM9 已完全停止，再真正改软件坐标，避免最后一个脉冲多/少记一步。
 */
static uint8_t Task3_TryCaptureBalanceZero(void)
{
    float error_pixel;

    error_pixel = task3_position_pid.setpoint - (float)task3_ball_x_pixel;
    if ((Task3_Abs(error_pixel) <= TASK3_ZERO_CAPTURE_PIXEL) &&
        (Task3_Abs(task3_ball_speed_pixel_per_second) <=
         TASK3_ZERO_CAPTURE_SPEED_PIXEL_PER_SECOND)) {
        if (task3_zero_capture_count < TASK3_ZERO_CAPTURE_COUNT) {
            task3_zero_capture_count++;
        }
    } else {
        task3_zero_capture_count = 0U;
    }

    if (task3_zero_capture_count < TASK3_ZERO_CAPTURE_COUNT) {
        return 0U;
    }

    task3_zero_capture_pending = 1U;
    task3_zero_capture_count = 0U;
    return 1U;
}

static void Task3_RunPositionPID(void)
{
    float error_pixel;
    float target_angle_deg;
    float move_speed;
    float active_kd;
    float active_ki;
    float active_deadband_pixel;

    error_pixel = task3_position_pid.setpoint - (float)task3_ball_x_pixel;
    active_kd = TASK3_KD;
    active_ki =
        ((task3_state == TASK3_WAIT_CENTER) &&
         (task3_zero_captured == 0U)) ?
            TASK3_CENTER_CALIBRATION_KI : TASK3_KI;
    if (((task3_state == TASK3_TO_NEGATIVE_5CM) ||
         (task3_state == TASK3_HOLD_NEGATIVE_5CM)) &&
        (Task3_Abs(error_pixel) <= TASK3_NEGATIVE_KI_DISABLE_PIXEL)) {
        active_ki = 0.0f;
    }
    active_deadband_pixel = TASK3_DEADBAND_PIXEL;
    /* 到达 -5 cm 后也继续沿用行程 PID，避免切换参数造成目标角突变。 */
    PID_SetTunings(&task3_position_pid,
                   TASK3_KP,
                   active_ki,
                   0.0f);
    PID_SetIntegralLimits(&task3_position_pid,
                          -TASK3_CENTER_INTEGRAL_LIMIT_DEG,
                          TASK3_CENTER_INTEGRAL_LIMIT_DEG);

    /* PID 输出是绝对目标角度，限幅已经由 PID_Init 设为 ±25°。 */
    target_angle_deg = PID_Update(&task3_position_pid,
                                   (float)task3_ball_x_pixel);

    /* 与 TASK1 一致：用真实像素/秒做 D 制动，K230 帧率变化不会改变 KD 含义。 */
    target_angle_deg -= active_kd * task3_ball_speed_pixel_per_second;

    /*
     * 原先只要位置进入 10 px 死区就立刻回 0°，高速穿过目标时没有任何刹车量，
     * 因而会从右端冲过 O 点再滚回右端。现在只有“位置近 + 速度慢”才放平；
     * 其余情况保持 PID（特别是 D 项）主动减速。
     */
    /*
     * 首次标定尚未记录平衡角时，不能强制回临时 0°，否则会抹掉 I 项刚找到的
     * 补偿角；记录完成后才沿用原有的“稳定时放平”处理。
     */
    if ((task3_zero_captured != 0U) &&
        (Task3_Abs(error_pixel) <= active_deadband_pixel) &&
        (Task3_Abs(task3_ball_speed_pixel_per_second) <=
         TASK3_SETTLED_SPEED_PIXEL_PER_SECOND)) {
        target_angle_deg = 0.0f;
    }

    /*
     * 新帧只覆盖目标角度，不会把旧输出继续累加；TIM9 中断按实际 STEP 脉冲
     * 维护当前位置。正负方向仍由 TASK3_MOTOR_SIGN 统一修正。
     */
    move_speed = TASK3_ANGLE_MOVE_SPEED;
    if ((task3_state == TASK3_TO_POSITIVE_5CM) ||
        (task3_state == TASK3_TO_NEGATIVE_5CM) ||
        (task3_state == TASK3_HOLD_NEGATIVE_5CM)) {
        move_speed = TASK3_TRAVEL_ANGLE_MOVE_SPEED;
    }

    (void)TMC2209_MoveToDegreesAsync(TASK3_MOTOR_SIGN * target_angle_deg,
                                     move_speed);
}
//------------------------------------------------------------
void WJ_task_reset(void)
{
    /* 兼容旧代码：旧工程调用该函数时默认启动 TASK2。 */
    WJ_task_2_start();
}

void WJ_task_2_init(void)
{
    /* 上电或 KEY4 复位时，TASK2 回到未启动状态。 */
    task2_state = TASK2_IDLE;
    task2_distance_mm = 0.0f;
    task2_elapsed_ms = 0U;
    WJ_Reset_Speed();
}

void WJ_task_2_start(void)
{
    /* 先清零目标，避免 TIM2 PID 中断在启动瞬间沿用上一轮速度命令。 */
    xunji_set_stop();
    /* 每次 KEY3 启动都重新计圈，并丢弃上一轮编码器滤波残值。 */
    task2_distance_mm = 0.0f;
    task2_elapsed_ms = 0U;
    WJ_Reset_Speed();
    /* 最后才进入 RUNNING，保证下一次 TIM2 中断看到的是全新的 PID 状态。 */
    task2_state = TASK2_RUNNING;
}

void WJ_task_2_stop(void)
{
    /*
     * 停车顺序：
     * 1. 标记 DONE，使主循环不再执行速度 PID；
     * 2. 重置 PID 内部积分/输出；
     * 3. 设置目标为 0；
     * 4. A4950_Stop() 直接进入主动制动。
     */
    task2_state = TASK2_DONE;
    /* 清掉旧 PID 输出，并立即让 A4950 进入 IN1=IN2=1 的主动制动状态。 */
    PID_all_init();
    xunji_set_stop();
    A4950_Stop();
}

void WJ_task_2(void)
{
    float stop_distance_mm;
    float base_speed_rpm;
    float correction;

    /* 本函数只由 TIM2 的固定 20 ms 节拍调用，不能再依赖主循环速度。 */
    if (task2_state != TASK2_RUNNING) {
        xunji_set_stop();
        return;
    }

    /* 先累计本周期距离，再判断是否到达停车阈值。 */
    Task2_IntegrateDistance();
    stop_distance_mm =
        TASK2_LAP_DISTANCE_MM - TASK2_STOP_COMPENSATION_MM;
    if (task2_distance_mm >= stop_distance_mm) {
        WJ_task_2_stop();
        return;
    }

    /* 到 D 点后切换为最后半圆低速，避免高速进弯丢线。 */
    if (task2_distance_mm >= TASK2_FINAL_CURVE_START_MM) {
        base_speed_rpm = TASK2_FINAL_CURVE_SPEED_RPM;
    } else {
        base_speed_rpm = TASK2_FAST_SPEED_RPM;
    }

    /* 按稳定备份：高速和低速段都使用原七路离散循迹修正。 */
    correction = WJ_xunji_fangxing();

    /* 将基础速度和循迹修正交给直流电机 PID。 */
    xunji_set_speed(base_speed_rpm, correction);
}

Task2_State WJ_task_2_get_state(void)
{
    /* 菜单 OLED 用此函数显示 IDLE/RUNNING/DONE。 */
    return task2_state;
}

float WJ_task_2_get_distance_mm(void)
{
    /* 返回累计里程的原始毫米值，OLED 页面显示时再除以 10 换算为厘米。 */
    return task2_distance_mm;
}

void WJ_task_2_time_tick_1ms(void)
{
    /* 只有实际运行阶段计时；停车和 KEY4 复位后都自动停止。 */
    if (task2_state == TASK2_RUNNING) {
        /* 防止极长时间运行时 uint32_t 溢出回到 0。 */
        if (task2_elapsed_ms < 0xFFFFFFFFUL) {
            task2_elapsed_ms++;
        }
    }
}

uint32_t WJ_task_2_get_time_ms(void)
{
    /* Cortex-M4 对齐的 32 位读写是原子的，主循环可安全读取 ISR 的计时值。 */
    return task2_elapsed_ms;
}

void WJ_task_all_reset(void)
{
    /*
     * KEY4 总复位顺序：先清任务和编码器，再清 PID，最后关闭两类驱动。
     * 该函数在主循环执行，不在 TIM2 中断里调用耗时外设操作。
     */
    WJ_task_1_init();
    WJ_task_2_init();
    WJ_task_3_init();
    WJ_task_4_init();
    PID_all_init();
    xunji_set_stop();
    A4950_Stop();
    TMC2209_Disable();
}

void WJ_task_3_init(void)
{
    /* 初始化位置 PID，输出限幅对应摆杆相对启动零点的绝对目标角度。 */
    PID_Init(&task3_position_pid,
             TASK3_KP, TASK3_KI, 0.0f,
             -TASK3_MAX_TARGET_ANGLE_DEG,
             TASK3_MAX_TARGET_ANGLE_DEG);
    /* 首次回 O 标定期间，积分补偿角最多限制为 ±5°。 */
    PID_SetIntegralLimits(&task3_position_pid,
                          -TASK3_CENTER_INTEGRAL_LIMIT_DEG,
                          TASK3_CENTER_INTEGRAL_LIMIT_DEG);
    /* 0.40 比原先 0.70 更快反映小球速度，避免 D 项刹车滞后。 */
    PID_SetDerivativeFilter(&task3_position_pid, 0.40f);

    /* 清空所有视觉缓存、计时器和连续到达计数。 */
    task3_state = TASK3_IDLE;
    task3_ball_x_pixel = 0;
    task3_elapsed_ms = 0U;
    task3_timer_stopped = 0U;
    task3_frame_age_ms = TASK3_FRAME_TIMEOUT_MS;
    task3_ball_found = 0U;
    task3_new_frame_ready = 0U;
    task3_zero_captured = 0U;
    task3_zero_capture_count = 0U;
    task3_zero_capture_pending = 0U;
    task3_previous_ball_x_pixel = 0;
    task3_ball_speed_pixel_per_second = 0.0f;
    task3_previous_frame_valid = 0U;
    task3_positive_brake_remaining_ms = 0U;
    task3_negative_brake_remaining_ms = 0U;
    task3_negative_brake_triggered = 0U;
}

void WJ_task_3_start(void)
{
    /* TASK3 只控制摆杆，因此先保证小车左右轮目标速度为 0。 */
    xunji_set_stop();
    /* 使能步进驱动，先以 O 点为 PID 目标，检测到球后自动回中。 */
    TMC2209_Enable();
    TMC2209_CancelAngleMove();
    /* 当前杆姿先作为临时零点；稳定到 O 后自动记录为实际平衡 0°。 */
    TMC2209_SetCurrentPositionSteps(0);
    TMC2209_SetSoftLimitsDegrees(-TASK3_MAX_TARGET_ANGLE_DEG,
                                  TASK3_MAX_TARGET_ANGLE_DEG);

    task3_state = TASK3_WAIT_CENTER;
    task3_elapsed_ms = 0U;
    task3_timer_stopped = 0U;
    task3_new_frame_ready = 0U;
    task3_zero_captured = 0U;
    task3_zero_capture_count = 0U;
    task3_zero_capture_pending = 0U;
    task3_previous_ball_x_pixel = 0;
    task3_ball_speed_pixel_per_second = 0.0f;
    task3_previous_frame_valid = 0U;
    task3_positive_brake_remaining_ms = 0U;
    task3_negative_brake_remaining_ms = 0U;
    task3_negative_brake_triggered = 0U;
    Task3_SetTarget(0.0f);
}

void WJ_task_3_stop(void)
{
    /* 手动停止时清状态、清 PID，并关闭步进驱动。 */
    task3_state = TASK3_IDLE;
    task3_elapsed_ms = 0U;
    task3_timer_stopped = 0U;
    task3_new_frame_ready = 0U;
    task3_zero_captured = 0U;
    task3_zero_capture_count = 0U;
    task3_zero_capture_pending = 0U;
    task3_previous_ball_x_pixel = 0;
    task3_ball_speed_pixel_per_second = 0.0f;
    task3_previous_frame_valid = 0U;
    task3_positive_brake_remaining_ms = 0U;
    task3_negative_brake_remaining_ms = 0U;
    task3_negative_brake_triggered = 0U;
    PID_Reset(&task3_position_pid);
    TMC2209_Disable();
}

void WJ_task_3(int16_t ball_x_pixel, uint8_t ball_found, bool new_frame)
{
    /*
     * 此函数只在主循环中接收 K230 数据，不再做 PID 计算。
     * 先写完数据、最后置新帧标志，TIM2 中断下一个 20 ms 节拍便会取走完整帧。
     */
    if (new_frame) {
        task3_ball_x_pixel = ball_x_pixel;
        task3_ball_found = ball_found;
        task3_new_frame_ready = 1U;
    }
}

/*
 * TASK3 的固定 20 ms 控制函数，只能由 Timer.c 的 TIM2 中断调用。
 * 这样 OLED/I2C 或主循环中的其他耗时操作，不会改变 PID 的实际控制节拍。
 */
void WJ_task_3_time_tick_20ms(void)
{
    /* +5 cm 和 -5 cm 使用 task.h 中分别标定的 K230 像素目标。 */
    float positive_target = TASK3_POSITIVE_5CM_TARGET_PIXEL;
    float negative_target = TASK3_NEGATIVE_5CM_TARGET_PIXEL;
    const TMC2209_State *tmc_state;
    uint8_t fresh_frame;
    uint8_t forced_brake_active = 0U;
    uint16_t frame_interval_ms;
    float raw_speed;

    /*
     * 每个 K230 图像最多参与一次 PID 计算和一次到达确认。
     * 旧代码会用同一张图连续积分，Ki 即使很小也会把机构推得越来越远。
     */
    fresh_frame = task3_new_frame_ready;
    if (fresh_frame != 0U) {
        task3_new_frame_ready = 0U;
        /* 仅用相邻两张有效图像求速度，含实际丢帧间隔，不假设 K230 固定帧率。 */
        frame_interval_ms = task3_frame_age_ms + TASK3_CONTROL_TIME_MS;
        if ((task3_ball_found == 0U) ||
            (frame_interval_ms > TASK3_FRAME_TIMEOUT_MS) ||
            (task3_previous_frame_valid == 0U)) {
            task3_ball_speed_pixel_per_second = 0.0f;
            task3_previous_frame_valid = task3_ball_found;
        } else {
            raw_speed = ((float)task3_ball_x_pixel -
                         (float)task3_previous_ball_x_pixel) * 1000.0f /
                        (float)frame_interval_ms;
            task3_ball_speed_pixel_per_second =
                TASK3_SPEED_FILTER_KEEP * task3_ball_speed_pixel_per_second +
                (1.0f - TASK3_SPEED_FILTER_KEEP) * raw_speed;
        }
        if (task3_ball_found != 0U) {
            task3_previous_ball_x_pixel = task3_ball_x_pixel;
        }
        task3_frame_age_ms = 0U;
    } else if (task3_frame_age_ms < TASK3_FRAME_TIMEOUT_MS) {
        task3_frame_age_ms += TASK3_CONTROL_TIME_MS;
        if (task3_frame_age_ms > TASK3_FRAME_TIMEOUT_MS) {
            task3_frame_age_ms = TASK3_FRAME_TIMEOUT_MS;
        }
    }
    if (task3_frame_age_ms >= TASK3_FRAME_TIMEOUT_MS) {
        task3_previous_frame_valid = 0U;
        task3_ball_speed_pixel_per_second = 0.0f;
    }

    if (task3_state == TASK3_IDLE) {
        return;
    }

    /* 第三题要求小车静止，状态机运行期间始终保持左右轮目标速度为 0。 */
    xunji_set_stop();

    /* DIAG 检测到堵转时立即关闭驱动并锁定故障状态。 */
    tmc_state = TMC2209_GetState();
    if ((tmc_state != 0) && tmc_state->stalled) {
        task3_state = TASK3_MOTOR_FAULT;
        TMC2209_Disable();
        return;
    }

    /*
     * 第一步：检测到小球后，用位置 PID 自动拉回 O 点。
     * 原先此处固定输出 0，导致“能识别球但不在 O 点”时电机永远不转。
     */
    if (task3_state == TASK3_WAIT_CENTER) {
        /* 已视觉确认稳定：先等正在输出的最后一个 STEP 脉冲结束，再重定义零点。 */
        if (task3_zero_capture_pending != 0U) {
            TMC2209_CancelPositionMove();
            if (TMC2209_IsPositionMoveActive()) {
                return;
            }

            /* 此时的物理杆姿就是平衡角：只重定义坐标，不会让电机额外转动。 */
            TMC2209_SetCurrentPositionSteps(0);
            PID_Reset(&task3_position_pid);
            PID_SetSetpoint(&task3_position_pid, 0.0f);
            task3_zero_captured = 1U;
            task3_zero_capture_pending = 0U;
            task3_state = TASK3_TO_POSITIVE_5CM;
            task3_elapsed_ms = 0U;
            Task3_SetTarget(positive_target);
            return;
        }

        if ((task3_ball_found != 0U) &&
            (task3_frame_age_ms < TASK3_FRAME_TIMEOUT_MS)) {
            /* 没有新图像时不重复发角度命令，等待下一次可靠的视觉测量。 */
            if (fresh_frame != 0U) {
                Task3_RunPositionPID();
                if (Task3_TryCaptureBalanceZero() != 0U) {
                    /* 已连续稳定，下一次 20 ms 节拍完成安全置零并进入 +5 cm。 */
                    TMC2209_CancelPositionMove();
                }
            }
        } else {
            task3_zero_capture_count = 0U;
            task3_zero_capture_pending = 0U;
            TMC2209_CancelPositionMove();
        }
        return;
    }

    /* 堵转属于终止状态；TIMEOUT 枚举仅兼容保留，不再由5秒计时触发。 */
    if ((task3_state == TASK3_TIMEOUT) ||
        (task3_state == TASK3_MOTOR_FAULT)) {
        TMC2209_CancelPositionMove();
        return;
    }

    /* 进入 -5 cm 容差窗口后锁定实际完成时间；不再限制为5秒。 */
    if (task3_timer_stopped == 0U) {
        if (task3_elapsed_ms <=
            (uint16_t)(0xFFFFU - TASK3_CONTROL_TIME_MS)) {
            task3_elapsed_ms += TASK3_CONTROL_TIME_MS;
        } else {
            task3_elapsed_ms = 0xFFFFU;
        }
    }

    /*
     * +5 cm 折返后先持续给强反向倾角；接近最终 -5 cm 时再轻制动一次。
     * 两段制动都不等待 K230 新帧，避免相机帧间隔拖慢刹车响应。
     */
    if ((task3_state == TASK3_TO_NEGATIVE_5CM) &&
        (task3_positive_brake_remaining_ms != 0U)) {
        if ((task3_ball_found != 0U) &&
            (task3_frame_age_ms < TASK3_FRAME_TIMEOUT_MS)) {
            Task3_ApplyApproachBrake(positive_target);
            forced_brake_active = 1U;
        }

        if (task3_positive_brake_remaining_ms <= TASK3_CONTROL_TIME_MS) {
            task3_positive_brake_remaining_ms = 0U;
        } else {
            task3_positive_brake_remaining_ms -= TASK3_CONTROL_TIME_MS;
        }
    } else if ((task3_state == TASK3_TO_NEGATIVE_5CM) &&
               (task3_negative_brake_remaining_ms != 0U)) {
        if ((task3_ball_found != 0U) &&
            (task3_frame_age_ms < TASK3_FRAME_TIMEOUT_MS)) {
            if (task3_ball_speed_pixel_per_second <
                -TASK3_NEGATIVE_BRAKE_RELEASE_SPEED_PIXEL_PER_SECOND) {
                Task3_ApplyApproachBrake(negative_target);
                forced_brake_active = 1U;
            } else {
                /*
                 * 已经减速或开始向右回弹时立刻释放强制制动；
                 * 若本周期有新帧，下面会马上用 -5 cm PID 覆盖旧目标。
                 */
                task3_negative_brake_remaining_ms = 0U;
            }
        }

        if (task3_negative_brake_remaining_ms <= TASK3_CONTROL_TIME_MS) {
            task3_negative_brake_remaining_ms = 0U;
        } else {
            task3_negative_brake_remaining_ms -= TASK3_CONTROL_TIME_MS;
        }
    }

    /* 丢球或串口超过 300 ms 没有新帧时先停杆，计时仍继续。 */
    if ((task3_ball_found == 0U) ||
        (task3_frame_age_ms >= TASK3_FRAME_TIMEOUT_MS)) {
        TMC2209_CancelPositionMove();
    } else if (forced_brake_active != 0U) {
        /* 强制制动已在上面发出，本周期不能再由普通 PID 覆盖。 */
    } else if (fresh_frame == 0U) {
        /* 图像未更新：不重复 PID、不重复累计到达次数，角度命令已自然结束。 */
    } else if (task3_state == TASK3_TO_POSITIVE_5CM) {
        /* 第二步：到 +5 cm 折返点即刻改发 -5 cm PID，不等待多帧确认。 */
        if (Task3_AtApproachBrakePoint(positive_target) != 0U) {
            task3_state = TASK3_TO_NEGATIVE_5CM;
            Task3_SetTarget(negative_target);
            /* 同一帧先给最大反向制动角，后续 200 ms 再继续保持该制动目标。 */
            task3_positive_brake_remaining_ms = TASK3_POSITIVE_BRAKE_TIME_MS;
            Task3_ApplyApproachBrake(positive_target);
        } else {
            Task3_RunPositionPID();
        }
    } else if (task3_state == TASK3_TO_NEGATIVE_5CM) {
        /*
         * 第三步：高速接近 -5 cm 的预测点时只触发一次反向制动。
         * 制动结束后仍以 -5 cm 为 PID 目标，位置和速度连续稳定才进入保持。
         */
        if ((task3_negative_brake_triggered == 0U) &&
            (task3_ball_speed_pixel_per_second <
             -TASK3_SETTLED_SPEED_PIXEL_PER_SECOND) &&
            (Task3_AtApproachBrakePoint(negative_target) != 0U)) {
            task3_negative_brake_triggered = 1U;
            task3_negative_brake_remaining_ms =
                TASK3_NEGATIVE_BRAKE_TIME_MS;
            /*
             * 不再把左向积分完全清零：保留35%供制动结束后继续靠近 -5 cm，
             * 同时丢弃65%以避免恢复全部长行程积分后重新发生左侧过冲。
             */
            task3_position_pid.integral *=
                TASK3_NEGATIVE_BRAKE_INTEGRAL_RETAIN;
            PID_SetSetpoint(&task3_position_pid, negative_target);
            Task3_ApplyApproachBrake(negative_target);
        } else {
            Task3_RunPositionPID();
            if ((task3_timer_stopped == 0U) &&
                (Task3_NegativeTimerStopReached() != 0U)) {
                /* 只锁定完成时间，此时仍用行程 PID 继续靠近 -140 px。 */
                task3_timer_stopped = 1U;
            }
            if (Task3_NegativeHoldReached() != 0U) {
                task3_state = TASK3_HOLD_NEGATIVE_5CM;
                /* 只记录完成状态；PID 参数、积分限幅和运动速度均保持连续。 */
            }
        }
    } else if (task3_state == TASK3_HOLD_NEGATIVE_5CM) {
        /* 第四步：继续使用行程 PID 闭环保持 -5 cm，不再切换另一套参数。 */
        Task3_RunPositionPID();
    }

    /* 未到达 -5 cm 时继续闭环运行，不再因运行超过5秒而停止。 */
}

Task3_State WJ_task_3_get_state(void)
{
    /* OLED 调试页面读取当前状态。 */
    return task3_state;
}

uint16_t WJ_task_3_get_time_ms(void)
{
    /* OLED 调试页面读取 TASK3 用时。 */
    return task3_elapsed_ms;
}
/*-------------------------- 第四题：AB 循迹 + 钢球 O 点平衡 --------------------------*/
/*
 * 参照 AccelStepper 示例，TASK4 的 PID 输出是“摆杆应处的绝对电机角度”，单位 °：
 *     PID(x) -> target_angle
 * 而不是每 20 ms 的速度或角度增量。下面的软件角度变量等效于
 * AccelStepper 的 currentPosition()；位置由 TMC2209 的 TIM9 脉冲中断逐步更新。
 */
/* TASK4 的滚球 PID 直接跟随 TASK1 当前实车参数。 */
#define TASK4_KP                        0.035f
#define TASK4_KI                        0.0012f
/* 与 TASK1 相同，D 的单位是 °/(像素/秒)，在通用 PID 外单独计算。 */
#define TASK4_KD                        0.065f
#define TASK4_CONTROL_TIME_MS           20U
/* TASK4/5/6 分别设置总目标角、积分角和摆杆跟随速度，互不引用。 */
#define TASK4_MAX_TARGET_ANGLE_DEG      20.0f
#define TASK4_INTEGRAL_LIMIT_DEG         5.0f  //积分限幅
#define TASK4_ANGLE_MOVE_SPEED          45.0f  //角速度
#define TASK5_MAX_TARGET_ANGLE_DEG      15.0f  //角度限制
#define TASK5_INTEGRAL_LIMIT_DEG         6.0f  //积分限幅
#define TASK5_ANGLE_MOVE_SPEED          45.0f  
#define TASK6_MAX_TARGET_ANGLE_DEG      20.0f
#define TASK6_INTEGRAL_LIMIT_DEG         6.0f
#define TASK6_ANGLE_MOVE_SPEED          45.0f
/* K230 位置与当前设定值相差不超过 8 px 时按命中目标处理，消除检测抖动。 */
#define TASK4_DEADBAND_PIXEL             8.0f
#define TASK4_FRAME_TIMEOUT_MS         300U
/* 新 K230 图像到达时的低通滤波：保留 70% 历史值、混入 30% 新测量。 */
#define TASK4_X_FILTER_KEEP              0.70f
/* 小球速度 D 项、积分滞回和积分限幅均跟随 TASK1。 */
#define TASK4_SPEED_FILTER_KEEP          TASK1_SPEED_FILTER_KEEP
#define TASK4_INTEGRAL_FREEZE_PIXEL       5.0f
#define TASK4_INTEGRAL_RESUME_PIXEL      20.0f
/* TASK4/5/6 基准直行速度分别独立，避免调速时相互影响。 */
#define TASK4_LINE_SPEED_RPM           75.0f
#define TASK5_LINE_SPEED_RPM           70.0f
#define TASK6_LINE_SPEED_RPM           70.0f
/*
 * TASK4/5/6 都采用柔和起步，减小车体前冲造成的小球惯性偏移。
 */
#define TASK4_LINE_START_SPEED_RPM     10.0f
#define TASK4_LINE_ACCEL_RPM_PER_S     25.0f
/* 从 A 点累计到 1.5 m（B 点）后，按与起步相同的斜率缓慢减速。 */
#define TASK4_DECEL_START_DISTANCE_MM 1500.0f
/* 75→10 RPM 按 25 RPM/s 理论约需 376 mm，随后在该里程主动刹车。 */
#define TASK4_STOP_DISTANCE_MM        1880.0f
#define TASK4_LINE_DECEL_RPM_PER_S     TASK4_LINE_ACCEL_RPM_PER_S
#define TASK4_STOP_SPEED_RPM           10.0f
#define TASK5_LINE_START_SPEED_RPM     10.0f
#define TASK5_LINE_ACCEL_RPM_PER_S     25.0f
/* TASK5 行驶一圈回到 A 后缓慢停车；先按 Task2 实测值初始化，后续可独立标定。 */
#define TASK5_DECEL_START_DISTANCE_MM 5800.0f
#define TASK5_LINE_DECEL_RPM_PER_S     TASK5_LINE_ACCEL_RPM_PER_S
#define TASK5_STOP_SPEED_RPM           10.0f
#define TASK6_LINE_START_SPEED_RPM     10.0f
#define TASK6_LINE_ACCEL_RPM_PER_S     25.0f
/* TASK6 的一圈停车参数独立保存，初值与 TASK5 相同。 */
#define TASK6_DECEL_START_DISTANCE_MM 5800.0f
#define TASK6_LINE_DECEL_RPM_PER_S     TASK6_LINE_ACCEL_RPM_PER_S
#define TASK6_STOP_SPEED_RPM           10.0f
/* TASK4/5/6 启动前先取得连续有效视觉帧，再锁车预平衡一段时间。 */
#define TASK456_STARTUP_VALID_FRAMES     5U
#define TASK456_STARTUP_SETTLE_TIME_MS 300U
/* TASK4 放车瞬间给少量向右预补偿，并在 1.5 s 内线性退出。 */
#define TASK4_START_FEEDFORWARD_DEG       1.5f   //补偿
#define TASK4_START_FEEDFORWARD_TIME_MS 1500U
/* TASK5 独立起步补偿，初值参考 TASK4，后续可单独实车标定。 */
#define TASK5_START_FEEDFORWARD_DEG       1.6f
#define TASK5_START_FEEDFORWARD_TIME_MS 2000U
/* TASK6 只保留自己的目标像素，其起步补偿初值与 TASK5 相同。 */
#define TASK6_START_FEEDFORWARD_DEG       1.6f
#define TASK6_START_FEEDFORWARD_TIME_MS 2000U
/* 提速时相对原 70 RPM 同比增强差速，保持接近原来的转弯半径。 */
#define TASK456_TURN_REFERENCE_SPEED_RPM 70.0f
/* TASK6 现场取点仍限制在摆杆中心两侧各 400 px。 */
#define TASK6_TARGET_MIN_PIXEL         (-400)
#define TASK6_TARGET_MAX_PIXEL           400
/* 共用运行引擎的任务编号；0 表示 TASK4/5/6 均未运行。 */
#define TASK456_MODE_NONE                 0U
#define TASK456_MODE_TASK4                4U
#define TASK456_MODE_TASK5                5U
#define TASK456_MODE_TASK6                6U
/* TASK5 右转状态仍用于 A/B 诊断；实际补偿角由 xunji.c 的探头函数给出。 */
#define TASK5_RIGHT_TURN_ACTIVE_RPM            1.0f
/* A/B 实验诊断：原始右探头退出连续60 ms后，记录随后1 s像素极值。 */
#define TASK5_DIAG_EXIT_CONFIRM_COUNT            3U
#define TASK5_DIAG_EXIT_WINDOW_MS              1000U
#define TASK5_DIAG_PHASE_WAIT_RIGHT               0U
#define TASK5_DIAG_PHASE_IN_TURN                  1U
#define TASK5_DIAG_PHASE_AFTER_EXIT               2U
/*
 * 参考《前馈控制实现方案》：
 * 1. IMU 层已经完成静止零偏和一阶低通；
 * 2. |a| 超过各任务设定阈值时，按“补偿角 = a × K_pitch”叠加到位置 PID；
 * 3. 单独限制前馈角，最后再由 TASK4 总角度软限位保护。
 */
#define TASK4_MG_TO_MPS2                  0.00980665f
#define TASK4_ACCEL_FF_THRESHOLD_MPS2     0.00980665f
#define TASK5_ACCEL_FF_THRESHOLD_MPS2     0.00980665f
#define TASK6_ACCEL_FF_THRESHOLD_MPS2     0.00980665f
#define TASK4_ACCEL_FF_K_DEG_PER_MPS2     2.6f  //前馈系数//4
#define TASK4_ACCEL_FF_MAX_DEG            5.0f  //前馈限幅
#define TASK5_ACCEL_FF_K_DEG_PER_MPS2     2.6f   //前馈系数
#define TASK5_ACCEL_FF_MAX_DEG            5.0f   //前馈限幅
#define TASK6_ACCEL_FF_K_DEG_PER_MPS2     2.6f
#define TASK6_ACCEL_FF_MAX_DEG            5.0f
#define TASK4_ACCEL_TIMEOUT_MS           100U   //超时时间
/*
 * ICM45686 X 轴正值若表示车头方向，保持 +1；实车向前加速时 OLED 显示负值则改为 -1。
 * 管子右侧就是车头，因此“车体向前加速”要叠加正控制角，让小球获得向右/向前的补偿力。
 */
#define TASK4_IMU_FORWARD_SIGN             1.0f
/* TASK4 与 TASK3 共用同一摆杆；若 TASK4 的视觉方向单独相反，只改这一行。 */
#define TASK4_MOTOR_SIGN                TASK3_MOTOR_SIGN

static PID_Controller task4_position_pid; /* TASK4 独立视觉位置 PID。 */
/* 以下视觉和 IMU 缓存由主循环写入、TIM2 读取。 */
static volatile int16_t task4_raw_x_pixel;
static volatile uint8_t task4_ball_found;
static volatile uint8_t task4_new_frame_ready;
static volatile float task4_forward_accel_mg;
static volatile uint8_t task4_new_accel_ready;
static volatile uint8_t task4_running;     /* 1=共用 TASK4/5/6 引擎正在运行。 */
static volatile uint8_t task456_active_mode;
static volatile uint32_t task456_elapsed_ms;
static volatile int16_t task6_target_pixel;
static float task456_line_speed_rpm;
/* 以下量只由 TASK4/5/6 共用的 TIM2 20 ms 控制函数修改。 */
static float task4_filtered_x_pixel;
static float task4_ball_speed_pixel_per_second;
static float task4_feedback_angle_deg;
static float task4_target_angle_deg;
static float task4_active_forward_accel_mg;
static uint16_t task4_frame_age_ms;
static uint16_t task4_accel_age_ms;
static int16_t task4_previous_ball_x_pixel;
static uint8_t task4_filter_initialized;
static uint8_t task4_previous_frame_valid;
static uint8_t task4_integral_enabled;
static uint8_t task456_startup_valid_frames;
static uint16_t task456_startup_settle_ms;
static uint8_t task456_drive_enabled;
static uint16_t task4_drive_elapsed_ms;
static uint16_t task5_drive_elapsed_ms;
static uint16_t task6_drive_elapsed_ms;
static volatile float task4_drive_distance_mm;
static uint8_t task4_drive_decelerating;
static volatile uint8_t task4_drive_stopped;
static volatile float task5_drive_distance_mm;
static uint8_t task5_drive_decelerating;
static volatile uint8_t task5_drive_stopped;
static volatile float task6_drive_distance_mm;
static uint8_t task6_drive_decelerating;
static volatile uint8_t task6_drive_stopped;
static Task5_TurnDiagnostic
    task5_turn_diagnostics[TASK5_DIAGNOSTIC_MAX_TURNS];
static volatile uint8_t task5_diagnostic_count;
static uint8_t task5_diagnostic_phase;
static uint8_t task5_exit_confirm_count;
static uint16_t task5_exit_window_elapsed_ms;
static uint32_t task5_raw_exit_time_ms;
static uint8_t task5_control_exit_recorded;

static float Task4_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* 与 TASK2 相同，按左右轮实测 RPM 的绝对值平均数累计里程。 */
static void Task4_IntegrateDriveDistance(void)
{
    float left_rpm = Task4_Abs((float)WJ_get_left_speed());
    float right_rpm = Task4_Abs((float)WJ_get_right_speed());
    float average_rpm = (left_rpm + right_rpm) * 0.5f;
    float wheel_circumference_mm =
        TASK2_PI * TASK2_WHEEL_DIAMETER_MM;

    task4_drive_distance_mm +=
        average_rpm * wheel_circumference_mm *
        ((float)TASK4_CONTROL_TIME_MS / 60000.0f);
}

/* TASK5 独立累计一圈里程，不与 TASK4 的直线距离共用状态。 */
static void Task5_IntegrateDriveDistance(void)
{
    float left_rpm = Task4_Abs((float)WJ_get_left_speed());
    float right_rpm = Task4_Abs((float)WJ_get_right_speed());
    float average_rpm = (left_rpm + right_rpm) * 0.5f;
    float wheel_circumference_mm =
        TASK2_PI * TASK2_WHEEL_DIAMETER_MM;

    task5_drive_distance_mm +=
        average_rpm * wheel_circumference_mm *
        ((float)TASK4_CONTROL_TIME_MS / 60000.0f);
}

/* TASK6 与 TASK5 使用相同算法，但保留自己的里程状态。 */
static void Task6_IntegrateDriveDistance(void)
{
    float left_rpm = Task4_Abs((float)WJ_get_left_speed());
    float right_rpm = Task4_Abs((float)WJ_get_right_speed());
    float average_rpm = (left_rpm + right_rpm) * 0.5f;
    float wheel_circumference_mm =
        TASK2_PI * TASK2_WHEEL_DIAMETER_MM;

    task6_drive_distance_mm +=
        average_rpm * wheel_circumference_mm *
        ((float)TASK4_CONTROL_TIME_MS / 60000.0f);
}

/* 结束直流电机距离任务，但继续保留步进电机的小球 O 点闭环。 */
static void Task4_StopDrive(void)
{
    task4_drive_stopped = 1U;
    task456_line_speed_rpm = 0.0f;
    PID_all_init();
    xunji_set_stop();
    A4950_Stop();
}

/* TASK5 底盘停车后，滚球 O 点闭环仍由共用控制函数继续执行。 */
static void Task5_StopDrive(void)
{
    task5_drive_stopped = 1U;
    task456_line_speed_rpm = 0.0f;
    PID_all_init();
    xunji_set_stop();
    A4950_Stop();
}

/* TASK6 底盘停车后继续保持用户指定位置的滚球闭环。 */
static void Task6_StopDrive(void)
{
    task6_drive_stopped = 1U;
    task456_line_speed_rpm = 0.0f;
    PID_all_init();
    xunji_set_stop();
    A4950_Stop();
}

static float Task4_Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float Task456_GetMaxTargetAngleDeg(uint8_t active_mode)
{
    if (active_mode == TASK456_MODE_TASK5) {
        return TASK5_MAX_TARGET_ANGLE_DEG;
    }
    if (active_mode == TASK456_MODE_TASK6) {
        return TASK6_MAX_TARGET_ANGLE_DEG;
    }
    return TASK4_MAX_TARGET_ANGLE_DEG;
}

static float Task456_GetIntegralLimitDeg(uint8_t active_mode)
{
    if (active_mode == TASK456_MODE_TASK5) {
        return TASK5_INTEGRAL_LIMIT_DEG;
    }
    if (active_mode == TASK456_MODE_TASK6) {
        return TASK6_INTEGRAL_LIMIT_DEG;
    }
    return TASK4_INTEGRAL_LIMIT_DEG;
}

static float Task456_GetAngleMoveSpeed(uint8_t active_mode)
{
    if (active_mode == TASK456_MODE_TASK5) {
        return TASK5_ANGLE_MOVE_SPEED;
    }
    if (active_mode == TASK456_MODE_TASK6) {
        return TASK6_ANGLE_MOVE_SPEED;
    }
    return TASK4_ANGLE_MOVE_SPEED;
}

static void Task5_ResetDiagnostics(void)
{
    uint8_t index;

    for (index = 0U; index < TASK5_DIAGNOSTIC_MAX_TURNS; index++) {
        task5_turn_diagnostics[index].turn_max_x = 0;
        task5_turn_diagnostics[index].turn_min_x = 0;
        task5_turn_diagnostics[index].exit_max_x = 0;
        task5_turn_diagnostics[index].exit_min_x = 0;
        task5_turn_diagnostics[index].control_exit_delay_ms = 0;
    }
    task5_diagnostic_count = 0U;
    task5_diagnostic_phase = TASK5_DIAG_PHASE_WAIT_RIGHT;
    task5_exit_confirm_count = 0U;
    task5_exit_window_elapsed_ms = 0U;
    task5_raw_exit_time_ms = 0U;
    task5_control_exit_recorded = 0U;
}

static void Task5_BeginDiagnosticTurn(void)
{
    Task5_TurnDiagnostic *diagnostic;

    if (task5_diagnostic_count >= TASK5_DIAGNOSTIC_MAX_TURNS) {
        return;
    }
    diagnostic = &task5_turn_diagnostics[task5_diagnostic_count];
    diagnostic->turn_max_x = (int16_t)-32768;
    diagnostic->turn_min_x = (int16_t)32767;
    diagnostic->exit_max_x = (int16_t)-32768;
    diagnostic->exit_min_x = (int16_t)32767;
    diagnostic->control_exit_delay_ms = 0;
    task5_exit_confirm_count = 0U;
    task5_exit_window_elapsed_ms = 0U;
    task5_control_exit_recorded = 0U;
    task5_diagnostic_phase = TASK5_DIAG_PHASE_IN_TURN;
}

static void Task5_UpdateDiagnosticExtremes(int16_t value,
                                           int16_t *minimum,
                                           int16_t *maximum)
{
    if (value < *minimum) {
        *minimum = value;
    }
    if (value > *maximum) {
        *maximum = value;
    }
}

static void Task5_FinalizeDiagnosticTurn(void)
{
    Task5_TurnDiagnostic *diagnostic;

    diagnostic = &task5_turn_diagnostics[task5_diagnostic_count];
    if (diagnostic->turn_min_x == (int16_t)32767) {
        diagnostic->turn_min_x = 0;
        diagnostic->turn_max_x = 0;
    }
    if (diagnostic->exit_min_x == (int16_t)32767) {
        diagnostic->exit_min_x = 0;
        diagnostic->exit_max_x = 0;
    }
    if (task5_control_exit_recorded == 0U) {
        diagnostic->control_exit_delay_ms =
            (int16_t)TASK5_DIAG_EXIT_WINDOW_MS;
    }
    task5_diagnostic_count++;
    task5_diagnostic_phase = TASK5_DIAG_PHASE_WAIT_RIGHT;
    task5_exit_confirm_count = 0U;
    task5_exit_window_elapsed_ms = 0U;
}

static void Task5_UpdateTurnDiagnostics(uint8_t fresh_frame,
                                        uint8_t control_right_turn_active)
{
    Task5_TurnDiagnostic *diagnostic;
    uint8_t raw_right_active;
    uint8_t raw_any_active;
    uint32_t delay_ms;

    if ((task456_active_mode != TASK456_MODE_TASK5) ||
        (task5_diagnostic_count >= TASK5_DIAGNOSTIC_MAX_TURNS)) {
        return;
    }

    raw_right_active = WJ_xunji_task4_right_raw_active();
    raw_any_active = WJ_xunji_task4_any_raw_active();
    if (task5_diagnostic_phase == TASK5_DIAG_PHASE_WAIT_RIGHT) {
        if (raw_right_active != 0U) {
            Task5_BeginDiagnosticTurn();
        } else {
            return;
        }
    }

    diagnostic = &task5_turn_diagnostics[task5_diagnostic_count];
    if (task5_diagnostic_phase == TASK5_DIAG_PHASE_IN_TURN) {
        if ((fresh_frame != 0U) && (task4_ball_found != 0U)) {
            Task5_UpdateDiagnosticExtremes(task4_raw_x_pixel,
                                           &diagnostic->turn_min_x,
                                           &diagnostic->turn_max_x);
        }

        if ((raw_right_active == 0U) && (raw_any_active != 0U)) {
            if (task5_exit_confirm_count <
                TASK5_DIAG_EXIT_CONFIRM_COUNT) {
                task5_exit_confirm_count++;
            }
        } else {
            task5_exit_confirm_count = 0U;
        }

        if (task5_exit_confirm_count >= TASK5_DIAG_EXIT_CONFIRM_COUNT) {
            task5_diagnostic_phase = TASK5_DIAG_PHASE_AFTER_EXIT;
            task5_exit_window_elapsed_ms = 0U;
            task5_raw_exit_time_ms = task456_elapsed_ms;
            if (control_right_turn_active == 0U) {
                diagnostic->control_exit_delay_ms = 0;
                task5_control_exit_recorded = 1U;
            }
        }
        return;
    }

    if ((fresh_frame != 0U) && (task4_ball_found != 0U)) {
        Task5_UpdateDiagnosticExtremes(task4_raw_x_pixel,
                                       &diagnostic->exit_min_x,
                                       &diagnostic->exit_max_x);
    }
    if ((task5_control_exit_recorded == 0U) &&
        (control_right_turn_active == 0U)) {
        delay_ms = task456_elapsed_ms - task5_raw_exit_time_ms;
        if (delay_ms > 32767UL) {
            delay_ms = 32767UL;
        }
        diagnostic->control_exit_delay_ms = (int16_t)delay_ms;
        task5_control_exit_recorded = 1U;
    }

    if (task5_exit_window_elapsed_ms <=
        (uint16_t)(TASK5_DIAG_EXIT_WINDOW_MS - TASK4_CONTROL_TIME_MS)) {
        task5_exit_window_elapsed_ms += TASK4_CONTROL_TIME_MS;
    } else {
        task5_exit_window_elapsed_ms = TASK5_DIAG_EXIT_WINDOW_MS;
    }
    if (task5_exit_window_elapsed_ms >= TASK5_DIAG_EXIT_WINDOW_MS) {
        Task5_FinalizeDiagnosticTurn();
    }
}

/* 消费主循环锁存的新视觉帧，并在 TIM2 的真实时间轴上计算丢帧超时。 */
static uint8_t Task4_UpdateVision(void)
{
    uint16_t frame_interval_ms;
    float raw_speed;

    if (task4_new_frame_ready != 0U) {
        task4_new_frame_ready = 0U;
        frame_interval_ms = task4_frame_age_ms + TASK4_CONTROL_TIME_MS;
        task4_frame_age_ms = 0U;

        if (task4_ball_found != 0U) {
            if (task4_filter_initialized == 0U) {
                task4_filtered_x_pixel = (float)task4_raw_x_pixel;
                task4_filter_initialized = 1U;
            } else {
                task4_filtered_x_pixel =
                    TASK4_X_FILTER_KEEP * task4_filtered_x_pixel +
                     (1.0f - TASK4_X_FILTER_KEEP) *
                     (float)task4_raw_x_pixel;
            }

            /* 与 TASK1 相同：按两帧真实间隔换算像素/秒，再低通后供 D 项制动。 */
            if ((task4_previous_frame_valid == 0U) ||
                (frame_interval_ms > TASK4_FRAME_TIMEOUT_MS)) {
                task4_ball_speed_pixel_per_second = 0.0f;
                task4_previous_frame_valid = 1U;
            } else {
                raw_speed =
                    ((float)task4_raw_x_pixel -
                     (float)task4_previous_ball_x_pixel) * 1000.0f /
                    (float)frame_interval_ms;
                task4_ball_speed_pixel_per_second =
                    TASK4_SPEED_FILTER_KEEP *
                    task4_ball_speed_pixel_per_second +
                    (1.0f - TASK4_SPEED_FILTER_KEEP) * raw_speed;
            }
            task4_previous_ball_x_pixel = task4_raw_x_pixel;
            return 1U;
        }
        task4_previous_frame_valid = 0U;
        task4_ball_speed_pixel_per_second = 0.0f;
        task4_filter_initialized = 0U;
    } else if (task4_frame_age_ms < TASK4_FRAME_TIMEOUT_MS) {
        task4_frame_age_ms += TASK4_CONTROL_TIME_MS;
        if (task4_frame_age_ms > TASK4_FRAME_TIMEOUT_MS) {
            task4_frame_age_ms = TASK4_FRAME_TIMEOUT_MS;
        }
    }
    if (task4_frame_age_ms >= TASK4_FRAME_TIMEOUT_MS) {
        task4_previous_frame_valid = 0U;
        task4_ball_speed_pixel_per_second = 0.0f;
        task4_filter_initialized = 0U;
    }
    return 0U;
}

/* 每 20 ms 消费最新 IMU 样本；超过 100 ms 没有新样本时禁止沿用旧加速度。 */
static void Task4_UpdateAcceleration(void)
{
    if (task4_new_accel_ready != 0U) {
        task4_new_accel_ready = 0U;
        task4_active_forward_accel_mg = task4_forward_accel_mg;
        task4_accel_age_ms = 0U;
    } else if (task4_accel_age_ms < TASK4_ACCEL_TIMEOUT_MS) {
        task4_accel_age_ms += TASK4_CONTROL_TIME_MS;
        if (task4_accel_age_ms > TASK4_ACCEL_TIMEOUT_MS) {
            task4_accel_age_ms = TASK4_ACCEL_TIMEOUT_MS;
        }
    }

    if (task4_accel_age_ms >= TASK4_ACCEL_TIMEOUT_MS) {
        task4_active_forward_accel_mg = 0.0f;
    }
}

/* 将已经去零偏、低通后的前后加速度换算成 PDF 中定义的俯仰前馈角。 */
static float Task4_GetAccelerationFeedforwardDeg(void)
{
    float forward_accel_mps2;
    float feedforward_deg;
    float threshold_mps2;
    float feedforward_k_deg_per_mps2;
    float feedforward_max_deg;

    if (task4_accel_age_ms >= TASK4_ACCEL_TIMEOUT_MS) {
        return 0.0f;
    }

    forward_accel_mps2 =
        task4_active_forward_accel_mg *
        TASK4_MG_TO_MPS2 *
        TASK4_IMU_FORWARD_SIGN;
    threshold_mps2 = TASK4_ACCEL_FF_THRESHOLD_MPS2;
    feedforward_k_deg_per_mps2 = TASK4_ACCEL_FF_K_DEG_PER_MPS2;
    feedforward_max_deg = TASK4_ACCEL_FF_MAX_DEG;
    if (task456_active_mode == TASK456_MODE_TASK5) {
        /* TASK5 使用自己的阈值、系数和前馈角限幅。 */
        threshold_mps2 = TASK5_ACCEL_FF_THRESHOLD_MPS2;
        feedforward_k_deg_per_mps2 = TASK5_ACCEL_FF_K_DEG_PER_MPS2;
        feedforward_max_deg = TASK5_ACCEL_FF_MAX_DEG;
    } else if (task456_active_mode == TASK456_MODE_TASK6) {
        threshold_mps2 = TASK6_ACCEL_FF_THRESHOLD_MPS2;
        feedforward_k_deg_per_mps2 = TASK6_ACCEL_FF_K_DEG_PER_MPS2;
        feedforward_max_deg = TASK6_ACCEL_FF_MAX_DEG;
    }
    if (Task4_Abs(forward_accel_mps2) <= threshold_mps2) {
        return 0.0f;
    }

    feedforward_deg =
        forward_accel_mps2 * feedforward_k_deg_per_mps2;
    return Task4_Clamp(feedforward_deg, feedforward_max_deg);
}

void WJ_task_4_init(void)
{
    /* KEY4/切换任务时清空共用 PID、视觉、IMU 缓存和软件角度。 */
    PID_Init(&task4_position_pid,
             TASK4_KP, TASK4_KI, 0.0f,
             -TASK4_MAX_TARGET_ANGLE_DEG, TASK4_MAX_TARGET_ANGLE_DEG);
    PID_SetIntegralLimits(&task4_position_pid,
                          -TASK4_INTEGRAL_LIMIT_DEG,
                          TASK4_INTEGRAL_LIMIT_DEG);
    /* 通用 PID 内的 D 关闭；TASK4_KD 在外部按真实像素/秒计算。 */
    PID_SetDerivativeFilter(&task4_position_pid, 0.0f);
    task4_raw_x_pixel = 0;
    task4_ball_found = 0U;
    task4_new_frame_ready = 0U;
    task4_forward_accel_mg = 0.0f;
    task4_new_accel_ready = 0U;
    task4_filtered_x_pixel = 0.0f;
    task4_ball_speed_pixel_per_second = 0.0f;
    task4_feedback_angle_deg = 0.0f;
    task4_target_angle_deg = 0.0f;
    task4_active_forward_accel_mg = 0.0f;
    task4_frame_age_ms = TASK4_FRAME_TIMEOUT_MS;
    task4_accel_age_ms = TASK4_ACCEL_TIMEOUT_MS;
    task4_previous_ball_x_pixel = 0;
    task4_filter_initialized = 0U;
    task4_previous_frame_valid = 0U;
    task4_integral_enabled = 1U;
    task456_startup_valid_frames = 0U;
    task456_startup_settle_ms = 0U;
    task456_drive_enabled = 0U;
    task4_drive_elapsed_ms = 0U;
    task5_drive_elapsed_ms = 0U;
    task6_drive_elapsed_ms = 0U;
    task4_drive_distance_mm = 0.0f;
    task4_drive_decelerating = 0U;
    task4_drive_stopped = 0U;
    task5_drive_distance_mm = 0.0f;
    task5_drive_decelerating = 0U;
    task5_drive_stopped = 0U;
    task6_drive_distance_mm = 0.0f;
    task6_drive_decelerating = 0U;
    task6_drive_stopped = 0U;
    task4_running = 0U;
    task456_active_mode = TASK456_MODE_NONE;
    task456_elapsed_ms = 0U;
    task6_target_pixel = 0;
    task456_line_speed_rpm = 0.0f;
    WJ_xunji_task4_reset();
    Task5_ResetDiagnostics();
}

static void Task456_Start(uint8_t active_mode, float target_pixel)
{
    float max_target_angle_deg;
    float integral_limit_deg;

    /*
     * 启动时钢球已经放在本题指定位置。当前摆杆机械位置定义为软件 0°，
     * 后续 PID 输出均为相对该启动姿态的绝对目标角度。
     */
    xunji_set_stop();
    PID_Reset(&task4_position_pid);
    PID_SetSetpoint(&task4_position_pid, target_pixel);
    max_target_angle_deg = Task456_GetMaxTargetAngleDeg(active_mode);
    integral_limit_deg = Task456_GetIntegralLimitDeg(active_mode);
    PID_SetOutputLimits(&task4_position_pid,
                        -max_target_angle_deg,
                        max_target_angle_deg);
    PID_SetIntegralLimits(&task4_position_pid,
                          -integral_limit_deg,
                          integral_limit_deg);
    TMC2209_Enable();
    TMC2209_CancelAngleMove();
    /* 当前杆姿作为本轮软件 0°；断电/手动拨动后下次启动会重新定义。 */
    TMC2209_SetCurrentPositionSteps(0);
    TMC2209_SetSoftLimitsDegrees(-max_target_angle_deg,
                                  max_target_angle_deg);
    task4_ball_found = 0U;
    task4_new_frame_ready = 0U;
    task4_forward_accel_mg = 0.0f;
    task4_new_accel_ready = 0U;
    task4_filtered_x_pixel = 0.0f;
    task4_ball_speed_pixel_per_second = 0.0f;
    task4_feedback_angle_deg = 0.0f;
    task4_target_angle_deg = 0.0f;
    task4_active_forward_accel_mg = 0.0f;
    task4_frame_age_ms = TASK4_FRAME_TIMEOUT_MS;
    task4_accel_age_ms = TASK4_ACCEL_TIMEOUT_MS;
    task4_previous_ball_x_pixel = 0;
    task4_filter_initialized = 0U;
    task4_previous_frame_valid = 0U;
    task4_integral_enabled = 1U;
    task456_startup_valid_frames = 0U;
    task456_startup_settle_ms = 0U;
    task4_drive_elapsed_ms = 0U;
    task5_drive_elapsed_ms = 0U;
    task6_drive_elapsed_ms = 0U;
    task4_drive_distance_mm = 0.0f;
    task4_drive_decelerating = 0U;
    task4_drive_stopped = 0U;
    task5_drive_distance_mm = 0.0f;
    task5_drive_decelerating = 0U;
    task5_drive_stopped = 0U;
    task6_drive_distance_mm = 0.0f;
    task6_drive_decelerating = 0U;
    task6_drive_stopped = 0U;
    if ((active_mode == TASK456_MODE_TASK4) ||
        (active_mode == TASK456_MODE_TASK5) ||
        (active_mode == TASK456_MODE_TASK6)) {
        /* 丢弃上一轮编码器和速度滤波残值，从 A 点重新累计距离。 */
        WJ_Reset_Speed();
    }
    /* 三个行驶任务都先等待视觉预平衡，再启用底盘。 */
    task456_drive_enabled = 0U;
    task456_elapsed_ms = 0U;
    task456_line_speed_rpm = 0.0f;
    WJ_xunji_task4_reset();
    Task5_ResetDiagnostics();
    task456_active_mode = active_mode;
    task4_running = 1U;
    /* TASK4/5/6 均在视觉预平衡后柔和起步。 */
    xunji_set_stop();
}

void WJ_task_4_start(void)
{
    Task456_Start(TASK456_MODE_TASK4, 0.0f);
}

void WJ_task_5_start(void)
{
    Task456_Start(TASK456_MODE_TASK5, 0.0f);
}

void WJ_task_6_prepare(void)
{
    /* 每次进入设置页都清空旧目标，必须重新双击 KEY2 现场取点。 */
    task6_target_pixel = 0;
}

uint8_t WJ_task_6_capture_target(int16_t ball_x_pixel, uint8_t ball_found)
{
    int16_t captured_target;

    if (ball_found == 0U) {
        return 0U;
    }

    captured_target = ball_x_pixel;
    if (captured_target > TASK6_TARGET_MAX_PIXEL) {
        captured_target = TASK6_TARGET_MAX_PIXEL;
    } else if (captured_target < TASK6_TARGET_MIN_PIXEL) {
        captured_target = TASK6_TARGET_MIN_PIXEL;
    }
    task6_target_pixel = captured_target;
    return 1U;
}

int16_t WJ_task_6_get_target_pixel(void)
{
    return task6_target_pixel;
}

void WJ_task_6_start(void)
{
    Task456_Start(TASK456_MODE_TASK6, (float)task6_target_pixel);
}

/*
 * 主循环入口只缓存传感器数据；真正 PID、循迹和步进脉冲命令都在 TIM2 内执行。
 * K230 每一帧只参与一次滤波，避免主循环反复使用同一帧造成伪积分。
 */
void WJ_task_4(int16_t x, uint8_t ball_found, bool new_frame,
               float forward_accel_mg, bool accel_new_sample)
{
    if (accel_new_sample) {
        task4_forward_accel_mg = forward_accel_mg;
        /* 加速度值写完后最后置位，TIM2 才会读取一组完整的新数据。 */
        task4_new_accel_ready = 1U;
    }
    if (new_frame) {
        task4_raw_x_pixel = x;
        task4_ball_found = ball_found;
        task4_new_frame_ready = 1U;
    }
}

uint8_t WJ_task_4_is_running(void)
{
    if ((task456_active_mode == TASK456_MODE_TASK4) &&
        (task4_drive_stopped != 0U)) {
        /* 让 Timer.c 停止速度 PID，但滚球位置闭环仍由本任务继续执行。 */
        return 0U;
    }
    if ((task456_active_mode == TASK456_MODE_TASK5) &&
        (task5_drive_stopped != 0U)) {
        return 0U;
    }
    if ((task456_active_mode == TASK456_MODE_TASK6) &&
        (task6_drive_stopped != 0U)) {
        return 0U;
    }
    return task4_running;
}

float WJ_task_4_get_distance_mm(void)
{
    return task4_drive_distance_mm;
}

uint8_t WJ_task_5_is_running(void)
{
    return ((task4_running != 0U) &&
            (task456_active_mode == TASK456_MODE_TASK5) &&
            (task5_drive_stopped == 0U)) ? 1U : 0U;
}

uint32_t WJ_task_5_get_time_ms(void)
{
    return task456_elapsed_ms;
}

uint8_t WJ_task_5_get_diagnostic_count(void)
{
    return task5_diagnostic_count;
}

uint8_t WJ_task_5_get_turn_diagnostic(uint8_t index,
                                      Task5_TurnDiagnostic *diagnostic)
{
    if ((diagnostic == 0) || (index >= task5_diagnostic_count) ||
        (index >= TASK5_DIAGNOSTIC_MAX_TURNS)) {
        return 0U;
    }
    *diagnostic = task5_turn_diagnostics[index];
    return 1U;
}

uint8_t WJ_task_6_is_running(void)
{
    return ((task4_running != 0U) &&
            (task456_active_mode == TASK456_MODE_TASK6) &&
            (task6_drive_stopped == 0U)) ? 1U : 0U;
}

uint32_t WJ_task_6_get_time_ms(void)
{
    return task456_elapsed_ms;
}

/* 仅由 TIM2 每 20 ms 调用：一边循迹，一边让钢球稳定在当前设定位置。 */
void WJ_task_4_time_tick_20ms(void)
{
    const TMC2209_State *tmc_state;
    float error_pixel;
    float pid_input_pixel;
    float accel_feedforward_deg;
    float startup_feedforward_deg;
    float turn_feedforward_deg;
    float task56_sensor_turn_compensation_deg;
    float line_correction_rpm;
    float line_speed_scale;
    float line_target_speed_rpm;
    float line_start_speed_rpm;
    float line_accel_rpm_per_s;
    float max_target_angle_deg;
    float angle_move_speed;
    uint8_t fresh_frame;
    uint8_t stepper_fault;
    uint8_t task5_right_turn_active;
    uint8_t task56_turn_compensation_active;
    uint8_t vision_valid;

    if (task4_running == 0U) {
        return;
    }

    max_target_angle_deg =
        Task456_GetMaxTargetAngleDeg(task456_active_mode);
    angle_move_speed =
        Task456_GetAngleMoveSpeed(task456_active_mode);

    /* TASK5/6 回到 A 即锁存整圈时间，后续柔和停车时间不计入圈时。 */
    if (!(((task456_active_mode == TASK456_MODE_TASK5) &&
           ((task5_drive_decelerating != 0U) ||
            (task5_drive_stopped != 0U))) ||
          ((task456_active_mode == TASK456_MODE_TASK6) &&
           ((task6_drive_decelerating != 0U) ||
            (task6_drive_stopped != 0U)))) &&
        (task456_elapsed_ms <=
         (0xFFFFFFFFUL - TASK4_CONTROL_TIME_MS))) {
        task456_elapsed_ms += TASK4_CONTROL_TIME_MS;
    }

    fresh_frame = Task4_UpdateVision();
    Task4_UpdateAcceleration();
    vision_valid = ((task4_ball_found != 0U) &&
                    (task4_frame_age_ms < TASK4_FRAME_TIMEOUT_MS)) ? 1U : 0U;

    tmc_state = TMC2209_GetState();
    stepper_fault = ((tmc_state != 0) && tmc_state->stalled) ? 1U : 0U;
    if (stepper_fault != 0U) {
        /* 摆杆故障只关闭步进电机，不能连带停止七路传感器循迹。 */
        TMC2209_Disable();
    }

    /*
     * TASK4/5/6 先锁住底盘，让视觉位置环在静止条件下工作：连续收到 5 帧有效
     * 小球数据后再预平衡 300 ms。这样不会在视觉尚未建立时先把底盘推出去。
     * 正常起步后，即使短时丢帧仍保持原来的继续循迹策略。
     */
    if (((task456_active_mode == TASK456_MODE_TASK4) ||
         (task456_active_mode == TASK456_MODE_TASK5) ||
         (task456_active_mode == TASK456_MODE_TASK6)) &&
        (task456_drive_enabled == 0U)) {
        task456_line_speed_rpm = 0.0f;
        xunji_set_stop();
        A4950_Stop();

        if (vision_valid == 0U) {
            task456_startup_valid_frames = 0U;
            task456_startup_settle_ms = 0U;
        } else {
            if ((fresh_frame != 0U) &&
                (task456_startup_valid_frames <
                 TASK456_STARTUP_VALID_FRAMES)) {
                task456_startup_valid_frames++;
            }
            if (task456_startup_valid_frames >=
                TASK456_STARTUP_VALID_FRAMES) {
                if (task456_startup_settle_ms <
                    TASK456_STARTUP_SETTLE_TIME_MS) {
                    task456_startup_settle_ms +=
                        TASK4_CONTROL_TIME_MS;
                } else {
                    task456_drive_enabled = 1U;
                }
            }
        }
    }

    line_target_speed_rpm = TASK4_LINE_SPEED_RPM;
    line_start_speed_rpm = TASK4_LINE_START_SPEED_RPM;
    line_accel_rpm_per_s = TASK4_LINE_ACCEL_RPM_PER_S;
    if (task456_active_mode == TASK456_MODE_TASK5) {
        line_target_speed_rpm = TASK5_LINE_SPEED_RPM;
        line_start_speed_rpm = TASK5_LINE_START_SPEED_RPM;
        line_accel_rpm_per_s = TASK5_LINE_ACCEL_RPM_PER_S;
    } else if (task456_active_mode == TASK456_MODE_TASK6) {
        line_target_speed_rpm = TASK6_LINE_SPEED_RPM;
        line_start_speed_rpm = TASK6_LINE_START_SPEED_RPM;
        line_accel_rpm_per_s = TASK6_LINE_ACCEL_RPM_PER_S;
    }

    line_correction_rpm = 0.0f;
    startup_feedforward_deg = 0.0f;
    if ((task456_active_mode == TASK456_MODE_TASK4) &&
        (task456_drive_enabled != 0U) &&
        (task4_drive_stopped == 0U)) {
        Task4_IntegrateDriveDistance();
        if (task4_drive_distance_mm >= TASK4_STOP_DISTANCE_MM) {
            Task4_StopDrive();
        } else if (task4_drive_distance_mm >=
                   TASK4_DECEL_START_DISTANCE_MM) {
            task4_drive_decelerating = 1U;
        }
    } else if ((task456_active_mode == TASK456_MODE_TASK5) &&
               (task456_drive_enabled != 0U) &&
               (task5_drive_stopped == 0U)) {
        Task5_IntegrateDriveDistance();
        if (task5_drive_distance_mm >=
            TASK5_DECEL_START_DISTANCE_MM) {
            task5_drive_decelerating = 1U;
        }
    } else if ((task456_active_mode == TASK456_MODE_TASK6) &&
               (task456_drive_enabled != 0U) &&
               (task6_drive_stopped == 0U)) {
        Task6_IntegrateDriveDistance();
        if (task6_drive_distance_mm >=
            TASK6_DECEL_START_DISTANCE_MM) {
            task6_drive_decelerating = 1U;
        }
    }

    if ((task456_active_mode == TASK456_MODE_TASK4) &&
        (task4_drive_stopped != 0U)) {
        xunji_set_stop();
        A4950_Stop();
    } else if ((task456_active_mode == TASK456_MODE_TASK5) &&
               (task5_drive_stopped != 0U)) {
        xunji_set_stop();
        A4950_Stop();
    } else if ((task456_active_mode == TASK456_MODE_TASK6) &&
               (task6_drive_stopped != 0U)) {
        xunji_set_stop();
        A4950_Stop();
    } else if (task456_drive_enabled != 0U) {
        if ((task456_active_mode == TASK456_MODE_TASK4) &&
            (task4_drive_decelerating != 0U)) {
            task456_line_speed_rpm -=
                TASK4_LINE_DECEL_RPM_PER_S *
                ((float)TASK4_CONTROL_TIME_MS / 1000.0f);
            if (task456_line_speed_rpm <= TASK4_STOP_SPEED_RPM) {
                /* 保持最低爬行速度，最终停车仍由里程阈值统一触发。 */
                task456_line_speed_rpm = TASK4_STOP_SPEED_RPM;
            }
        } else if ((task456_active_mode == TASK456_MODE_TASK5) &&
                   (task5_drive_decelerating != 0U)) {
            task456_line_speed_rpm -=
                TASK5_LINE_DECEL_RPM_PER_S *
                ((float)TASK4_CONTROL_TIME_MS / 1000.0f);
            if (task456_line_speed_rpm <= TASK5_STOP_SPEED_RPM) {
                /* 低于稳定爬行速度后立即主动制动，完成 Task5。 */
                Task5_StopDrive();
            }
        } else if ((task456_active_mode == TASK456_MODE_TASK6) &&
                   (task6_drive_decelerating != 0U)) {
            task456_line_speed_rpm -=
                TASK6_LINE_DECEL_RPM_PER_S *
                ((float)TASK4_CONTROL_TIME_MS / 1000.0f);
            if (task456_line_speed_rpm <= TASK6_STOP_SPEED_RPM) {
                /* 与 TASK5 相同，减到稳定爬行速度后主动制动。 */
                Task6_StopDrive();
            }
        } else if (task456_line_speed_rpm < line_start_speed_rpm) {
            task456_line_speed_rpm = line_start_speed_rpm;
        } else if (task456_line_speed_rpm < line_target_speed_rpm) {
            task456_line_speed_rpm +=
                line_accel_rpm_per_s *
                ((float)TASK4_CONTROL_TIME_MS / 1000.0f);
            if (task456_line_speed_rpm > line_target_speed_rpm) {
                task456_line_speed_rpm = line_target_speed_rpm;
            }
        }
        if ((task4_drive_stopped == 0U) &&
            (task5_drive_stopped == 0U) &&
            (task6_drive_stopped == 0U)) {
            line_speed_scale =
                task456_line_speed_rpm /
                TASK456_TURN_REFERENCE_SPEED_RPM;
            line_correction_rpm =
                WJ_xunji_task4_fangxing() * line_speed_scale;
            xunji_set_speed(task456_line_speed_rpm,
                             line_correction_rpm);
        }

        if ((task456_active_mode == TASK456_MODE_TASK4) &&
            (task4_drive_elapsed_ms <
             TASK4_START_FEEDFORWARD_TIME_MS)) {
            startup_feedforward_deg =
                TASK4_START_FEEDFORWARD_DEG *
                (1.0f -
                 (float)task4_drive_elapsed_ms /
                 (float)TASK4_START_FEEDFORWARD_TIME_MS);
            task4_drive_elapsed_ms += TASK4_CONTROL_TIME_MS;
            if (task4_drive_elapsed_ms >
                TASK4_START_FEEDFORWARD_TIME_MS) {
                task4_drive_elapsed_ms =
                    TASK4_START_FEEDFORWARD_TIME_MS;
            }
        } else if ((task456_active_mode == TASK456_MODE_TASK5) &&
                   (task5_drive_elapsed_ms <
                    TASK5_START_FEEDFORWARD_TIME_MS)) {
            startup_feedforward_deg =
                TASK5_START_FEEDFORWARD_DEG *
                (1.0f -
                 (float)task5_drive_elapsed_ms /
                 (float)TASK5_START_FEEDFORWARD_TIME_MS);
            task5_drive_elapsed_ms += TASK4_CONTROL_TIME_MS;
            if (task5_drive_elapsed_ms >
                TASK5_START_FEEDFORWARD_TIME_MS) {
                task5_drive_elapsed_ms =
                    TASK5_START_FEEDFORWARD_TIME_MS;
            }
        } else if ((task456_active_mode == TASK456_MODE_TASK6) &&
                   (task6_drive_elapsed_ms <
                    TASK6_START_FEEDFORWARD_TIME_MS)) {
            startup_feedforward_deg =
                TASK6_START_FEEDFORWARD_DEG *
                (1.0f -
                 (float)task6_drive_elapsed_ms /
                 (float)TASK6_START_FEEDFORWARD_TIME_MS);
            task6_drive_elapsed_ms += TASK4_CONTROL_TIME_MS;
            if (task6_drive_elapsed_ms >
                TASK6_START_FEEDFORWARD_TIME_MS) {
                task6_drive_elapsed_ms =
                    TASK6_START_FEEDFORWARD_TIME_MS;
            }
        }
    }
    task5_right_turn_active =
        ((task456_active_mode == TASK456_MODE_TASK5) &&
         (line_correction_rpm > TASK5_RIGHT_TURN_ACTIVE_RPM)) ? 1U : 0U;
    task56_sensor_turn_compensation_deg = 0.0f;
    if ((((task456_active_mode == TASK456_MODE_TASK5) &&
          (task5_drive_stopped == 0U)) ||
         ((task456_active_mode == TASK456_MODE_TASK6) &&
          (task6_drive_stopped == 0U))) &&
        (task456_drive_enabled != 0U)) {
        task56_sensor_turn_compensation_deg =
            WJ_xunji_task5_buchangjiao();
    }
    task56_turn_compensation_active =
        (Task4_Abs(task56_sensor_turn_compensation_deg) > 0.01f) ? 1U : 0U;
    /*
     * A/B 诊断只读取视觉和循迹状态，放在故障/丢帧早退之前，
     * 保证即使滚球控制本周期暂停，小车的实际出弯时刻仍能被记录。
     */
    Task5_UpdateTurnDiagnostics(fresh_frame,
                                task5_right_turn_active);

    /* 步进故障时保留小车循迹，只跳过本周期的滚球位置控制。 */
    if (stepper_fault != 0U) {
        return;
    }

    /*
     * 视觉无效时取消尚未完成的摆杆移动，避免沿最后一次目标继续运动；
     * 这里只暂停滚球位置控制，不清零轮速，也不调用 A4950 主动制动。
     */
    if (vision_valid == 0U) {
        TMC2209_CancelPositionMove();
        return;
    }

    /*
     * 位置反馈只在新 K230 帧到达时计算一次；P/I/D 的量纲和积分冻结逻辑均参照 TASK1。
     * 无新视觉帧时保留上次反馈角，但加速度前馈仍会按固定 20 ms 更新。
     */
    if (fresh_frame != 0U) {
        pid_input_pixel = task4_filtered_x_pixel;
        error_pixel = task4_position_pid.setpoint - pid_input_pixel;
        if (Task4_Abs(error_pixel) <= TASK4_DEADBAND_PIXEL) {
            /*
             * TASK4/5 的设定值为 0；TASK6 则把输入吸附到用户选择的非零设定值。
             * 不能再把所有死区都强制写成 0，否则 TASK6 会向 O 点产生固定误差。
             */
            pid_input_pixel = task4_position_pid.setpoint;
        }

        error_pixel = task4_position_pid.setpoint - pid_input_pixel;
        if (task4_integral_enabled != 0U) {
            if (Task4_Abs(error_pixel) <= TASK4_INTEGRAL_FREEZE_PIXEL) {
                task4_integral_enabled = 0U;
            }
        } else if (Task4_Abs(error_pixel) >=
                   TASK4_INTEGRAL_RESUME_PIXEL) {
            task4_integral_enabled = 1U;
        }
        PID_SetTunings(&task4_position_pid,
                       TASK4_KP,
                       ((task4_integral_enabled != 0U) &&
                         (task56_turn_compensation_active == 0U)) ?
                            TASK4_KI : 0.0f,
                       0.0f);
        task4_feedback_angle_deg =
            PID_Update(&task4_position_pid, pid_input_pixel) -
            TASK4_KD * task4_ball_speed_pixel_per_second;
        task4_feedback_angle_deg =
            Task4_Clamp(task4_feedback_angle_deg,
                        max_target_angle_deg);
    }

    /* TASK5/6 直接采用七路探头函数返回的转弯补偿角。 */
    turn_feedforward_deg = task56_sensor_turn_compensation_deg;

    /*
     * TASK5/6 转弯阶段只使用专用补偿，避免 IMU 前馈与其重复叠加；
     * Task4 以及 TASK5/6 直线段仍保持原来的加速度前馈逻辑。
     */
    accel_feedforward_deg = Task4_GetAccelerationFeedforwardDeg();
    if (task56_turn_compensation_active != 0U) {
        accel_feedforward_deg = 0.0f;
    }
    task4_target_angle_deg =
        task4_feedback_angle_deg +
        accel_feedforward_deg +
        startup_feedforward_deg +
        turn_feedforward_deg;
    task4_target_angle_deg =
        Task4_Clamp(task4_target_angle_deg, max_target_angle_deg);

    /*
     * 这里等价于 Arduino 的 stepper.moveTo(-output)：传入的是绝对目标角度，
     * 新图像只覆盖目标而不会叠加旧命令。TIM9 在每一条完整 STEP 脉冲后更新当前位置。
     */
    (void)TMC2209_MoveToDegreesAsync(TASK4_MOTOR_SIGN * task4_target_angle_deg,
                                     angle_move_speed);
}
