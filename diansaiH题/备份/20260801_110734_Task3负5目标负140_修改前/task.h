#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * K230 的 dx 是以画面中心为 0 的像素偏差。
 * TASK3_PIXELS_PER_CM 只用于把“允许误差 1 cm”换算成像素。
 */
#define TASK3_PIXELS_PER_CM 26.0f

/*
 * TASK3 的两个目标位置，单位都是 K230 回传的 dx 像素。
 * 实车标定时只需要修改下面两个数；左右成像不对称时可以分别设置。
 */
#define TASK3_POSITIVE_5CM_TARGET_PIXEL   130.0f
#define TASK3_NEGATIVE_5CM_TARGET_PIXEL  (-130.0f)
/*
 * 小球到 +5 cm 后的基础提前折返点；实际折返点还会按当前球速自动提前，
 * 用于抵消相机帧间隔、步进反向和小球惯性。
 */
#define TASK3_POSITIVE_RETURN_LEAD_PIXEL   7.0f
/* 按球速预测步进反向延迟的时间，单位 s；只影响 +5 cm 的折返预测。 */
#define TASK3_POSITIVE_RETURN_RESPONSE_TIME_S 0.18f
/* 动态提前量上限，避免高速噪声导致刚离开 O 点就折返。 */
#define TASK3_POSITIVE_RETURN_MAX_LEAD_PIXEL 72.0f
/* -5 cm 停车制动保留原先的提前量，不随 +5 cm 折返点调整。 */
#define TASK3_NEGATIVE_BRAKE_LEAD_PIXEL       33.0f
#define TASK3_NEGATIVE_BRAKE_RESPONSE_TIME_S   0.18f
#define TASK3_NEGATIVE_BRAKE_MAX_LEAD_PIXEL   98.0f

/* 若电机转动后小球向目标的反方向运动，把 1.0f 改为 -1.0f。 */
#define TASK3_MOTOR_SIGN     -1.0f

/* 第二题赛道的理论形状：两段 1.5 m 直线和半径 0.5 m 的两个半圆。 */
/* 该值是当前编码器里程计的实车停车阈值，单位 mm，可按实测结果标定。 */
#define TASK2_LAP_DISTANCE_MM  5710.0f
/* 轮子的有效滚动直径，不能只量外壳直径；RPM 积分距离时使用该参数。 */
#define TASK2_WHEEL_DIAMETER_MM 65.0f
/*
 * 顺时针 A->B->C->D->A：在 D 点前约 145 mm 开始降速。
 * 此参数必须独立于停车距离，否则修改 LAP_DISTANCE 会把降速点一起移动。
 */
#define TASK2_FINAL_CURVE_START_MM 4000.0f
/* D 点之前的快速循迹速度，单位 RPM。 */
#define TASK2_FAST_SPEED_RPM        160.0f
/* D->A 最后半圆的低速循迹速度，单位 RPM。 */
#define TASK2_FINAL_CURVE_SPEED_RPM  90.0f
/* 重复性稳定后再按平均误差设置；正数表示提前多少 mm 触发刹车。 */
#define TASK2_STOP_COMPENSATION_MM 0.0f

/* TASK2 的生命周期状态。 */
typedef enum {
    TASK2_IDLE = 0, /* 尚未启动或已复位。 */
    TASK2_RUNNING,  /* 正在循迹并累计距离。 */
    TASK2_DONE      /* 达到停车阈值并已刹车。 */
} Task2_State;

/* TASK3 滚球控制状态机的各个状态。 */
typedef enum {
    TASK3_IDLE = 0,             /* 未运行。 */
    TASK3_WAIT_CENTER,          /* 等待小球回到 O 点。 */
    TASK3_TO_POSITIVE_5CM,      /* 向 +5 cm 运动。 */
    TASK3_TO_NEGATIVE_5CM,      /* 向 -5 cm 运动。 */
    TASK3_HOLD_NEGATIVE_5CM,    /* 到达 -5 cm 后继续闭环保持。 */
    TASK3_TIMEOUT,              /* 5 s 内未完成。 */
    TASK3_MOTOR_FAULT           /* TMC2209 报告堵转。 */
} Task3_State;

/*
 * TASK1：钢球回 O 点测试。
 * main.c 仅缓存 K230 新帧；TIM2 每 20 ms 做一次位置 PID，小车全程保持刹车。
 */
void WJ_task_1_init(void);
void WJ_task_1_start(void);
void WJ_task_1(int16_t ball_x_pixel, uint8_t ball_found, bool new_frame);
void WJ_task_1_time_tick_20ms(void);
uint8_t WJ_task_1_is_running(void);

/*
 * TASK4/5/6 共用“循迹 + 钢球位置闭环”引擎：
 * TASK4、TASK5 的目标为 O 点，TASK6 的目标由二级菜单设置。
 */
void WJ_task_4_init(void);
void WJ_task_4_start(void);
void WJ_task_4(int16_t x, uint8_t ball_found, bool new_frame,
               float forward_accel_mg, bool accel_new_sample);
void WJ_task_4_time_tick_20ms(void);
/* Timer.c 判断当前 TASK4/5/6 是否允许直流电机速度 PID 继续驱动循迹。 */
uint8_t WJ_task_4_is_running(void);
/* TASK5：钢球保持 O 点，沿线行驶一圈并通过 A 点。 */
void WJ_task_5_start(void);
uint8_t WJ_task_5_is_running(void);
uint32_t WJ_task_5_get_time_ms(void);
/* TASK6：进入菜单时先设置目标像素，再启动同一套循迹/滚球闭环。 */
void WJ_task_6_prepare(void);
void WJ_task_6_target_increase(void);
void WJ_task_6_target_decrease(void);
int16_t WJ_task_6_get_target_pixel(void);
void WJ_task_6_start(void);
uint8_t WJ_task_6_is_running(void);
uint32_t WJ_task_6_get_time_ms(void);

/* 兼容旧工程的 TASK2 启动入口。 */
void WJ_task_reset(void);
/* 将 TASK2 状态和累计距离清零，同时清空编码器速度滤波。 */
void WJ_task_2_init(void);
/*
 * 第二题的 20 ms 控制更新入口，由 Timer.c 的 TIM2 中断调用。
 * 放在定时器节拍中可避免 OLED 等主循环阻塞延迟距离积分或停车判断。
 */
void WJ_task_2(void);
/* 清零距离并进入 TASK2_RUNNING。 */
void WJ_task_2_start(void);
/* 进入 TASK2_DONE，清 PID 并对 A4950 立即主动刹车。 */
void WJ_task_2_stop(void);
/* 读取 TASK2 当前生命周期状态。 */
Task2_State WJ_task_2_get_state(void);
/* 读取 TASK2 当前累计里程，单位 mm。 */
float WJ_task_2_get_distance_mm(void);
/*
 * TIM2 每 1 ms 调用一次：仅在 TASK2_RUNNING 状态累计比赛计时。
 * 到达停车点后状态改为 TASK2_DONE，计时会自然冻结。
 */
void WJ_task_2_time_tick_1ms(void);
/* 读取本次 TASK2 已计时的毫秒数；KEY4 总复位后返回 0。 */
uint32_t WJ_task_2_get_time_ms(void);

/*
 * 第三题：菜单选中 TASK3 后按 key3 启动，等待小球位于 O 点后，依次运行到 +5 cm、-5 cm，
 * 最后闭环稳定在 -5 cm。
 * main.c 只负责接收 K230 新帧；真正的固定 20 ms PID 由 TIM2 调用。
 */
/* 清零 TASK3 状态并初始化滚球位置 PID。 */
void WJ_task_3_init(void);
/* 使能 TMC2209，并从 O 点等待状态开始。 */
void WJ_task_3_start(void);
/* 停止步进输出并回到 TASK3_IDLE。 */
void WJ_task_3_stop(void);
/*
 * 在 main.c 的 Mode==3 分支中反复调用：
 * ball_x_pixel：小球相对画面中心的横向偏差，单位像素；
 * ball_found：1 表示本帧识别到小球，0 表示未识别；
 * new_frame：true 表示前三个参数来自尚未处理的新帧。
 */
void WJ_task_3(int16_t ball_x_pixel, uint8_t ball_found, bool new_frame);
/*
 * 仅由 Timer.c 的 TIM2 20 ms 节拍调用：执行一次 TASK3 状态机和 PID。
 * 不要从 main.c 直接调用，否则会破坏固定控制周期。
 */
void WJ_task_3_time_tick_20ms(void);
/* 读取 TASK3 当前状态机状态。 */
Task3_State WJ_task_3_get_state(void);
/* 读取 TASK3 已运行时间，单位 ms。 */
uint16_t WJ_task_3_get_time_ms(void);

/* KEY4 菜单复位使用：停止两类电机并恢复任务/PID 的初始状态。 */
void WJ_task_all_reset(void);

#endif
