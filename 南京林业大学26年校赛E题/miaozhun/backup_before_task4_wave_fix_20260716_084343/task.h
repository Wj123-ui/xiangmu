/*
 * task.h — 竞赛任务接口 (2026南林大电赛 E题: 自动瞄准及绘图装置)
 *
 * 任务总览:
 *   task1 [基本1] 2s内激光打到中心, 光斑在靶心2cm圆圈内
 *   task2 [基本2] 画 y=sin(x), 峰峰值15cm, 2周期, 长度22cm
 *   task3 [基本3] 指定位置4s内打到中心
 *   task4 [基本4] 画 y=Asin(x+ψ), A/ψ可调, LCD同步显示波形+表达式
 *   task5 [发挥1] 3s内扫描黑色矩形轮廓, 测边长面积, LCD显示
 *   task6 [发挥2] 识别手写数字, LCD显示, 激光写出对应数字
 *   task7 [发挥3] 20s内激光复绘手绘线条图
 *
 * 硬件: 二维云台(Yaw+Pitch) + 激光笔, 屏幕50cm外A4感光纸
 * 视觉: K230 via UART4, Z=1精确靶心、Z=2部分框引导、Z=0丢失
 */
#ifndef _task_H_
#define _task_H_
#include "main.h"
#include <math.h>

/* ========== 几何参数 (题目要求) ========== */
#define SCREEN_DISTANCE_CM    50.0f   /* 屏幕距装置距离 (cm) */
#define SINE_LENGTH_CM        22.0f   /* 正弦波总长度 (cm) */
#define SINE_PEAK_PEAK_CM     15.0f   /* 正弦波峰峰值 (cm) */
#define SINE_AMPLITUDE_CM     (SINE_PEAK_PEAK_CM / 2.0f)  /* 幅值 7.5cm */
#define SINE_PERIODS          2       /* 周期数 */
#define DRAW_DURATION_MS      5000    /* 绘图持续时间 (ms) */
#define SINE_MOVE_SPEED_CM_S  12.0f   /* 关激光移动到波形起点的速度 */
#define SINE_X_MOTOR_SIGN     -1.0f   /* 屏幕 x 正方向对应 PITCH；左右镜像时改为 +1 */
#define SINE_Y_MOTOR_SIGN      1.0f   /* 屏幕 y 正方向对应 YAW；上下镜像时改为 -1 */
#define SINE_X_SCALE           0.52f  /* A4 靶适配：实际绘制宽度约 22*0.52=11.44cm */
#define SINE_Y_SCALE           0.55f  /* A4 靶适配：实际峰峰值约 15*0.55=8.25cm */

/* ========== 瞄准参数 ========== */
#define AIM_THRESHOLD_PIXEL   5       /* OpenMV误差 |err|<5 视为到位 */
#define TASK2_AIM_THRESHOLD_PIXEL 10  /* K230 坐标约 3 像素一档，Task2 稍放宽 */
#define TASK1_TIMEOUT_MS      2000    /* 基本要求1: 2秒超时 */
#define TASK3_TIMEOUT_MS      4000    /* 基本要求3: 4秒超时 */

/* ========== task6 开环数字书写参数 ========== */
#define TASK6_DEFAULT_DIGIT       6U      /* 暂不接 K230 时默认绘制的数字 */
#define TASK6_X_OFFSET_CM         6.5f    /* 数字中心相对靶心向右偏移 */
#define TASK6_HALF_WIDTH_CM       3.0f    /* 数字半宽: 总宽 6cm */
#define TASK6_HALF_HEIGHT_CM      5.0f    /* 数字半高: 总高 10cm */
#define TASK6_MOVE_SPEED_CM_S    15.0f    /* 关激光移笔速度 */
#define TASK6_DRAW_SPEED_CM_S     6.0f    /* 开激光书写速度 */
#define TASK6_MIN_MOTION_MS      80U
#define TASK6_YAW_SIGN            1.0f    /* 方向不对时改为 -1.0f */
#define TASK6_PITCH_SIGN          1.0f    /* 上下颠倒时改为 -1.0f */

/* ========== cm → 角度换算 ========== */
/* 屏幕在50cm外, 屏幕上d cm对应角度 atan(d/50) rad */
#define CM_TO_DEG(cm)  (atanf((cm) / SCREEN_DISTANCE_CM) * 57.29578f)

/* ========== 任务状态机 ========== */
typedef enum {
    TASK_IDLE = 0,       /* 空闲 */
    TASK_AIMING,         /* PID瞄准中 */
    TASK_LOCKED,         /* 到位锁定 */
    TASK_DRAWING,        /* 绘图中 */
    TASK_DONE,           /* 完成 */
} TaskState_t;

/* task2 状态 */
typedef enum {
    T2_FIND,             /* 瞄准靶心 */
    T2_MOVE_START,       /* 关激光移动到波形左端 */
    T2_DRAW,             /* 绘制正弦波 */
    T2_DONE,             /* 完成 */
} Task2State_t;

/* task4 状态 */
typedef enum {
    T4_INPUT,            /* 参数输入 (等待菜单确认) */
    T4_FIND,             /* 瞄准靶心 */
    T4_DRAW,             /* 绘制参数化正弦波 */
    T4_DONE,             /* 完成 */
} Task4State_t;

/* ========== task4 可调参数 ========== */
/* 幅值A选项: 3cm, 5cm, 7.5cm, 10cm (题目要求精度±1cm) */
extern float task4_amplitude;
extern uint8_t task4_amp_idx;
/* 相位ψ选项: 0°, 30°, 60°, 90°, 120°, 150°, 180° */
extern float task4_phase;
extern uint8_t task4_pha_idx;
/* task6 当前要绘制的数字 (0~9), 后续可由 K230 识别结果直接赋值 */
extern uint8_t task6_digit;

/* ========== 任务函数 (由主循环调用, 参数为OpenMV误差) ========== */
void task1(int x, int y);  /* 基本1: 2s瞄准中心 */
void task2(int x, int y);  /* 基本2: 画sin(x) */
void task3(int x, int y);  /* 基本3: 4s瞄准中心 */
void task4(int x, int y);  /* 基本4: 画Asin(x+ψ) */
void task5(int x, int y);  /* 发挥1: 扫描矩形轮廓 */
void task6(int x, int y);  /* 发挥2: 数字识别+书写 */
void task7(int x, int y);  /* 发挥3: 线条复绘 */

/* ========== 任务管理 ========== */
void Task_Reset(void);                        /* 重置所有任务状态 */
void Task_Run(uint8_t task_id, int x, int y); /* 调度: 按 task_id 执行对应任务 */
uint8_t Task_IsRunning(void);                  /* 查询是否有任务运行中 */

/* ========== 全局状态 ========== */
extern volatile uint8_t task_direct_control;  /* 1=任务直接控制电机, 斜坡跳过 */
extern Task2State_t task2_state;
extern Task4State_t task4_state;

#endif
