#ifndef TMC2209_H
#define TMC2209_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * TMC2209 步进电机驱动接口。
 * UART 用于配置驱动器寄存器，TIM9_CH1 输出 STEP 脉冲，GPIO 控制 DIR/EN，
 * DIAG 外部中断用于锁存堵转故障。
 */

/* TMC2209 MRES 编码：0=1/256 微步，数值每加 1 微步数减半，8=整步。 */
#define TMC2209_MRES_256  0U
#define TMC2209_MRES_128  1U
#define TMC2209_MRES_64   2U
#define TMC2209_MRES_32   3U
#define TMC2209_MRES_16   4U
#define TMC2209_MRES_8    5U
#define TMC2209_MRES_4    6U
#define TMC2209_MRES_2    7U
#define TMC2209_MRES_FULL 8U

/* 当前机构参数：1.8° 电机每圈 200 整步，电机轴与输出轴为 1:1。 */
#define TMC2209_FULL_STEPS_PER_REV 200U
#define TMC2209_REDUCTION_RATIO    1U

/* 供菜单/任务查询的驱动器运行状态和诊断信息。 */
typedef struct {
    bool enabled;                    /* EN 是否已使能，true 表示允许驱动电机。 */
    bool stalled;                    /* DIAG 是否触发并锁存堵转故障。 */
    uint8_t microstep_resolution;    /* 当前 MRES 编码，使用上方宏定义。 */
    uint32_t step_frequency_hz;      /* 当前 STEP 输出频率，单位 Hz。 */
    uint32_t communication_errors;   /* UART 发送、接收或校验失败累计次数。 */
} TMC2209_State;

/* 初始化 UART、STEP 定时器及 TMC2209 默认寄存器，成功通信时返回 true。 */
bool TMC2209_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim,
                  uint8_t default_mres);

/* 拉低 EN 使能驱动；堵转故障锁存期间不会重新使能。 */
void TMC2209_Enable(void);

/* 停止 STEP 输出、清空速度斜坡，并拉高 EN 关闭驱动。 */
void TMC2209_Disable(void);

/* 设置 DIR 电平：0/1 对应两个相反旋转方向，实际正反由接线决定。 */
void TMC2209_SetDirection(uint8_t dir);

/* 直接设置 STEP 脉冲频率，单位 Hz；0 表示停止输出。 */
void TMC2209_SetSpeed(uint32_t frequency_hz);

/* 只停止当前 STEP；若斜坡目标非零，下一次 2 ms 斜坡扫描会继续运行。 */
void TMC2209_Stop(void);

/* 阻塞式运行指定脉冲数，steps 的正负决定方向，频率单位 Hz。 */
void TMC2209_Step(int32_t steps, uint32_t frequency_hz);

/* 根据整步数、微步设置和减速比计算输出轴一圈所需 STEP 脉冲数。 */
uint32_t TMC2209_GetPulsesPerRev(void);

/* 把 STEP 脉冲数换算为输出轴角度，单位 °。 */
float TMC2209_StepsToDegrees(int32_t steps);

/* 把输出轴角度换算为 STEP 脉冲数。 */
int32_t TMC2209_DegreesToSteps(float degrees);

/* 立即设置 STEP 角速度大小；调用前需 Enable 并用 SetDirection 设置方向。 */
void TMC2209_SetAngularVelocity(float deg_per_sec);

/* 阻塞式转动：degrees 的正负决定方向，第二个参数单位为 °/s。 */
void TMC2209_RotateDegrees(float degrees, float deg_per_sec);

/*
 * 非阻塞相对角度运动：同样按“转多少度、以多快转”调用，但函数会立即返回。
 * 由 Timer.c 每 1 ms 调用 TMC2209_AngleMoveTick1ms() 结束本次脉冲输出，
 * 因此适合 TASK3 等不能阻塞的定时器控制。
 */
void TMC2209_RotateDegreesAsync(float degrees, float deg_per_sec);
void TMC2209_AngleMoveTick1ms(void);
/* 取消当前非阻塞角度运动，停止 STEP，但不关闭 EN 保持力矩。 */
void TMC2209_CancelAngleMove(void);

/*
 * 非阻塞绝对位置运动。
 * 位置以实际已完成的 STEP 脉冲数保存；Task4 启动时把当前机械位置定义为 0，
 * 随后的目标角度均相对这个软件零点。它和 AccelStepper 的 currentPosition()
 * 一样是开环软件坐标：断电、手动拨动、堵转或丢步后必须重新定义零点。
 */
void TMC2209_SetCurrentPositionSteps(int32_t steps);
int32_t TMC2209_GetCurrentPositionSteps(void);
float TMC2209_GetCurrentPositionDegrees(void);
bool TMC2209_HasPositionReference(void);

/* 设置输出轴的软件软限位；最小值大于最大值时函数会自动交换。 */
void TMC2209_SetSoftLimitsDegrees(float minimum_deg, float maximum_deg);

/*
 * 非阻塞移动到绝对角度目标，第二个参数是角速度（°/s）。
 * 返回 false 表示尚未定义零点、驱动故障、未初始化或速度无效。
 * 由 TIM9 的 CC1 中断逐脉冲更新当前位置，不能在主循环中等待它完成。
 */
bool TMC2209_MoveToDegreesAsync(float target_deg, float deg_per_sec);
bool TMC2209_IsPositionMoveActive(void);
void TMC2209_CancelPositionMove(void);

/* 仅供定时器层调用：TIM2 每 1 ms 调度方向建立时间，TIM9 每脉冲完成一次计数。 */
void TMC2209_PositionMoveTick1ms(void);
void TMC2209_PositionPulseFinished(void);

/* 非阻塞斜坡：speed 为带符号目标角速度（°/s），accel 单位为 °/s²。 */
void TMC2209_SetAngleSpeed(float speed, float accel);

/* 由 Timer.c 每 2 ms 调用一次，不要在主循环重复调用。 */
void TMC2209_SpeedRamp_Update(void);

/* 设置微步 MRES；超出范围时自动使用 1/16 微步。 */
void TMC2209_SetMicrostep(uint8_t mres);

/* 设置保持电流、运行电流和保持延时，电流参数有效范围均按寄存器位宽截断。 */
void TMC2209_SetCurrent(uint8_t hold_current, uint8_t run_current,
                        uint8_t hold_delay);

/* 只修改运行电流，保留最近设置的保持电流。 */
void TMC2209_SetRunCurrent(uint8_t run_current);

/* 通过单线 UART 写入一个 32 位 TMC2209 寄存器。 */
void TMC2209_WriteRegister(uint8_t reg, uint32_t value);

/* 通过单线 UART 读取一个 32 位寄存器，并检查回复地址及 CRC。 */
bool TMC2209_ReadRegister(uint8_t reg, uint32_t *value);

/* 在 DIAG 引脚 EXTI 回调中调用：锁存故障并立即禁用驱动。 */
void TMC2209_DiagIRQ(void);

/* 清除 DIAG 锁存；清除后仍保持禁用，需重新调用 Enable/SetAngleSpeed。 */
void TMC2209_ClearStall(void);

/* 返回只读状态指针，调用者不应修改指向的内部状态。 */
const TMC2209_State *TMC2209_GetState(void);

#endif
