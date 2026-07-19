/*
 * control.h — TMC2209 步进电机驱动接口 (二维云台: Yaw + Pitch)
 *
 * 适用于: 2026南京林业大学电子设计竞赛 E题 — 自动瞄准及绘图装置
 *
 * 驱动芯片: TMC2209 (Trinamic)
 *   - UART 配置寄存器 (波特率 115200, 单线半双工 PDN_UART)
 *   - STEP/DIR 硬件脉冲控制 (TIM3 PWM 生成 STEP 信号)
 *   - StealthChop 静音驱动 + CoolStep 自适应电流 + StallGuard 堵转检测
 *
 * 电机参数: 42步进电机, 200整步/转, 1.8°/步, 1:1 直驱 (无减速器)
 * 当前配置: 1/16 微步 → 3200 脉冲/转 → 0.1125°/脉冲
 */
#ifndef _control_H_
#define _control_H_
#include "main.h"

/* ========== TMC2209 寄存器地址 ========== */
#define TMC_REG_GCONF          0x00
#define TMC_REG_GSTAT          0x01
#define TMC_REG_IFCNT          0x02
#define TMC_REG_SLAVECONF      0x03
#define TMC_REG_OTP_PROG       0x04
#define TMC_REG_OTP_READ       0x05
#define TMC_REG_IOIN           0x06
#define TMC_REG_FACTORY_CONF   0x07
#define TMC_REG_IHOLD_IRUN     0x10
#define TMC_REG_TPOWERDOWN     0x11
#define TMC_REG_TSTEP          0x12
#define TMC_REG_TPWMTHRS       0x13
#define TMC_REG_TCOOLTHRS      0x14
#define TMC_REG_THIGH          0x15
#define TMC_REG_VACTUAL        0x22
#define TMC_REG_CHOPCONF       0x6C
#define TMC_REG_COOLCONF       0x6D
#define TMC_REG_DRV_STATUS     0x6F
#define TMC_REG_PWMCONF        0x70
#define TMC_REG_PWM_SCALE      0x71
#define TMC_REG_PWM_AUTO       0x72
#define TMC_REG_SGTHRS         0x40

/* ========== TMC2209 错误码 ========== */
#define TMC_ERR_TX_FAIL        0xFFFFFFFE
#define TMC_ERR_RX_FAIL        0xFFFFFFFD
#define TMC_ERR_CRC            0xFFFFFFFC
#define TMC_IS_ERROR(val)      ((val) >= 0xFFFFFFFC)

/* ========== GCONF 位域 ========== */
#define TMC_GCONF_I_SCALE_ANALOG      (1<<0)
#define TMC_GCONF_INTERNAL_RSENSE     (1<<1)
#define TMC_GCONF_EN_SPREADCYCLE      (1<<2)
#define TMC_GCONF_SHAFT               (1<<3)
#define TMC_GCONF_INDEX_OTPW          (1<<4)
#define TMC_GCONF_INDEX_STEP          (1<<5)
#define TMC_GCONF_PDN_DISABLE         (1<<6)
#define TMC_GCONF_MSTEP_REG_SELECT    (1<<7)
#define TMC_GCONF_MULTISTEP_FILT      (1<<8)
#define TMC_GCONF_TEST_MODE           (1<<9)

/* ========== CHOPCONF 位域 ========== */
#define TMC_CHOPCONF_TOFF_Pos         0
#define TMC_CHOPCONF_HSTRT_Pos        4
#define TMC_CHOPCONF_HEND_Pos         7
#define TMC_CHOPCONF_TBL_Pos          15
#define TMC_CHOPCONF_VSENSE           (1<<17)
#define TMC_CHOPCONF_MRES_Pos         24
#define TMC_CHOPCONF_INTPOL           (1<<28)
#define TMC_CHOPCONF_DEDGE            (1<<29)
#define TMC_CHOPCONF_DISS2G           (1<<30)
#define TMC_CHOPCONF_DISS2VS          (1<<31)

/* ========== IHOLD_IRUN 位域 ========== */
#define TMC_IHOLD_IRUN_IHOLD_Pos      0
#define TMC_IHOLD_IRUN_IRUN_Pos       8
#define TMC_IHOLD_IRUN_IHOLDDELAY_Pos 16

/* ========== PWMCONF 位域 (StealthChop PWM 配置) ==========
 * PWM_OFS [7:0]    : PWM 幅度偏移, 静止时的基础 PWM 幅度 (默认 36)
 * PWM_GRAD [15:8]  : 速度相关梯度, 补偿反电动势 (默认 20, 配合 AUTOGRAD 自动调谐)
 * PWM_FREQ [17:16] : PWM 频率 0=23.44k 1=35.15k 2=46.51k 3=58.82kHz
 * PWM_AUTOSCALE[18]: 自动电流缩放, StealthChop 核心功能
 * PWM_AUTOGRAD [19]: 自动梯度适配, 运动中自动优化 PWM_GRAD
 * FREEWHEEL [21:20]: 静止模式 0=正常 1=自由 2=强制动 3=制动
 * PWM_REG [25:22]  : 自动缩放比例系数 (默认 4)
 * PWM_LIM [29:26]  : PWM_SCALE_AUTO 上限 (默认 12)
 */
#define TMC_PWMCONF_OFS_Pos           0
#define TMC_PWMCONF_GRAD_Pos          8
#define TMC_PWMCONF_FREQ_Pos          16
#define TMC_PWMCONF_AUTOSCALE        (1U<<18)
#define TMC_PWMCONF_AUTOGRAD         (1U<<19)
#define TMC_PWMCONF_FREEWHEEL_Pos     20
#define TMC_PWMCONF_REG_Pos           22
#define TMC_PWMCONF_LIM_Pos           26

/* ========== COOLCONF 位域 (CoolStep 自适应电流) ==========
 * SEMIN [3:0]   : 下限阈值, SG_RESULT 低于此值时增加电流 (×32 缩放)
 * SEUP [5:4]    : 电流增量步进 0=1 1=2 2=4 3=8
 * SEMAX [11:8]  : 上限阈值, SG_RESULT 高于此值时降低电流
 * SEDN [13:12]  : 电流减量步进 0=32 1=8 2=2 3=1
 * SEIMIN [15]   : 最小电流选择 0=1/2 IRUN 1=1/4 IRUN
 */
#define TMC_COOLCONF_SEMIN_Pos        0
#define TMC_COOLCONF_SEUP_Pos         4
#define TMC_COOLCONF_SEMAX_Pos        8
#define TMC_COOLCONF_SEDN_Pos         12
#define TMC_COOLCONF_SEIMIN          (1U<<15)

/* ========== 电机物理参数 ==========
 * MOTOR_FULLSTEPS_REV: 42步进电机(1.8°/步), 每圈200整步
 * MOTOR_REDUCTION:     减速比, 1表示电机轴与云台轴直连(1:1直驱)
 *                      题目要求激光在50cm外靶面精确瞄准/绘图,
 *                      1:1直驱消除了减速器的回程间隙, 提高定位精度
 * MOTOR_OUTPUT_STEPS_REV: 云台轴每转对应的电机输出步数(整步)
 */
#define MOTOR_FULLSTEPS_REV     200
#define MOTOR_REDUCTION         1
#define MOTOR_OUTPUT_STEPS_REV  (MOTOR_FULLSTEPS_REV * MOTOR_REDUCTION)

/* ========== 微步分辨率 (mres) ==========
 * mres编码  微步数    每转脉冲数(1:1直驱, 200整步电机)
 * 0         1/256     51200
 * 1         1/128     25600
 * 2         1/64      12800
 * 3         1/32      6400
 * 4         1/16      3200    ← 当前使用, 精度与速度的平衡点
 * 5         1/8       1600
 * 6         1/4       800
 * 7         1/2       400
 * 8         全步      200
 * 说明: 脉冲数 = 200(整步) × 微步数 × MOTOR_REDUCTION(1:1=1)
 */
#define TMC_MRES_256    0
#define TMC_MRES_128    1
#define TMC_MRES_64     2
#define TMC_MRES_32     3
#define TMC_MRES_16     4
#define TMC_MRES_8      5
#define TMC_MRES_4      6
#define TMC_MRES_2      7
#define TMC_MRES_FULL   8

/* ========== 轴选择 ========== */
typedef enum {
    AXIS_YAW   = 0,
    AXIS_PITCH = 1
} Axis_TypeDef;

/* ========== 电机状态结构体 ========== */
typedef struct {
    uint8_t  enabled;            /* EN 使能状态: 1=锁定 0=自由 */
    int32_t  target_pos;         /* 目标位置 (脉冲数) */
    int32_t  current_pos;        /* 当前位置 (脉冲数) */
    uint32_t pulse_freq;         /* 当前脉冲频率 (Hz) */
    uint32_t run_current;        /* 运行电流 (0~31) */
    uint32_t hold_current;       /* 保持电流 (0~31) */
    uint8_t  microstep_res;      /* 微步分辨率索引 */
    uint8_t  stealthchop;        /* StealthChop 使能标志 */
    uint8_t  spreadcycle;        /* SpreadCycle 使能标志 */
} Motor_TypeDef;

/* ========== 全局电机实例 ========== */
extern Motor_TypeDef motor_yaw;
extern Motor_TypeDef motor_pitch;
extern volatile uint32_t tmc_error_count;

/* ========== TMC2209 驱动 API ========== */
void TMC2209_Init(uint8_t default_mres);
void TMC2209_UART_Write(Axis_TypeDef axis, uint8_t reg, uint32_t data);

/* 电流设置 */
void TMC2209_SetCurrent(Axis_TypeDef axis, uint8_t ihold, uint8_t irun, uint8_t iholddelay);
void TMC2209_SetRunCurrent(Axis_TypeDef axis, uint8_t irun);

/* 微步设置 */
void TMC2209_SetMicrostep(Axis_TypeDef axis, uint8_t mres);

/* 模式切换 */
void TMC2209_EnableStealthChop(Axis_TypeDef axis);
void TMC2209_EnableSpreadCycle(Axis_TypeDef axis);
void TMC2209_SetStealthChopThreshold(Axis_TypeDef axis, uint32_t tpwmthrs);

/* CoolStep 自适应电流 (降低发热, 延长续航) */
void TMC2209_EnableCoolStep(Axis_TypeDef axis, uint8_t semin, uint8_t semax);
void TMC2209_DisableCoolStep(Axis_TypeDef axis);
void TMC2209_SetCoolStepThreshold(Axis_TypeDef axis, uint32_t tcoolthrs);

/* StallGuard 堵转检测 (可用于无传感器回零) */
void TMC2209_SetStallGuardThreshold(Axis_TypeDef axis, uint8_t sgthrs);

/* 堵转检测 */
uint32_t TMC2209_ReadDRVStatus(Axis_TypeDef axis);
uint8_t  TMC2209_ReadStallGuard(Axis_TypeDef axis);

/* 诊断调试 */
uint32_t TMC2209_ReadRegister(Axis_TypeDef axis, uint8_t reg);
void     TMC2209_Diagnose(Axis_TypeDef axis);

/* 电机控制 */
void TMC2209_Enable(Axis_TypeDef axis);
void TMC2209_Disable(Axis_TypeDef axis);
void TMC2209_SetSpeed(Axis_TypeDef axis, uint32_t freq_hz);
void TMC2209_Stop(Axis_TypeDef axis);
void TMC2209_SetDirection(Axis_TypeDef axis, uint8_t dir);
void TMC2209_Step(Axis_TypeDef axis, int32_t steps, uint32_t freq_hz);

/* VACTUAL 速度控制 (UART 内部脉冲发生器, 无需 TIM3 STEP)
 * 用于双轴独立调速: 一个轴用 VACTUAL, 另一个轴用 TIM3 PWM */
void TMC2209_SetVACTUAL(Axis_TypeDef axis, float deg_per_sec);
void TMC2209_StopVACTUAL(Axis_TypeDef axis);

/* 角度/速度换算 (1:1 直驱, 无减速器) */
uint32_t TMC2209_GetPulsesPerRev(Axis_TypeDef axis);
float    TMC2209_StepsToDegrees(Axis_TypeDef axis, int32_t steps);
int32_t  TMC2209_DegreesToSteps(Axis_TypeDef axis, float degrees);
void     TMC2209_SetAngularVelocity(Axis_TypeDef axis, float deg_per_sec);
void     TMC2209_RotateDegrees(Axis_TypeDef axis, float degrees, uint32_t freq_hz);

/* 激光 */
void Laser_Set(uint8_t on);
void TMC2209_SetAngleSpeed(Axis_TypeDef axis, float speed, float accel);
void TMC2209_SpeedRamp_Update(void);
#endif
