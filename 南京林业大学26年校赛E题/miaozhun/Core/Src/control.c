#include "control.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

/* ========================================================================== */
/*  TMC2209 步进电机驱动 — 二维云台控制 (自动瞄准及绘图装置)                    */
/*                                                                            */
/*  硬件连接:                                                                  */
/*    Yaw   轴: USART2(PA2=TX) + DIR(PB12) + EN(PB13) + STEP(TIM3_CH1/PC6)    */
/*    Pitch 轴: USART3(PB10=TX) + DIR(PB14) + EN(PB15) + STEP(TIM3_CH2/PC7)  */
/*                                                                            */
/*  控制架构:                                                                  */
/*    OpenMV/PID → TMC2209_SetAngleSpeed(目标角速度)                          */
/*              → TMC2209_SpeedRamp_Update(斜坡加减速, TIM1中断 2ms)           */
/*              → TMC2209_SetAngularVelocity → TMC2209_SetSpeed → TIM3 PWM    */
/*                                                                            */
/*  驱动特性 (参考 TMC2209_STM32 开源库优化):                                  */
/*    - StealthChop:  PWM 静音驱动, 自动电流缩放+梯度适配                      */
/*    - CoolStep:     自适应电流, 低负载降流减热                                */
/*    - StallGuard4:  堵转检测, 可用于无传感器回零                              */
/*    - UART 通信:    CRC8 校验 + 读取重试 (MAX_READ_RETRIES)                  */
/*    - 1/16 微步:    3200 脉冲/转, 0.1125°/脉冲 (1:1直驱)                    */
/* ========================================================================== */

/* ========== 外设句柄 (由 CubeMX 生成在 main.c) ========== */
extern UART_HandleTypeDef huart2;   /* Yaw   TMC2209: PA2=TX */
extern UART_HandleTypeDef huart3;   /* Pitch TMC2209: PB10=TX */
extern TIM_HandleTypeDef  htim3;    /* STEP脉冲: PC6=CH1, PC7=CH2 */

/* ========== Arduino库移植的常量和类型 ========== */
#define SYNC 0x05
#define RW_READ 0
#define RW_WRITE 1
#define READ_REPLY_SERIAL_ADDRESS 0xFF

#define WRITE_READ_REPLY_DATAGRAM_SIZE 8
#define READ_REQUEST_DATAGRAM_SIZE 4

#define SERIAL_ADDRESS_0 0
#define SERIAL_ADDRESS_1 1
#define SERIAL_ADDRESS_2 2
#define SERIAL_ADDRESS_3 3

#define MAX_READ_RETRIES 5
#define READ_RETRY_DELAY_MS 20

/* ========== 额外寄存器地址 ========== */
#define TMC_REG_SGTHRS  0x40

/* ========== 微步分辨率常量 ========== */
#define MRES_256 0x00
#define MRES_128 0x01
#define MRES_064 0x02
#define MRES_032 0x03
#define MRES_016 0x04
#define MRES_008 0x05
#define MRES_004 0x06
#define MRES_002 0x07
#define MRES_001 0x08

/* ========== 电流设置常量 ========== */
#define CURRENT_SETTING_MIN 0
#define CURRENT_SETTING_MAX 31
#define PERCENT_MIN 0
#define PERCENT_MAX 100
#define HOLD_DELAY_MIN 0
#define HOLD_DELAY_MAX 15

/* ========== 默认值 ========== */
#define IHOLD_DEFAULT 16
#define IRUN_DEFAULT 31
#define IHOLDDELAY_DEFAULT 1
#define CHOPPER_CONFIG_DEFAULT 0x10000053
#define TBL_DEFAULT 2
#define HEND_DEFAULT 0
#define HSTART_DEFAULT 5
#define TOFF_DEFAULT 3
#define TOFF_DISABLE 0
#define PWM_CONFIG_DEFAULT 0xC10D0024
#define PWM_OFFSET_DEFAULT 0x24
#define PWM_GRAD_DEFAULT 0x14

/* ========== 电机实例 ========== */
Motor_TypeDef motor_yaw   = {0};
Motor_TypeDef motor_pitch = {0};
volatile uint32_t tmc_error_count = 0;

/* ========== 轴级上下文 ========== */
#define SPEED_RAMP_DT 0.002f

typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef       *en_port;   uint16_t en_pin;
    uint32_t            tim_channel;
    Motor_TypeDef      *motor;
    uint8_t             serial_address;

    float current_speed;
    float target_speed;
    float accel;

    uint32_t global_config;
    uint32_t chopper_config;
    uint32_t pwm_config;
    uint32_t driver_current;
    uint32_t cool_config;
    bool cool_step_enabled;

    int8_t  last_dir;     /* 上次设置的方向 (-1=未设置, 0=正, 1=反) */
} AxisCtx_t;

static AxisCtx_t axis_ctx[2] = {
    [AXIS_YAW]   = {&huart2, GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13, TIM_CHANNEL_1, &motor_yaw,   SERIAL_ADDRESS_0, 0, 0, 3000.0f, 0, 0, 0, 0, 0, false, -1},
    [AXIS_PITCH] = {&huart3, GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, TIM_CHANNEL_2, &motor_pitch, SERIAL_ADDRESS_0, 0, 0, 3000.0f, 0, 0, 0, 0, 0, false, -1},
};

/* ========== 内部函数声明 ========== */
static void TMC2209_GPIO_Init(void);
static void TMC2209_SetPWM(Axis_TypeDef axis, uint32_t arr);
static uint8_t TMC2209_CRC8(const uint8_t *data, uint8_t len);
static void TMC2209_WriteRegister(Axis_TypeDef axis, uint8_t reg, uint32_t data);
static uint32_t TMC2209_ReadRegisterInternal(Axis_TypeDef axis, uint8_t reg);
static void TMC2209_reverseData(uint32_t data, uint8_t *buf);
static uint32_t TMC2209_bytesToUint32(const uint8_t *buf);
static void TMC2209_setOperationModeToSerial(Axis_TypeDef axis);
static void TMC2209_setRegistersToDefaults(Axis_TypeDef axis);
static void TMC2209_writeStoredGlobalConfig(Axis_TypeDef axis);
static void TMC2209_InitAxis(Axis_TypeDef axis, uint8_t mres);

/* ========================================================================== */
/*                           公开函数                                         */
/* ========================================================================== */

/**
  * @brief  初始化单个轴的 TMC2209 寄存器配置
  *
  *         配置流程参考 TMC2209 datasheet 和开源驱动库 TMC2209_STM32:
  *         1. GCONF       — 切换 UART 串行模式, 启用 StealthChop(关闭 SpreadCycle)
  *         2. IHOLD_IRUN  — 运行/保持电流
  *         3. CHOPCONF    — 斩波器参数 + 微步分辨率
  *         4. PWMCONF     — StealthChop PWM 自动缩放/梯度适配
  *         5. TPWMTHRS    — StealthChop→SpreadCycle 切换阈值
  *         6. CoolStep    — 自适应电流 (降低空载发热, 延长绘图续航)
  *         7. SGTHRS      — StallGuard 堵转检测阈值
  *
  * @param  axis  轴号 (AXIS_YAW / AXIS_PITCH)
  * @param  mres  微步分辨率索引 (0=1/256 ... 8=全步)
  */
static void TMC2209_InitAxis(Axis_TypeDef axis, uint8_t mres)
{
    Motor_TypeDef *m = axis_ctx[axis].motor;
    m->microstep_res = mres;
    m->stealthchop   = 1;
    m->run_current   = 20;
    m->hold_current  = 8;

    /* Keep the register sequence identical to the proven yuntai project. */
    TMC2209_setOperationModeToSerial(axis);
    TMC2209_setRegistersToDefaults(axis);
    /* 保持UART/VACTUAL和寄存器微步配置有效。 */
    TMC2209_WriteRegister(axis, TMC_REG_GCONF,
        TMC_GCONF_PDN_DISABLE |
        TMC_GCONF_MSTEP_REG_SELECT |
        TMC_GCONF_MULTISTEP_FILT);
    HAL_Delay(10);

    TMC2209_WriteRegister(axis, TMC_REG_PWMCONF,
        (1U << 18) | (4U << 16) | (200U << 0));
    HAL_Delay(5);
    TMC2209_SetCurrent(axis, 8, 20, 4);
    HAL_Delay(5);
    TMC2209_SetMicrostep(axis, mres);
    HAL_Delay(5);
    TMC2209_WriteRegister(axis, TMC_REG_TPWMTHRS, 500);
    HAL_Delay(5);
    TMC2209_WriteRegister(axis, TMC_REG_CHOPCONF,
        (4  << TMC_CHOPCONF_TOFF_Pos)  |
        (5  << TMC_CHOPCONF_HSTRT_Pos) |
        (0  << TMC_CHOPCONF_HEND_Pos)  |
        (2  << TMC_CHOPCONF_TBL_Pos)   |
        TMC_CHOPCONF_VSENSE            |
        (mres << TMC_CHOPCONF_MRES_Pos) |
        TMC_CHOPCONF_INTPOL);
    HAL_Delay(5);
}

/**
  * @brief  TMC2209 总初始化: GPIO → TIM3 PWM → 寄存器配置
  *
  *         初始化流程:
  *         1. 配置 DIR/EN 引脚为推挽输出
  *         2. 启动 TIM3 CH1/CH2 PWM (STEP 脉冲源, 占空比初始 0=停止)
  *         3. 拉高 EN = 电机自由态 (待 InitAxis 完成后再按需使能)
  *         4. 配置两个轴的 TMC2209 寄存器 (GCONF/IHOLD/CHOPCONF/PWMCONF等)
  *         5. 停止脉冲输出, DIR 拉低 (默认正转方向)
  *
  * @param  default_mres  默认微步分辨率 (通常 TMC_MRES_16 = 1/16)
  */
void TMC2209_Init(uint8_t default_mres)
{
    TMC2209_GPIO_Init();

    /* 启动 STEP 脉冲 PWM 输出 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    /* 初始化时拉高 EN = 电机自由态 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    motor_yaw.enabled = 0;
    motor_pitch.enabled = 0;

    /* 初始化两个轴 */
    TMC2209_InitAxis(AXIS_YAW, default_mres);
    TMC2209_InitAxis(AXIS_PITCH, default_mres);

    TMC2209_Stop(AXIS_YAW);
    TMC2209_Stop(AXIS_PITCH);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void TMC2209_SetCurrent(Axis_TypeDef axis, uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    uint32_t val = ((iholddelay & 0x0F) << TMC_IHOLD_IRUN_IHOLDDELAY_Pos)
                 | ((irun       & 0x1F) << TMC_IHOLD_IRUN_IRUN_Pos)
                 |  (ihold      & 0x1F);
    TMC2209_WriteRegister(axis, TMC_REG_IHOLD_IRUN, val);

    if (axis == AXIS_YAW) {
        motor_yaw.run_current  = irun;
        motor_yaw.hold_current = ihold;
    } else {
        motor_pitch.run_current  = irun;
        motor_pitch.hold_current = ihold;
    }
}

void TMC2209_SetRunCurrent(Axis_TypeDef axis, uint8_t irun)
{
    Motor_TypeDef *m = (axis == AXIS_YAW) ? &motor_yaw : &motor_pitch;
    TMC2209_SetCurrent(axis, m->hold_current, irun, 4);
}

void TMC2209_SetMicrostep(Axis_TypeDef axis, uint8_t mres)
{
    uint32_t chopconf =
        (4  << TMC_CHOPCONF_TOFF_Pos)  |
        (5  << TMC_CHOPCONF_HSTRT_Pos) |
        (0  << TMC_CHOPCONF_HEND_Pos)  |
        (2  << TMC_CHOPCONF_TBL_Pos)   |
        TMC_CHOPCONF_VSENSE            |
        ((mres & 0xF) << TMC_CHOPCONF_MRES_Pos) |
        TMC_CHOPCONF_INTPOL;

    TMC2209_WriteRegister(axis, TMC_REG_CHOPCONF, chopconf);

    if (axis == AXIS_YAW)
        motor_yaw.microstep_res = mres;
    else
        motor_pitch.microstep_res = mres;
}

void TMC2209_EnableStealthChop(Axis_TypeDef axis)
{
    /* 保留 UART 模式位 (PDN_DISABLE/MSTEP_REG_SELECT/MULTISTEP_FILT),
     * 仅清除 EN_SPREADCYCLE → 启用 StealthChop 静音模式 */
    uint32_t gconf = TMC_GCONF_PDN_DISABLE | TMC_GCONF_MSTEP_REG_SELECT | TMC_GCONF_MULTISTEP_FILT;
    TMC2209_WriteRegister(axis, TMC_REG_GCONF, gconf);
    if (axis == AXIS_YAW) { motor_yaw.stealthchop = 1; motor_yaw.spreadcycle = 0; }
    else                  { motor_pitch.stealthchop = 1; motor_pitch.spreadcycle = 0; }
}

void TMC2209_EnableSpreadCycle(Axis_TypeDef axis)
{
    /* 保留 UART 模式位, 设置 EN_SPREADCYCLE → 启用 SpreadCycle 大扭矩模式 */
    uint32_t gconf = TMC_GCONF_PDN_DISABLE | TMC_GCONF_MSTEP_REG_SELECT |
                     TMC_GCONF_MULTISTEP_FILT | TMC_GCONF_EN_SPREADCYCLE;
    TMC2209_WriteRegister(axis, TMC_REG_GCONF, gconf);
    if (axis == AXIS_YAW) { motor_yaw.spreadcycle = 1; motor_yaw.stealthchop = 0; }
    else                  { motor_pitch.spreadcycle = 1; motor_pitch.stealthchop = 0; }
}

void TMC2209_SetStealthChopThreshold(Axis_TypeDef axis, uint32_t tpwmthrs)
{
    TMC2209_WriteRegister(axis, TMC_REG_TPWMTHRS, tpwmthrs & 0xFFFFF);
}

/* ====================== CoolStep 自适应电流 ====================== */

/**
  * @brief  启用 CoolStep — 根据负载自动调节电流
  *
  *         低负载时降低电流 (减热), 高负载时增加电流 (保扭).
  *         基于 StallGuard4 测量值 SG_RESULT:
  *           SG_RESULT < SEMIN×32  → 增加电流 (负载增大)
  *           SG_RESULT ≥ (SEMIN+SEMAX+1)×32 → 降低电流 (负载减小)
  *
  * @param  axis   轴号
  * @param  semin  下限阈值 (0~15), 0=关闭 CoolStep
  * @param  semax  上限阈值 (0~15)
  */
void TMC2209_EnableCoolStep(Axis_TypeDef axis, uint8_t semin, uint8_t semax)
{
    uint32_t coolconf =
        ((semin & 0x0F) << TMC_COOLCONF_SEMIN_Pos) |
        (1U              << TMC_COOLCONF_SEUP_Pos)  |  /* SEUP=1: 每次增加 2/32 电流 */
        ((semax & 0x0F) << TMC_COOLCONF_SEMAX_Pos) |
        (0U              << TMC_COOLCONF_SEDN_Pos);    /* SEDN=0: 每次降低 1/32 电流 */
    TMC2209_WriteRegister(axis, TMC_REG_COOLCONF, coolconf);
    axis_ctx[axis].cool_step_enabled = true;
}

void TMC2209_DisableCoolStep(Axis_TypeDef axis)
{
    TMC2209_WriteRegister(axis, TMC_REG_COOLCONF, 0);
    axis_ctx[axis].cool_step_enabled = false;
}

/**
  * @brief  设置 CoolStep 启用速度阈值
  * @note   低于此速度 CoolStep 关闭 (满电流保证定位精度),
  *         高于此速度 CoolStep 启用 (节能降热)
  */
void TMC2209_SetCoolStepThreshold(Axis_TypeDef axis, uint32_t tcoolthrs)
{
    TMC2209_WriteRegister(axis, TMC_REG_TCOOLTHRS, tcoolthrs & 0xFFFFF);
}

/* ====================== StallGuard 堵转检测 ====================== */

/**
  * @brief  设置 StallGuard4 堵转检测阈值
  * @note   SG_RESULT ≤ SGTHRS×2 时触发堵转, 可用于无传感器回零.
  *         阈值需实验确定, 典型范围 10~100.
  * @param  axis    轴号
  * @param  sgthrs  阈值 (0~255), 0=关闭检测
  */
void TMC2209_SetStallGuardThreshold(Axis_TypeDef axis, uint8_t sgthrs)
{
    TMC2209_WriteRegister(axis, TMC_REG_SGTHRS, sgthrs);
}

uint32_t TMC2209_ReadDRVStatus(Axis_TypeDef axis)
{
    return TMC2209_ReadRegisterInternal(axis, TMC_REG_DRV_STATUS);
}

uint8_t TMC2209_ReadStallGuard(Axis_TypeDef axis)
{
    uint32_t status = TMC2209_ReadDRVStatus(axis);
    return (uint8_t)(status & 0x3FF);
}

/* ====================== 电机运动控制 ====================== */

void TMC2209_Enable(Axis_TypeDef axis)
{
    HAL_GPIO_WritePin(axis_ctx[axis].en_port, axis_ctx[axis].en_pin, GPIO_PIN_RESET);
    axis_ctx[axis].motor->enabled = 1;
}

void TMC2209_Disable(Axis_TypeDef axis)
{
    TMC2209_Stop(axis);
    HAL_GPIO_WritePin(axis_ctx[axis].en_port, axis_ctx[axis].en_pin, GPIO_PIN_SET);
    axis_ctx[axis].motor->enabled = 0;
}

/**
  * @brief  设置电机脉冲频率 (STEP 信号)
  *
  *         TIM3 计数时钟 = 84MHz / (83+1) = 1MHz
  *         脉冲频率 = 1MHz / ARR
  *         例如: freq_hz=200 → ARR=5000 → 200Hz → 1/16微步下约 22.5°/s
  *
  * @param  axis     轴号
  * @param  freq_hz  脉冲频率 (Hz), 0=停止
  */
void TMC2209_SetSpeed(Axis_TypeDef axis, uint32_t freq_hz)
{
    /*
     * Do not restart TIM3 when the quantized pulse frequency is unchanged.
     * SpeedRamp_Update runs about every 1 ms, while low-speed STEP periods can
     * be much longer (10 deg/s at 1/16 microstep is only about 88 Hz).  Reissuing
     * EGR.UG on every ramp tick resets the counter before it reaches CCR and
     * suppresses every STEP pulse.
     */
    if (axis_ctx[axis].motor->pulse_freq == freq_hz) {
        return;
    }

    axis_ctx[axis].motor->pulse_freq = freq_hz;

    if (freq_hz == 0) {
        TMC2209_SetPWM(axis, 0);
        return;
    }

    /* 
     * TIM3 时钟: APB1 = 42MHz (HCLK=168MHz, APB1分频=4, 但TIM3在APB1上且倍频后实际为84MHz)
     * Prescaler = 83, 所以计数频率 = 84MHz / (83+1) = 1MHz
     * ARR = 1MHz / freq_hz - 1
     * 例如 freq_hz=200 时, ARR = 5000-1 = 4999
     */
    uint32_t arr = 1000000U / freq_hz;
    if (arr < 2)  arr = 2;        /* 最小周期, 避免 ARR=0 或 1 */
    if (arr > 65535) arr = 65535; /* TIM3 是 16 位定时器 */

    TMC2209_SetPWM(axis, arr - 1);
}

void TMC2209_Stop(Axis_TypeDef axis)
{
    axis_ctx[axis].motor->pulse_freq = 0;
    TMC2209_SetPWM(axis, 0);
}

void TMC2209_SetDirection(Axis_TypeDef axis, uint8_t dir)
{
    GPIO_PinState state = dir ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(axis_ctx[axis].dir_port, axis_ctx[axis].dir_pin, state);
}

void TMC2209_Step(Axis_TypeDef axis, int32_t steps, uint32_t freq_hz)
{
    if (steps == 0 || freq_hz == 0) return;

    int32_t dir = (steps > 0) ? 0 : 1;
    if (steps < 0) steps = -steps;

    /* 确保电机已使能 */
    if (!axis_ctx[axis].motor->enabled) {
        TMC2209_Enable(axis);
        HAL_Delay(10); /* 给电机驱动器一些启动时间 */
    }

    TMC2209_SetDirection(axis, dir);
    HAL_Delay(2); /* 方向建立时间 */
    
    TMC2209_SetSpeed(axis, freq_hz);

    uint32_t delay_ms = (1000U * steps) / freq_hz + 1;
    HAL_Delay(delay_ms);

    TMC2209_Stop(axis);
}

/* ====================== VACTUAL 速度控制 ======================
 * TMC2209 VACTUAL 寄存器 (0x22): 24 位有符号值
 *   0    = 正常模式, 响应外部 STEP 脉冲
 *   ≠0   = 内部脉冲发生器, 忽略 STEP 引脚, 自行产生脉冲
 *   速度公式: v[μsteps/s] = VACTUAL × 0.715 Hz
 *   符号决定方向 (正=正转, 负=反转)
 *
 * 用途: 双轴独立调速 — Yaw 用 VACTUAL (UART控制),
 *       Pitch 用 TIM3 PWM (STEP控制), 互不干扰.
 */

/**
  * @brief  通过 VACTUAL 寄存器设置轴速度 (不依赖 TIM3)
  * @param  axis        轴号
  * @param  deg_per_sec 目标角速度 (°/s), 正=正转, 负=反转, 0=停止VACTUAL
  */
void TMC2209_SetVACTUAL(Axis_TypeDef axis, float deg_per_sec)
{
    if (deg_per_sec == 0.0f) {
        TMC2209_WriteRegister(axis, TMC_REG_VACTUAL, 0);
        return;
    }

    /* °/s → 微步/s → VACTUAL 值 */
    uint32_t ppr = TMC2209_GetPulsesPerRev(axis);
    float msteps_per_sec = deg_per_sec * (float)ppr / 360.0f;
    int32_t vactual = (int32_t)(msteps_per_sec / 0.715f);

    /* 限幅 ±(2^23 - 1) */
    if (vactual > 8388607)  vactual = 8388607;
    if (vactual < -8388607) vactual = -8388607;

    /* 方向: VACTUAL 符号即方向, 不需要单独设 DIR 引脚 */
    TMC2209_WriteRegister(axis, TMC_REG_VACTUAL, (uint32_t)vactual & 0xFFFFFF);
}

/**
  * @brief  停止 VACTUAL 控制, 恢复为 STEP 脉冲模式
  */
void TMC2209_StopVACTUAL(Axis_TypeDef axis)
{
    TMC2209_WriteRegister(axis, TMC_REG_VACTUAL, 0);
}

/* ====================== 角度/速度换算 (1:1 直驱, 无减速器) ======================
 * 云台轴与电机轴直连(MOTOR_REDUCTION=1), 角度换算直接基于电机参数:
 *   每转脉冲数 = 200(整步) × 微步数 × 1(减速比)
 * 例如 1/16 微步时: 200 × 16 = 3200 脉冲/转, 即 0.1125°/脉冲
 */

static const uint16_t microstep_table[] = {
    256, 128, 64, 32, 16, 8, 4, 2, 1
};

uint32_t TMC2209_GetPulsesPerRev(Axis_TypeDef axis)
{
    Motor_TypeDef *m = axis_ctx[axis].motor;
    uint16_t mult = microstep_table[m->microstep_res];
    return (uint32_t)MOTOR_FULLSTEPS_REV * mult * MOTOR_REDUCTION;
}

float TMC2209_StepsToDegrees(Axis_TypeDef axis, int32_t steps)
{
    uint32_t ppr = TMC2209_GetPulsesPerRev(axis);
    return (float)steps * 360.0f / (float)ppr;
}

int32_t TMC2209_DegreesToSteps(Axis_TypeDef axis, float degrees)
{
    uint32_t ppr = TMC2209_GetPulsesPerRev(axis);
    return (int32_t)(degrees * (float)ppr / 360.0f);
}

void TMC2209_SetAngularVelocity(Axis_TypeDef axis, float deg_per_sec)
{
    uint32_t ppr = TMC2209_GetPulsesPerRev(axis);
    uint32_t freq = (uint32_t)(deg_per_sec * (float)ppr / 360.0f);
    TMC2209_SetSpeed(axis, freq);
}

void TMC2209_RotateDegrees(Axis_TypeDef axis, float degrees, uint32_t freq_hz)
{
    int32_t steps = TMC2209_DegreesToSteps(axis, degrees);
    TMC2209_Step(axis, steps, freq_hz);
}

/* ====================== 激光控制 ====================== */

void Laser_Set(uint8_t on)
{
    if (on)
        LL_GPIO_SetOutputPin(GPIOE, GPIO_PIN_4);
    else
        LL_GPIO_ResetOutputPin(GPIOE, GPIO_PIN_4);
}

/* ========================================================================== */
/*            硬件初始化 (由 CubeMX 生成, 此处仅保留 GPIO 配置)               */
/* ========================================================================== */

/**
  * @brief  初始化 TMC2209 控制所需的 GPIO:
  *         PB12(Yaw DIR)  PB13(Yaw EN)  PB14(Pitch DIR)  PB15(Pitch EN)  输出
  *         注意: UART 和 TIM3 由 CubeMX 生成的 MX_USART2_UART_Init/MX_USART3_UART_Init/MX_TIM3_Init 初始化
  */
static void TMC2209_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12 = Yaw DIR */
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB13 = Yaw EN */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB14 = Pitch DIR */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB15 = Pitch EN */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ========================================================================== */
/*            诊断与调试函数                                                  */
/* ========================================================================== */

uint32_t TMC2209_ReadRegister(Axis_TypeDef axis, uint8_t reg)
{
    return TMC2209_ReadRegisterInternal(axis, reg);
}

void TMC2209_Diagnose(Axis_TypeDef axis)
{
    uint32_t gconf      = TMC2209_ReadRegister(axis, TMC_REG_GCONF);       HAL_Delay(5);
    uint32_t chopconf   = TMC2209_ReadRegister(axis, TMC_REG_CHOPCONF);    HAL_Delay(5);
    uint32_t ihold_irun = TMC2209_ReadRegister(axis, TMC_REG_IHOLD_IRUN);  HAL_Delay(5);
    uint32_t drv_status = TMC2209_ReadRegister(axis, TMC_REG_DRV_STATUS);  HAL_Delay(5);

    /* 诊断输出 — 在调试器中查看这些值，gconf==0xFFFFFFFF 表示 UART 通信失败 */
    (void)gconf; (void)chopconf; (void)ihold_irun; (void)drv_status;
}

/* ========================================================================== */
/*            内部工具函数                                                    */
/* ========================================================================== */

/**
  * @brief  设置 STEP 脉冲的 PWM 频率
  *
  *         TIM3 CH1(Yaw) 和 CH2(Pitch) 共享同一个 ARR 寄存器,
  *         因此两个轴的脉冲频率不能完全独立.
  *         实际影响: 当两个轴同时以不同速度运动时, 后设置的轴会
  *         覆盖 ARR, 导致两个轴短暂以相同频率运行.
  *
  *         解决方案(如需独立双轴调速):
  *         - 方案A: 将一个轴迁移到独立定时器(需 CubeMX 重新配置)
  *         - 方案B: 使用 TMC2209 的 VACTUAL 寄存器通过 UART 控制速度,
  *                  无需外部 STEP 脉冲 (参考 TMC2209_STM32 库 move_at_velocity)
  *
  *         当前设计: PID 斜坡控制每 2ms 更新一次, 两轴交替设置 ARR,
  *         宏观上近似独立. 对于瞄准定位(低速)和绘图(匀速)场景影响较小.
  *
  * @param  axis  轴号
  * @param  arr    自动重装载值 (0=停止脉冲, >0=频率=1MHz/(arr+1))
  */
static void TMC2209_SetPWM(Axis_TypeDef axis, uint32_t arr)
{
    uint32_t channel = axis_ctx[axis].tim_channel;
    TIM_TypeDef *tim = htim3.Instance;

    if (channel == TIM_CHANNEL_1) {
        if (arr == 0) {
            tim->CCR1 = 0;
            return;
        }
        tim->ARR = arr;
        tim->CCR1 = arr / 2;
    } else {
        if (arr == 0) {
            tim->CCR2 = 0;
            return;
        }
        tim->ARR = arr;
        tim->CCR2 = arr / 2;
    }
    tim->EGR = TIM_EGR_UG;
}

/**
  * @brief  计算CRC8 (多项式0x07)
  */
static uint8_t TMC2209_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
            byte >>= 1;
        }
    }
    return crc;
}

/**
  * @brief  反转32位数据的字节序（用于TMC2209协议）
  */
static void TMC2209_reverseData(uint32_t data, uint8_t *buf)
{
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8) & 0xFF;
    buf[3] = data & 0xFF;
}

/**
  * @brief  将4字节数组转换为32位整数（字节序已反转）
  */
static uint32_t TMC2209_bytesToUint32(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

/**
  * @brief  写入寄存器（移植自 Arduino 库）
  *
  *         TMC2209 UART 写帧格式 (8 字节):
  *         [SYNC=0x05] [地址] [寄存器|0x80] [数据3] [数据2] [数据1] [数据0] [CRC]
  *         数据为大端序 (高位在前), CRC8 多项式 0x07
  */
static void TMC2209_WriteRegister(Axis_TypeDef axis, uint8_t reg, uint32_t data)
{
    uint8_t buf[8];
    
    buf[0] = SYNC;  // sync
    buf[1] = axis_ctx[axis].serial_address;  // serial_address
    buf[2] = (reg & 0x7F) | (RW_WRITE << 7);  // register_address + rw bit
    TMC2209_reverseData(data, &buf[3]);  // data (4 bytes, reversed)
    buf[7] = TMC2209_CRC8(buf, 7);  // CRC

    /* 清空接收缓冲区 */
    while (__HAL_UART_GET_FLAG(axis_ctx[axis].huart, UART_FLAG_RXNE)) {
        (void)axis_ctx[axis].huart->Instance->DR;
    }

    HAL_UART_Transmit(axis_ctx[axis].huart, buf, 8, 10);
}

/**
  * @brief  读取寄存器（移植自 Arduino 库, 增加重试机制）
  *
  *         TMC2209 UART 为单线半双工, 读取时先发 4 字节请求帧,
  *         驱动器返回 8 字节响应帧. 通信可能因时序抖动失败,
  *         因此加入 MAX_READ_RETRIES 次重试提高可靠性.
  *
  * @return 读取到的 32 位数据; 失败返回 TMC_ERR_* 错误码
  */
static uint32_t TMC2209_ReadRegisterInternal(Axis_TypeDef axis, uint8_t reg)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[8] = {0};

    tx_buf[0] = SYNC;
    tx_buf[1] = axis_ctx[axis].serial_address;
    tx_buf[2] = (reg & 0x7F) | (RW_READ << 7);
    tx_buf[3] = TMC2209_CRC8(tx_buf, 3);

    for (uint8_t retry = 0; retry < MAX_READ_RETRIES; retry++) {
        /* 清空接收缓冲区, 避免上次残留数据干扰 */
        while (__HAL_UART_GET_FLAG(axis_ctx[axis].huart, UART_FLAG_RXNE)) {
            (void)axis_ctx[axis].huart->Instance->DR;
        }

        /* 发送读取请求 (4 字节) */
        if (HAL_UART_Transmit(axis_ctx[axis].huart, tx_buf, 4, 10) != HAL_OK) {
            tmc_error_count++;
            HAL_Delay(READ_RETRY_DELAY_MS);
            continue;
        }

        /* 等待 TMC2209 处理并返回响应 */
        HAL_Delay(3);

        /* 接收响应帧 (8 字节) */
        if (HAL_UART_Receive(axis_ctx[axis].huart, rx_buf, 8, 20) != HAL_OK) {
            tmc_error_count++;
            HAL_Delay(READ_RETRY_DELAY_MS);
            continue;
        }

        /* 验证响应帧: 同步字节 + 地址 + 寄存器 + CRC */
        uint8_t calc_crc = TMC2209_CRC8(rx_buf, 7);
        if (rx_buf[0] != SYNC || rx_buf[1] != READ_REPLY_SERIAL_ADDRESS ||
            rx_buf[2] != ((reg & 0x7F) | (RW_READ << 7)) || rx_buf[7] != calc_crc) {
            tmc_error_count++;
            HAL_Delay(READ_RETRY_DELAY_MS);
            continue;
        }

        /* 校验通过, 返回数据 */
        return TMC2209_bytesToUint32(&rx_buf[3]);
    }

    /* 所有重试均失败 */
    return TMC_ERR_CRC;
}

/**
  * @brief  设置操作模式为 UART 串行模式
  *
  *         GCONF 寄存器配置:
  *         bit6  PDN_DISABLE=1     关闭 PDN 硬件引脚, 启用 UART 通信
  *         bit7  MSTEP_REG_SELECT=1 微步分辨率由 CHOPCONF.MRES 决定
  *         bit8  MULTISTEP_FILT=1   多步滤波, STEP 脉冲更稳定
  *         (bit2 EN_SPREADCYCLE=0   默认 StealthChop 静音模式)
  */
static void TMC2209_setOperationModeToSerial(Axis_TypeDef axis)
{
    axis_ctx[axis].global_config = 0;
    axis_ctx[axis].global_config |= (1 << 6);  // pdn_disable
    axis_ctx[axis].global_config |= (1 << 7);  // mstep_reg_select
    axis_ctx[axis].global_config |= (1 << 8);  // multistep_filt
    TMC2209_writeStoredGlobalConfig(axis);
}

/**
  * @brief  将寄存器恢复为出厂默认值 (工具函数, 当前 InitAxis 不调用)
  *
  *         保留供调试或需要重置 TMC2209 时使用.
  *         InitAxis 已逐寄存器配置, 无需先恢复默认值.
  */
static void TMC2209_setRegistersToDefaults(Axis_TypeDef axis)
{
    /* 设置电流默认值 */
    axis_ctx[axis].driver_current = 0;
    axis_ctx[axis].driver_current |= (IHOLD_DEFAULT << 0);
    axis_ctx[axis].driver_current |= (IRUN_DEFAULT << 8);
    axis_ctx[axis].driver_current |= (IHOLDDELAY_DEFAULT << 16);
    TMC2209_WriteRegister(axis, TMC_REG_IHOLD_IRUN, axis_ctx[axis].driver_current);

    /* 设置斩波器配置默认值 */
    axis_ctx[axis].chopper_config = CHOPPER_CONFIG_DEFAULT;
    axis_ctx[axis].chopper_config |= (TBL_DEFAULT << 15);
    axis_ctx[axis].chopper_config |= (HEND_DEFAULT << 7);
    axis_ctx[axis].chopper_config |= (HSTART_DEFAULT << 4);
    axis_ctx[axis].chopper_config |= (TOFF_DEFAULT << 0);
    TMC2209_WriteRegister(axis, TMC_REG_CHOPCONF, axis_ctx[axis].chopper_config);

    /* 设置PWM配置默认值 */
    axis_ctx[axis].pwm_config = PWM_CONFIG_DEFAULT;
    TMC2209_WriteRegister(axis, TMC_REG_PWMCONF, axis_ctx[axis].pwm_config);

    /* 设置CoolStep配置默认值 */
    axis_ctx[axis].cool_config = 0;
    TMC2209_WriteRegister(axis, TMC_REG_COOLCONF, axis_ctx[axis].cool_config);

    /* 其他寄存器默认值 */
    TMC2209_WriteRegister(axis, TMC_REG_TPOWERDOWN, 20);
    TMC2209_WriteRegister(axis, TMC_REG_TPWMTHRS, 0);
    TMC2209_WriteRegister(axis, TMC_REG_VACTUAL, 0);
    TMC2209_WriteRegister(axis, TMC_REG_TCOOLTHRS, 0);
    TMC2209_WriteRegister(axis, TMC_REG_SGTHRS, 0);
}

/**
  * @brief  写入存储的全局配置
  */
static void TMC2209_writeStoredGlobalConfig(Axis_TypeDef axis)
{
    TMC2209_WriteRegister(axis, TMC_REG_GCONF, axis_ctx[axis].global_config);
}

/* ========== 以下函数来自 Arduino 库移植，当前未使用但保留供扩展 ========== */
#if 0
static uint8_t TMC2209_percentToCurrentSetting(uint8_t percent)
{
    if (percent > PERCENT_MAX) percent = PERCENT_MAX;
    return (uint8_t)((uint32_t)percent * CURRENT_SETTING_MAX / PERCENT_MAX);
}

static uint8_t TMC2209_percentToHoldDelaySetting(uint8_t percent)
{
    if (percent > PERCENT_MAX) percent = PERCENT_MAX;
    return (uint8_t)((uint32_t)percent * HOLD_DELAY_MAX / PERCENT_MAX);
}
#endif

void TMC2209_UART_Write(Axis_TypeDef axis, uint8_t reg, uint32_t data)
{
    TMC2209_WriteRegister(axis, reg, data);
}

/**
  * @brief  设置目标角速度 (带斜坡加减速)
  *
  *         由 task.c 的 PID 控制器调用:
  *           PID 输出 → 本函数设为目标速度 → SpeedRamp_Update 斜坡逼近
  *
  * @param  axis   轴号
  * @param  speed  目标角速度 (°/s), 正=正转, 负=反转
  * @param  accel  加速度 (°/s²), 0=保持上次值
  */
void TMC2209_SetAngleSpeed(Axis_TypeDef axis, float speed, float accel)
{
    if (!axis_ctx[axis].motor->enabled && speed != 0) {
        TMC2209_Enable(axis);
    }
    axis_ctx[axis].target_speed = speed;
    if (accel > 0) {
        axis_ctx[axis].accel = accel;
    }
}

/* task_direct_control: 绘图任务直接控制电机时置1, 斜坡更新跳过 */
extern volatile uint8_t task_direct_control;

/**
  * @brief  速度斜坡更新 — 在 TIM1 中断中每 2ms 调用
  *
  *         将当前速度向目标速度逼近 (加减速斜坡), 并输出到电机.
  *         优化: 仅在方向或速度实际变化时才操作 GPIO/定时器寄存器,
  *         避免每个 tick 都做冗余写入 (原代码每 2ms 无条件写 GPIO+TIM).
  *
  *         调用链: task.c PID → TMC2209_SetAngleSpeed(设目标) →
  *                 本函数(斜坡逼近) → TMC2209_SetAngularVelocity → TIM3 PWM
  *
  * @note   当 task_direct_control=1 时 (绘图任务直接控电机), 本函数直接返回,
  *         由 task.c 的 draw_sine_wave() 直接控制 VACTUAL + TIM3.
  */
void TMC2209_SpeedRamp_Update(void)
{
    /* 绘图任务直接控制电机时, 跳过斜坡更新 */
    if (task_direct_control) return;

    for (int i = 0; i < 2; i++) {
        AxisCtx_t *a = &axis_ctx[i];
        float current = a->current_speed;
        float target  = a->target_speed;
        float accel   = a->accel;

        /* 目标和当前都为零 → 确保电机停止, 跳过斜坡计算 */
        if (current == 0.0f && target == 0.0f) {
            if (a->motor->pulse_freq != 0) {
                TMC2209_Stop((Axis_TypeDef)i);
            }
            continue;
        }

        /* 加减速斜坡: 按 accel × dt 步进逼近目标速度 */
        float step = accel * SPEED_RAMP_DT;
        if (current < target) {
            current += step;
            if (current > target) current = target;
        } else if (current > target) {
            current -= step;
            if (current < target) current = target;
        }
        a->current_speed = current;

        /* 方向控制: 仅在方向变化时写 GPIO (减少中断内 GPIO 操作) */
        int8_t dir = (current >= 0.0f) ? 0 : 1;
        if (dir != a->last_dir) {
            TMC2209_SetDirection((Axis_TypeDef)i, dir);
            a->last_dir = dir;
        }

        /* 速度输出: 取绝对值设置角速度 */
        float abs_speed = (current >= 0.0f) ? current : -current;
        TMC2209_SetAngularVelocity((Axis_TypeDef)i, abs_speed);
    }
}
