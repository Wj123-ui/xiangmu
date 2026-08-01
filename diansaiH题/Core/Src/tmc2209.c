#include "tmc2209.h"

/*
 * TMC2209 驱动实现。
 * UART 使用 TMC2209 的单线帧格式完成寄存器配置/诊断，定时器 CH1 产生 STEP，
 * 速度斜坡由 2 ms 周期任务调用。所有通信失败都会累计到状态结构体中。
 */

/* TMC2209 UART 帧及寄存器地址。 */
#define TMC_SYNC             0x05U
#define TMC_WRITE_BIT        0x80U
#define TMC_REPLY_ADDRESS    0xFFU
#define TMC_REG_GCONF        0x00U
#define TMC_REG_IFCNT        0x02U
#define TMC_REG_IHOLD_IRUN   0x10U
#define TMC_REG_TPOWERDOWN   0x11U
#define TMC_REG_TPWMTHRS     0x13U
#define TMC_REG_TCOOLTHRS    0x14U
#define TMC_REG_VACTUAL      0x22U
#define TMC_REG_SGTHRS       0x40U
#define TMC_REG_CHOPCONF     0x6CU
#define TMC_REG_COOLCONF     0x6DU
#define TMC_REG_PWMCONF      0x70U

/* STEP 定时器等效时钟及速度斜坡参数。 */
#define TMC_STEP_CLOCK_HZ    1000000U
#define TMC_STEP_FREQ_MIN    16U
#define TMC_STEP_FREQ_MAX    (TMC_STEP_CLOCK_HZ / 2U)
#define TMC_SPEED_RAMP_DT    0.002f
#define TMC_DEFAULT_ACCEL    3000.0f
#define TMC_READ_RETRIES      5U
#define TMC_READ_RETRY_MS    20U
/* 方向改变后至少等待 2 ms，再允许发出新的 STEP 脉冲。 */
#define TMC_POSITION_DIR_SETUP_MS 2U

#define TMC_GCONF_SERIAL     ((1UL << 6) | (1UL << 7) | (1UL << 8))
#define TMC_PWMCONF_DEFAULT  0xC10D0024UL
#define TMC_PWMCONF_RUN      ((1UL << 18) | (4UL << 16) | 200UL)

/* MRES 编码到每整步微步数的查表。 */
static const uint16_t tmc_microsteps[] = {
    256U, 128U, 64U, 32U, 16U, 8U, 4U, 2U, 1U
};

static UART_HandleTypeDef *tmc_uart;       /* TMC2209 配置和诊断所用 UART。 */
static TIM_HandleTypeDef *tmc_timer;       /* 产生 STEP 脉冲的 PWM 定时器。 */
static TMC2209_State tmc_state;            /* 对外提供的状态快照。 */
static volatile float tmc_current_speed;   /* 斜坡当前角速度，单位 °/s。 */
static volatile float tmc_target_speed;    /* 斜坡目标角速度，单位 °/s。 */
static volatile float tmc_acceleration;    /* 斜坡加速度，单位 °/s²。 */
static volatile bool tmc_direct_control;   /* 阻塞式 Step 期间禁止斜坡任务改写速度。 */
static volatile bool tmc_fault_latched;    /* DIAG 故障锁存后禁止再次输出脉冲。 */
/* 非阻塞相对角度运动的剩余运行时间；每 1 ms 由 Timer.c 递减。 */
static volatile bool tmc_angle_move_active;
static volatile uint16_t tmc_angle_move_remaining_ms;
static int8_t tmc_last_direction;          /* 最近一次 DIR 电平，避免重复切换。 */
static uint8_t tmc_hold_current;           /* 最近一次保持电流，供 SetRunCurrent 使用。 */

/*
 * 非阻塞绝对位置状态。current_steps 只在 TIM9 的“一个 PWM 高电平已经结束”
 * 回调中更新，因此其值对应实际完整送出的 STEP 脉冲，而不是按时间估算的角度。
 */
static volatile bool tmc_position_reference_valid;
static volatile bool tmc_position_move_active;
static volatile bool tmc_position_pulse_running;
static volatile bool tmc_position_abort_requested;
static volatile bool tmc_position_soft_limits_enabled;
static volatile int32_t tmc_position_steps;
static volatile int32_t tmc_position_target_steps;
static volatile int32_t tmc_position_min_steps;
static volatile int32_t tmc_position_max_steps;
static volatile uint32_t tmc_position_frequency_hz;
static volatile uint8_t tmc_position_dir_settle_ms;
static int8_t tmc_position_step_sign;

/* 根据 MRES 编码构造 CHOPCONF，设置电流斩波和微步模式。 */
static uint32_t TMC2209_BuildChopConf(uint8_t mres)
{
    return (4UL << 0) | (5UL << 4) | (2UL << 15) |
           (1UL << 17) | ((uint32_t)mres << 24) | (1UL << 28);
}

/* 清空非阻塞速度斜坡的当前值和目标值。 */
static void TMC2209_ResetRamp(void)
{
    tmc_current_speed = 0.0f;
    tmc_target_speed = 0.0f;
}

/* 把外部输入频率限制在定时器和驱动器允许的范围内。 */
static uint32_t TMC2209_NormalizeFrequency(uint32_t frequency_hz)
{
    if (frequency_hz == 0U) return 0U;
    if (frequency_hz < TMC_STEP_FREQ_MIN) return TMC_STEP_FREQ_MIN;
    if (frequency_hz > TMC_STEP_FREQ_MAX) return TMC_STEP_FREQ_MAX;
    return frequency_hz;
}

/* TMC2209 UART 帧 CRC8，按芯片规定的多项式 0x07 计算。 */
static uint8_t TMC2209_CRC8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;
    uint8_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++) {
        uint8_t value = data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if (((crc >> 7) ^ (value & 1U)) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc <<= 1;
            }
            value >>= 1;
        }
    }
    return crc;
}

/* 清掉半双工 UART 中可能残留的回复字节，避免污染下一次事务。 */
static void TMC2209_FlushRx(void)
{
    while (__HAL_UART_GET_FLAG(tmc_uart, UART_FLAG_RXNE) != RESET) {
        (void)tmc_uart->Instance->DR;
    }
}

/* 发送 8 字节写寄存器帧：同步字、地址、寄存器/写位、32 位数据和 CRC。 */
void TMC2209_WriteRegister(uint8_t reg, uint32_t value)
{
    uint8_t packet[8];

    if (tmc_uart == NULL) return;
    packet[0] = TMC_SYNC;
    packet[1] = 0U;
    packet[2] = (uint8_t)((reg & 0x7FU) | TMC_WRITE_BIT);
    packet[3] = (uint8_t)(value >> 24);
    packet[4] = (uint8_t)(value >> 16);
    packet[5] = (uint8_t)(value >> 8);
    packet[6] = (uint8_t)value;
    packet[7] = TMC2209_CRC8(packet, 7U);
    TMC2209_FlushRx();
    if (HAL_UART_Transmit(tmc_uart, packet, sizeof(packet), 20U) != HAL_OK) {
        tmc_state.communication_errors++;
    }
}

/*
 * 发送 4 字节读请求并等待 8 字节回复；最多重试 TMC_READ_RETRIES 次，
 * 回复的同步字、地址、寄存器号和 CRC 均正确时才把数据交给调用者。
 */
bool TMC2209_ReadRegister(uint8_t reg, uint32_t *value)
{
    uint8_t request[4];
    uint8_t reply[8];
    uint8_t retry;

    if ((tmc_uart == NULL) || (value == NULL)) return false;
    request[0] = TMC_SYNC;
    request[1] = 0U;
    request[2] = reg & 0x7FU;
    request[3] = TMC2209_CRC8(request, 3U);

    for (retry = 0U; retry < TMC_READ_RETRIES; retry++) {
        TMC2209_FlushRx();
        if (HAL_UART_Transmit(tmc_uart, request, sizeof(request), 10U) != HAL_OK) {
            tmc_state.communication_errors++;
            HAL_Delay(TMC_READ_RETRY_MS);
            continue;
        }

        HAL_Delay(3U);
        if (HAL_UART_Receive(tmc_uart, reply, sizeof(reply), 20U) != HAL_OK) {
            tmc_state.communication_errors++;
            HAL_Delay(TMC_READ_RETRY_MS);
            continue;
        }

        if ((reply[0] == TMC_SYNC) &&
            (reply[1] == TMC_REPLY_ADDRESS) &&
            (reply[2] == (reg & 0x7FU)) &&
            (reply[7] == TMC2209_CRC8(reply, 7U))) {
            *value = ((uint32_t)reply[3] << 24) |
                     ((uint32_t)reply[4] << 16) |
                     ((uint32_t)reply[5] << 8) |
                     (uint32_t)reply[6];
            return true;
        }

        tmc_state.communication_errors++;
        HAL_Delay(TMC_READ_RETRY_MS);
    }

    return false;
}

/* 把保持电流、运行电流和保持延时打包到 IHOLD_IRUN 寄存器。 */
void TMC2209_SetCurrent(uint8_t hold_current, uint8_t run_current,
                        uint8_t hold_delay)
{
    uint32_t value = ((uint32_t)(hold_delay & 0x0FU) << 16) |
                     ((uint32_t)(run_current & 0x1FU) << 8) |
                     (uint32_t)(hold_current & 0x1FU);
    TMC2209_WriteRegister(TMC_REG_IHOLD_IRUN, value);
    tmc_hold_current = hold_current & 0x1FU;
}

/* 修改运行电流时复用当前保持电流，保持延时固定为 4。 */
void TMC2209_SetRunCurrent(uint8_t run_current)
{
    TMC2209_SetCurrent(tmc_hold_current, run_current, 4U);
}

/* 写入 CHOPCONF 的 MRES 字段，并同步更新软件状态。 */
void TMC2209_SetMicrostep(uint8_t mres)
{
    if (mres > TMC2209_MRES_FULL) mres = TMC2209_MRES_16;
    /* 微步数改变后“步数→角度”的坐标比例已改变，旧位置参考不再可信。 */
    tmc_position_reference_valid = false;
    TMC2209_WriteRegister(TMC_REG_CHOPCONF,
                          TMC2209_BuildChopConf(mres));
    tmc_state.microstep_resolution = mres;
}

/* 上电后使用的寄存器默认值，与参考工程的初始化顺序保持一致。 */
static void TMC2209_WriteDefaultRegisters(void)
{
    TMC2209_WriteRegister(TMC_REG_IHOLD_IRUN, 0x00011F10UL);
    TMC2209_WriteRegister(TMC_REG_CHOPCONF, 0x10010053UL);
    TMC2209_WriteRegister(TMC_REG_PWMCONF, TMC_PWMCONF_DEFAULT);
    TMC2209_WriteRegister(TMC_REG_COOLCONF, 0U);
    TMC2209_WriteRegister(TMC_REG_TPOWERDOWN, 20U);
    TMC2209_WriteRegister(TMC_REG_TPWMTHRS, 0U);
    TMC2209_WriteRegister(TMC_REG_VACTUAL, 0U);
    TMC2209_WriteRegister(TMC_REG_TCOOLTHRS, 0U);
    TMC2209_WriteRegister(TMC_REG_SGTHRS, 0U);
}

/*
 * 初始化 TMC2209：启动 STEP PWM、关闭驱动、尝试读取通信计数器，
 * 再按固定顺序写入工作模式、电流、微步和斩波参数，最后复读通信计数器。
 */
bool TMC2209_Init(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim,
                  uint8_t default_mres)
{
    uint32_t ifcnt_before = 0U;
    uint32_t ifcnt_after = 0U;
    bool before_ok;
    bool after_ok;
    bool pwm_ok;

    tmc_uart = huart;
    tmc_timer = htim;
    if (default_mres > TMC2209_MRES_FULL) {
        default_mres = TMC2209_MRES_16;
    }

    tmc_state.enabled = false;
    tmc_state.stalled = false;
    tmc_state.step_frequency_hz = 0U;
    tmc_state.communication_errors = 0U;
    tmc_state.microstep_resolution = default_mres;
    TMC2209_ResetRamp();
    tmc_acceleration = TMC_DEFAULT_ACCEL;
    tmc_direct_control = false;
    tmc_fault_latched = false;
    tmc_angle_move_active = false;
    tmc_angle_move_remaining_ms = 0U;
    tmc_last_direction = -1;
    tmc_hold_current = 8U;
    tmc_position_reference_valid = false;
    tmc_position_move_active = false;
    tmc_position_pulse_running = false;
    tmc_position_abort_requested = false;
    tmc_position_soft_limits_enabled = false;
    tmc_position_steps = 0;
    tmc_position_target_steps = 0;
    tmc_position_min_steps = 0;
    tmc_position_max_steps = 0;
    tmc_position_frequency_hz = 0U;
    tmc_position_dir_settle_ms = 0U;
    tmc_position_step_sign = 1;

    pwm_ok = (tmc_timer != NULL) &&
             (HAL_TIM_PWM_Start(tmc_timer, TIM_CHANNEL_1) == HAL_OK);
    if (!pwm_ok) {
        tmc_state.communication_errors++;
    }
    if (tmc_timer != NULL) {
        __HAL_TIM_SET_COMPARE(tmc_timer, TIM_CHANNEL_1, 0U);
        __HAL_TIM_DISABLE_IT(tmc_timer, TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(tmc_timer, TIM_FLAG_CC1);
    }
    TMC2209_Disable();
    TMC2209_SetDirection(0U);

    before_ok = TMC2209_ReadRegister(TMC_REG_IFCNT, &ifcnt_before);

    /* 保持与参考工程 miaozhun/control.c 相同的寄存器顺序和最终配置值。 */
    TMC2209_WriteRegister(TMC_REG_GCONF, TMC_GCONF_SERIAL);
    TMC2209_WriteDefaultRegisters();
    TMC2209_WriteRegister(TMC_REG_GCONF, TMC_GCONF_SERIAL);
    HAL_Delay(10U);

    TMC2209_WriteRegister(TMC_REG_PWMCONF, TMC_PWMCONF_RUN);
    HAL_Delay(5U);
    TMC2209_SetCurrent(8U, 20U, 4U);
    HAL_Delay(5U);
    TMC2209_SetMicrostep(default_mres);
    HAL_Delay(5U);
    TMC2209_WriteRegister(TMC_REG_TPWMTHRS, 500U);
    HAL_Delay(5U);
    TMC2209_WriteRegister(TMC_REG_CHOPCONF,
                          TMC2209_BuildChopConf(default_mres));
    HAL_Delay(5U);

    after_ok = TMC2209_ReadRegister(TMC_REG_IFCNT, &ifcnt_after);
    return pwm_ok && after_ok &&
           (!before_ok || (ifcnt_after != ifcnt_before));
}

/* 通过 EN 引脚开启功率级；故障锁存时保持关闭。 */
void TMC2209_Enable(void)
{
    if (tmc_fault_latched) return;

    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);
    if (tmc_fault_latched) {
        HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);
        tmc_state.enabled = false;
        return;
    }
    tmc_state.enabled = true;
}

/* 停止斜坡和 STEP，并通过 EN 引脚关闭功率级。 */
void TMC2209_Disable(void)
{
    TMC2209_ResetRamp();
    tmc_direct_control = false;
    tmc_angle_move_active = false;
    tmc_angle_move_remaining_ms = 0U;
    /* 断使能后电机可能被外力移动，因此软件绝对位置必须重新建立。 */
    tmc_position_move_active = false;
    tmc_position_pulse_running = false;
    tmc_position_abort_requested = false;
    tmc_position_dir_settle_ms = 0U;
    tmc_position_reference_valid = false;
    if (tmc_timer != NULL) {
        __HAL_TIM_DISABLE_IT(tmc_timer, TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(tmc_timer, TIM_FLAG_CC1);
    }
    TMC2209_Stop();
    HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);
    tmc_state.enabled = false;
}

/* 设置方向引脚并记录软件方向，后续速度输出使用该方向。 */
void TMC2209_SetDirection(uint8_t dir)
{
    HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin,
                      dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
    tmc_last_direction = dir ? 1 : 0;
}

/*
 * 将目标频率转换成 PWM 周期和占空比。直接写定时器寄存器可在不阻塞的情况下
 * 改变 STEP 频率；故障锁存时强制 CCR1 为 0。
 */
void TMC2209_SetSpeed(uint32_t frequency_hz)
{
    uint32_t period;
    TIM_TypeDef *tim;

    if (tmc_timer == NULL) return;
    frequency_hz = TMC2209_NormalizeFrequency(frequency_hz);

    if (tmc_fault_latched && (frequency_hz != 0U)) {
        tmc_timer->Instance->CCR1 = 0U;
        tmc_state.step_frequency_hz = 0U;
        return;
    }
    if (frequency_hz == tmc_state.step_frequency_hz) return;

    tim = tmc_timer->Instance;
    if (frequency_hz == 0U) {
        tim->CCR1 = 0U;
        tmc_state.step_frequency_hz = 0U;
        return;
    }

    period = TMC_STEP_CLOCK_HZ / frequency_hz;
    if (period < 2U) period = 2U;
    if (period > 65536U) period = 65536U;

    tim->ARR = period - 1U;
    tim->CCR1 = period / 2U;
    tim->EGR = TIM_EGR_UG;
    tmc_state.step_frequency_hz = frequency_hz;

    if (tmc_fault_latched) {
        tim->CCR1 = 0U;
        tmc_state.step_frequency_hz = 0U;
    }
}

/* 根据当前微步和减速比计算每转 STEP 脉冲数。 */
uint32_t TMC2209_GetPulsesPerRev(void)
{
    uint8_t mres = tmc_state.microstep_resolution;

    if (mres > TMC2209_MRES_FULL) {
        mres = TMC2209_MRES_16;
    }
    return TMC2209_FULL_STEPS_PER_REV *
           (uint32_t)tmc_microsteps[mres] *
           TMC2209_REDUCTION_RATIO;
}

/* STEP 脉冲数到角度的线性换算。 */
float TMC2209_StepsToDegrees(int32_t steps)
{
    return (float)steps * 360.0f /
           (float)TMC2209_GetPulsesPerRev();
}

/* 角度到 STEP 脉冲数的线性换算，结果向零截断为整数。 */
int32_t TMC2209_DegreesToSteps(float degrees)
{
    return (int32_t)(degrees *
                     (float)TMC2209_GetPulsesPerRev() /
                     360.0f);
}

/* 绝对位置目标取最接近的微步，避免正负目标都因向零截断而产生固定偏差。 */
static int32_t TMC2209_RoundDegreesToSteps(float degrees)
{
    float steps = degrees * (float)TMC2209_GetPulsesPerRev() / 360.0f;

    return (steps >= 0.0f) ? (int32_t)(steps + 0.5f)
                            : (int32_t)(steps - 0.5f);
}

static uint32_t TMC2209_AngularVelocityToFrequency(float deg_per_sec)
{
    /* 角速度绝对值乘以每转脉冲数，得到 STEP 频率。 */
    float speed = (deg_per_sec < 0.0f) ? -deg_per_sec : deg_per_sec;
    uint32_t frequency_hz =
        (uint32_t)(speed *
                   (float)TMC2209_GetPulsesPerRev() /
                   360.0f);

    if ((speed > 0.0f) && (frequency_hz == 0U)) {
        frequency_hz = 1U;
    }
    return frequency_hz;
}

/* 立即设置角速度大小；方向需由调用者提前通过 SetDirection 设置。 */
void TMC2209_SetAngularVelocity(float deg_per_sec)
{
    TMC2209_SetSpeed(TMC2209_AngularVelocityToFrequency(deg_per_sec));
}

/* 只清零当前 PWM 输出，不修改斜坡目标值。 */
void TMC2209_Stop(void)
{
    TMC2209_SetSpeed(0U);
}

/*
 * 阻塞式定步运行：使能驱动、设置方向和频率，按脉冲数计算运行时间，
 * 到时停止 STEP。执行期间 tmc_direct_control 防止 2 ms 斜坡任务介入。
 */
void TMC2209_Step(int32_t steps, uint32_t frequency_hz)
{
    uint32_t pulse_count;
    uint32_t run_time_ms;

    /* 阻塞式定步不属于绝对位置模式，先停止旧任务并让位置参考失效。 */
    TMC2209_CancelPositionMove();
    tmc_position_reference_valid = false;
    /* 阻塞式定步若被其他模块调用，先取消可能残留的 TASK3 异步角度命令。 */
    if (tmc_angle_move_active) {
        TMC2209_CancelAngleMove();
    }

    frequency_hz = TMC2209_NormalizeFrequency(frequency_hz);
    if (tmc_fault_latched ||
        (steps == 0) || (frequency_hz == 0U)) return;

    tmc_direct_control = true;
    TMC2209_ResetRamp();
    pulse_count = (steps < 0) ? (uint32_t)(-(int64_t)steps)
                              : (uint32_t)steps;

    if (!tmc_state.enabled) {
        TMC2209_Enable();
        HAL_Delay(10U);
    }
    if (tmc_fault_latched) {
        tmc_direct_control = false;
        return;
    }

    TMC2209_SetDirection((steps < 0) ? 1U : 0U);
    HAL_Delay(2U);
    if (tmc_fault_latched) {
        tmc_direct_control = false;
        return;
    }
    TMC2209_SetSpeed(frequency_hz);

    run_time_ms = (uint32_t)(((uint64_t)pulse_count * 1000ULL) /
                             frequency_hz) + 1U;
    HAL_Delay(run_time_ms);

    TMC2209_Stop();
    tmc_direct_control = false;
}

/* 阻塞式角度运动封装：先把角度换算成脉冲，再调用定步函数。 */
void TMC2209_RotateDegrees(float degrees, float deg_per_sec)
{
    int32_t steps = TMC2209_DegreesToSteps(degrees);
    uint32_t frequency_hz =
        TMC2209_AngularVelocityToFrequency(deg_per_sec);

    TMC2209_Step(steps, frequency_hz);
}

/*
 * 取消当前非阻塞角度运动：停止 STEP，但不拉高 EN，因此电机继续保持力矩。
 * TASK3 丢球、到达死区或切换目标时都应调用本函数，而不是调用阻塞式 RotateDegrees。
 */
void TMC2209_CancelAngleMove(void)
{
    tmc_angle_move_active = false;
    tmc_angle_move_remaining_ms = 0U;
    tmc_direct_control = false;
    TMC2209_ResetRamp();
    TMC2209_Stop();
}

/*
 * 非阻塞相对角度运动。
 * 与阻塞版 TMC2209_RotateDegrees 使用相同的“脉冲数 / 频率”时间换算，
 * 但把等待过程交给 Timer.c 每 1 ms 的 TMC2209_AngleMoveTick1ms 完成。
 * 角度会被微步分辨率量化；当前 1/16 微步、1:1 传动时最小约 0.1125°。
 */
void TMC2209_RotateDegreesAsync(float degrees, float deg_per_sec)
{
    int32_t steps;
    uint32_t pulse_count;
    uint32_t frequency_hz;
    uint64_t run_time_ms;

    /* 相对角度模式会改变轴位置，但不记录逐脉冲坐标，故不能与绝对位置混用。 */
    TMC2209_CancelPositionMove();
    tmc_position_reference_valid = false;

    steps = TMC2209_DegreesToSteps(degrees);
    /* 小于一个微步的非零角度向远离 0 的方向取为 1 微步，避免闭环卡在死区外。 */
    if ((steps == 0) && (degrees != 0.0f)) {
        steps = (degrees < 0.0f) ? -1 : 1;
    }
    frequency_hz = TMC2209_AngularVelocityToFrequency(deg_per_sec);
    if (tmc_fault_latched || (steps == 0) || (frequency_hz == 0U)) {
        TMC2209_CancelAngleMove();
        return;
    }

    pulse_count = (steps < 0) ? (uint32_t)(-(int64_t)steps)
                              : (uint32_t)steps;
    /* 向上取整到毫秒，确保本次脉冲数不会因为提前停止而少发。 */
    run_time_ms = (((uint64_t)pulse_count * 1000ULL) + frequency_hz - 1U) /
                  frequency_hz;
    if (run_time_ms == 0U) {
        run_time_ms = 1U;
    }
    if (run_time_ms > 0xFFFFULL) {
        run_time_ms = 0xFFFFULL;
    }

    /* 新角度命令覆盖上一条未完成命令，避免视觉闭环积压旧动作。 */
    TMC2209_Stop();
    TMC2209_ResetRamp();
    tmc_direct_control = true;
    tmc_angle_move_active = true;
    tmc_angle_move_remaining_ms = (uint16_t)run_time_ms;

    if (!tmc_state.enabled) {
        TMC2209_Enable();
    }
    if (tmc_fault_latched || (!tmc_state.enabled)) {
        TMC2209_CancelAngleMove();
        return;
    }

    TMC2209_SetDirection((steps < 0) ? 1U : 0U);
    TMC2209_SetSpeed(frequency_hz);
}

/* TIM2 每 1 ms 调用一次：非阻塞角度运动到时后停止 PWM。 */
void TMC2209_AngleMoveTick1ms(void)
{
    if (!tmc_angle_move_active) {
        return;
    }

    if (tmc_fault_latched) {
        tmc_angle_move_active = false;
        tmc_angle_move_remaining_ms = 0U;
        tmc_direct_control = false;
        return;
    }

    if (tmc_angle_move_remaining_ms > 1U) {
        tmc_angle_move_remaining_ms--;
        return;
    }

    tmc_angle_move_active = false;
    tmc_angle_move_remaining_ms = 0U;
    TMC2209_Stop();
    tmc_direct_control = false;
}

/*-------------------------- 非阻塞绝对位置模式 --------------------------*/

/* 停止位置模式的 PWM/CC1 中断，但保留 EN，电机继续提供保持力矩。 */
static void TMC2209_PositionFinish(void)
{
    if (tmc_timer != NULL) {
        __HAL_TIM_DISABLE_IT(tmc_timer, TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(tmc_timer, TIM_FLAG_CC1);
    }
    TMC2209_SetSpeed(0U);
    tmc_position_move_active = false;
    tmc_position_pulse_running = false;
    tmc_position_abort_requested = false;
    tmc_position_dir_settle_ms = 0U;
    tmc_direct_control = false;
    TMC2209_ResetRamp();
}

/*
 * 在没有 STEP 高电平输出时切换 DIR，并交给 1 ms 调度器等待建立时间。
 * 该函数只能在“尚未发脉冲”或“刚刚在 CC1 回调结束一个完整脉冲”后调用，
 * 这样不会把正在进行的脉冲截断，也不会漏记软件位置。
 */
static void TMC2209_PositionBeginDirectionSettle(void)
{
    int32_t remain = tmc_position_target_steps - tmc_position_steps;

    if (remain == 0) {
        TMC2209_PositionFinish();
        return;
    }

    if (tmc_timer != NULL) {
        __HAL_TIM_DISABLE_IT(tmc_timer, TIM_IT_CC1);
        __HAL_TIM_CLEAR_FLAG(tmc_timer, TIM_FLAG_CC1);
    }
    TMC2209_SetSpeed(0U);
    tmc_position_pulse_running = false;
    tmc_position_step_sign = (remain < 0) ? -1 : 1;
    TMC2209_SetDirection((remain < 0) ? 1U : 0U);
    tmc_position_dir_settle_ms = TMC_POSITION_DIR_SETUP_MS;
}

void TMC2209_SetCurrentPositionSteps(int32_t steps)
{
    /* TASK1/TASK3/TASK4 启动时由当前机械姿态定义软件零点；先安全停止任何旧运动。 */
    TMC2209_CancelAngleMove();
    TMC2209_CancelPositionMove();
    tmc_position_steps = steps;
    tmc_position_target_steps = steps;
    tmc_position_reference_valid = true;
}

int32_t TMC2209_GetCurrentPositionSteps(void)
{
    return tmc_position_steps;
}

float TMC2209_GetCurrentPositionDegrees(void)
{
    return TMC2209_StepsToDegrees(tmc_position_steps);
}

bool TMC2209_HasPositionReference(void)
{
    return tmc_position_reference_valid;
}

void TMC2209_SetSoftLimitsDegrees(float minimum_deg, float maximum_deg)
{
    int32_t minimum_steps = TMC2209_RoundDegreesToSteps(minimum_deg);
    int32_t maximum_steps = TMC2209_RoundDegreesToSteps(maximum_deg);
    int32_t temporary;

    if (minimum_steps > maximum_steps) {
        temporary = minimum_steps;
        minimum_steps = maximum_steps;
        maximum_steps = temporary;
    }
    tmc_position_min_steps = minimum_steps;
    tmc_position_max_steps = maximum_steps;
    tmc_position_soft_limits_enabled = true;
}

bool TMC2209_MoveToDegreesAsync(float target_deg, float deg_per_sec)
{
    int32_t target_steps;
    uint32_t frequency_hz;

    if ((tmc_timer == NULL) || tmc_fault_latched ||
        (!tmc_position_reference_valid)) {
        return false;
    }

    frequency_hz = TMC2209_AngularVelocityToFrequency(deg_per_sec);
    frequency_hz = TMC2209_NormalizeFrequency(frequency_hz);
    if (frequency_hz == 0U) {
        return false;
    }

    target_steps = TMC2209_RoundDegreesToSteps(target_deg);
    if (tmc_position_soft_limits_enabled) {
        if (target_steps < tmc_position_min_steps) {
            target_steps = tmc_position_min_steps;
        } else if (target_steps > tmc_position_max_steps) {
            target_steps = tmc_position_max_steps;
        }
    }

    /* 目标更新可直接覆盖旧目标，等价于 AccelStepper 的 moveTo(newTarget)。 */
    tmc_position_target_steps = target_steps;
    tmc_position_frequency_hz = frequency_hz;
    tmc_position_abort_requested = false;

    /* 若前一条是 TASK3 的相对角度命令，先停止它再接管 STEP 输出。 */
    if (tmc_angle_move_active) {
        TMC2209_CancelAngleMove();
    }

    if (!tmc_position_move_active) {
        if (target_steps == tmc_position_steps) {
            return true;
        }
        if (!tmc_state.enabled) {
            TMC2209_Enable();
        }
        if (tmc_fault_latched || (!tmc_state.enabled)) {
            return false;
        }

        TMC2209_ResetRamp();
        tmc_direct_control = true;
        tmc_position_move_active = true;
        TMC2209_PositionBeginDirectionSettle();
    } else if (!tmc_position_pulse_running) {
        /* 当前正等待 DIR 建立时可以安全地按最新目标重新选择方向。 */
        if (target_steps == tmc_position_steps) {
            TMC2209_PositionFinish();
        } else if (((target_steps < tmc_position_steps) ? -1 : 1) !=
                   tmc_position_step_sign) {
            TMC2209_PositionBeginDirectionSettle();
        }
    }

    /* 脉冲正在进行时不直接改 DIR/CCR；CC1 回调会在这一个脉冲结束后处理新目标。 */
    return true;
}

bool TMC2209_IsPositionMoveActive(void)
{
    return tmc_position_move_active;
}

void TMC2209_CancelPositionMove(void)
{
    /* 若 STEP 高电平正在输出，等本脉冲在 CC1 自然结束后再关，避免漏记一步。 */
    if (tmc_position_move_active && tmc_position_pulse_running) {
        tmc_position_abort_requested = true;
        return;
    }
    TMC2209_PositionFinish();
}

/* TIM2 每 1 ms 调用：等待 DIR 建立完成后才开启 CC1 脉冲计数。 */
void TMC2209_PositionMoveTick1ms(void)
{
    int8_t desired_sign;

    if (!tmc_position_move_active) {
        return;
    }
    if (tmc_fault_latched || (!tmc_position_reference_valid)) {
        TMC2209_PositionFinish();
        return;
    }
    if (tmc_position_pulse_running) {
        return;
    }
    if (tmc_position_target_steps == tmc_position_steps) {
        TMC2209_PositionFinish();
        return;
    }

    if (tmc_position_dir_settle_ms > 0U) {
        tmc_position_dir_settle_ms--;
        if (tmc_position_dir_settle_ms > 0U) {
            return;
        }
    }

    desired_sign = (tmc_position_target_steps < tmc_position_steps) ? -1 : 1;
    if (desired_sign != tmc_position_step_sign) {
        TMC2209_PositionBeginDirectionSettle();
        return;
    }

    TMC2209_SetSpeed(tmc_position_frequency_hz);
    /* 清掉启动前遗留的比较标志，再开始统计每个完整 PWM 脉冲。 */
    __HAL_TIM_CLEAR_FLAG(tmc_timer, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(tmc_timer, TIM_IT_CC1);
    tmc_position_pulse_running = true;
}

/*
 * TIM9_CH1 的 PWM 高电平结束时调用：这正是一条 STEP 脉冲已经完整送出之时。
 * 此函数运行在 TIM1_BRK_TIM9_IRQn 中，不能加入 HAL_Delay、UART 或浮点 PID。
 */
void TMC2209_PositionPulseFinished(void)
{
    int8_t desired_sign;

    if ((!tmc_position_move_active) || (!tmc_position_pulse_running)) {
        return;
    }

    tmc_position_steps += (int32_t)tmc_position_step_sign;

    if (tmc_position_abort_requested ||
        (tmc_position_target_steps == tmc_position_steps)) {
        TMC2209_PositionFinish();
        return;
    }

    desired_sign = (tmc_position_target_steps < tmc_position_steps) ? -1 : 1;
    if (desired_sign != tmc_position_step_sign) {
        /* 目标跨过当前位置：当前脉冲已结束，现可安全换向。 */
        TMC2209_PositionBeginDirectionSettle();
    } else if (tmc_state.step_frequency_hz != tmc_position_frequency_hz) {
        /* 若上层改变了角速度，也只在完整脉冲结束后更新周期。 */
        TMC2209_SetSpeed(tmc_position_frequency_hz);
    }
}

/* 设置非阻塞斜坡目标；后续由 Timer.c 周期调用 SpeedRamp_Update 实际追踪。 */
void TMC2209_SetAngleSpeed(float speed, float accel)
{
    if (tmc_fault_latched) return;

    /* 切回连续角速度模式会丢失逐脉冲位置坐标，不能继续沿用绝对目标参考。 */
    if (tmc_position_move_active || (speed != 0.0f)) {
        TMC2209_CancelPositionMove();
        tmc_position_reference_valid = false;
    }

    /* 若其他任务切回角速度控制，先安全取消 TASK3 遗留的角度命令。 */
    if (tmc_angle_move_active) {
        TMC2209_CancelAngleMove();
    }

    if (!tmc_state.enabled && (speed != 0.0f)) {
        TMC2209_Enable();
    }
    tmc_target_speed = speed;
    if (accel > 0.0f) {
        tmc_acceleration = accel;
    }
}

/*
 * 每 2 ms 执行一次的速度斜坡状态机：当前速度按加速度逼近目标速度，
 * 方向变化时先更新 DIR，再把速度绝对值换算为 STEP 频率。
 */
void TMC2209_SpeedRamp_Update(void)
{
    float current;
    float target;
    float speed_step;
    float speed_abs;
    int8_t direction;

    if (tmc_direct_control || tmc_fault_latched) return;

    current = tmc_current_speed;
    target = tmc_target_speed;
    /* 当前和目标都为零时，确保定时器比较值也被清零。 */
    if ((current == 0.0f) && (target == 0.0f)) {
        if (tmc_state.step_frequency_hz != 0U) {
            TMC2209_Stop();
        }
        return;
    }

    /* 本周期允许的最大速度变化量。 */
    speed_step = tmc_acceleration * TMC_SPEED_RAMP_DT;
    if (current < target) {
        current += speed_step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= speed_step;
        if (current < target) current = target;
    }
    tmc_current_speed = current;

    direction = (current >= 0.0f) ? 0 : 1;
    if (direction != tmc_last_direction) {
        TMC2209_SetDirection((uint8_t)direction);
    }

    speed_abs = (current >= 0.0f) ? current : -current;
    TMC2209_SetAngularVelocity(speed_abs);
}

/* DIAG 外部中断入口：锁存堵转并立即关断驱动，避免继续撞击机构。 */
void TMC2209_DiagIRQ(void)
{
    tmc_fault_latched = true;
    tmc_state.stalled = true;
    TMC2209_Disable();
}

/* 清除堵转锁存并保持禁用，重新运行前需显式 Enable 或设置非零目标速度。 */
void TMC2209_ClearStall(void)
{
    TMC2209_Disable();
    tmc_state.stalled = false;
    tmc_fault_latched = false;
}

/* 返回内部状态地址，供显示和任务逻辑读取。 */
const TMC2209_State *TMC2209_GetState(void)
{
    return &tmc_state;
}
