#include "imu.h"
#include "board.h"
#include "inv_imu_driver.h"
#include <string.h>

/*
 * ICM45686 硬件适配层。
 * 本文件把官方 inv_imu_driver 所需的寄存器读写接口连接到 STM32 HAL SPI，
 * 并负责传感器上电初始化、量程/采样率配置、原始数据物理量换算和 DRDY 标志。
 */

/* SPI 超时、上电稳定等待及初始化重试参数，单位均为毫秒。 */
#define ICM_SPI_TIMEOUT_MS       20U
#define ICM_POWER_ON_DELAY_MS   100U
#define ICM_INIT_RETRY_COUNT      5U
#define ICM_INIT_RETRY_DELAY_MS  20U
#define ICM_SOFT_RESET_DELAY_MS  10U

static SPI_HandleTypeDef *icm_spi;   /* ICM45686 所连接的 STM32 SPI 句柄。 */
static inv_imu_device_t icm_device;  /* 官方驱动使用的设备及传输接口对象。 */
static volatile bool icm_data_ready; /* INT1 数据就绪中断置位，读取成功后清零。 */
static bool icm_initialized;         /* 芯片完整配置成功标志。 */

/*
 * SPI 连续读取寄存器：最高位为 1 表示读操作；片选在地址和数据阶段保持低电平。
 * 返回官方驱动规定的 0、参数错误或传输错误代码。
 */
static int ICM_ReadRegisters(uint8_t reg, uint8_t *buffer, uint32_t length)
{
    uint8_t address = reg | 0x80U;

    if ((icm_spi == NULL) || (buffer == NULL)) return INV_IMU_ERROR_BAD_ARG;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(icm_spi, &address, 1U, ICM_SPI_TIMEOUT_MS) != HAL_OK ||
        HAL_SPI_Receive(icm_spi, buffer, (uint16_t)length,
                        ICM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
        return INV_IMU_ERROR_TRANSPORT;
    }
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return 0;
}

/*
 * SPI 写寄存器。为兼容芯片寄存器访问时序，多字节数据逐个地址写入，
 * 每个寄存器独立拉低和释放片选。
 */
static int ICM_WriteRegisters(uint8_t reg, const uint8_t *buffer, uint32_t length)
{
    uint32_t i;

    if ((icm_spi == NULL) || (buffer == NULL)) return INV_IMU_ERROR_BAD_ARG;
    for (i = 0U; i < length; i++) {
        uint8_t packet[2] = {(uint8_t)(reg + i), buffer[i]};
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
        if (HAL_SPI_Transmit(icm_spi, packet, sizeof(packet),
                            ICM_SPI_TIMEOUT_MS) != HAL_OK) {
            HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
            return INV_IMU_ERROR_TRANSPORT;
        }
        HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    }
    return 0;
}

/*
 * 初始化流程：绑定 SPI 与官方驱动回调，等待电源稳定，检查 WHO_AM_I，
 * 软件复位后配置 INT1、量程、200 Hz 采样率、低通带宽和低噪声工作模式。
 */
bool ICM45686_Init(SPI_HandleTypeDef *hspi)
{
    uint8_t who_am_i;
    uint8_t attempt;
    inv_imu_int_pin_config_t pin_config;
    inv_imu_int_state_t interrupt_config;
    int rc;

    icm_spi = hspi;
    icm_data_ready = false;
    icm_initialized = false;
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    /* 将 STM32 HAL SPI 读写和微秒延时函数交给官方驱动调用。 */
    icm_device.transport.read_reg = ICM_ReadRegisters;
    icm_device.transport.write_reg = ICM_WriteRegisters;
    icm_device.transport.serif_type = UI_SPI4;
    icm_device.transport.sleep_us = Board_DelayUs;

    /* 电池供电的电压上升可能慢于 USB，先等待电源轨和芯片内部电路稳定。 */
    HAL_Delay(ICM_POWER_ON_DELAY_MS);

    /* WHO_AM_I 或任一配置步骤失败时整套初始化会重新尝试。 */
    for (attempt = 0U; attempt < ICM_INIT_RETRY_COUNT; attempt++) {
        who_am_i = 0U;
        rc = inv_imu_get_who_am_i(&icm_device, &who_am_i);
        if ((rc != 0) || (who_am_i != INV_IMU_WHOAMI)) {
            HAL_Delay(ICM_INIT_RETRY_DELAY_MS);
            continue;
        }

        rc = inv_imu_soft_reset(&icm_device);
        HAL_Delay(ICM_SOFT_RESET_DELAY_MS);
        if (rc != 0) {
            HAL_Delay(ICM_INIT_RETRY_DELAY_MS);
            continue;
        }

        /* INT1 配置为高电平有效、脉冲模式、推挽输出。 */
        memset(&pin_config, 0, sizeof(pin_config));
        pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
        pin_config.int_mode = INTX_CONFIG2_INTX_MODE_PULSE;
        pin_config.int_drive = INTX_CONFIG2_INTX_DRIVE_PP;
        rc = inv_imu_set_pin_config_int(&icm_device, INV_IMU_INT1,
                                        &pin_config);

        /* 仅开启 UI 数据就绪中断，其他中断源保持关闭。 */
        memset(&interrupt_config, INV_IMU_DISABLE, sizeof(interrupt_config));
        interrupt_config.INV_UI_DRDY = INV_IMU_ENABLE;
        rc |= inv_imu_set_config_int(&icm_device, INV_IMU_INT1,
                                     &interrupt_config);
        /* 加速度 ±4 g、陀螺仪 ±1000 °/s，二者输出数据率均为 200 Hz。 */
        rc |= inv_imu_set_accel_fsr(&icm_device,
                                    ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G);
        rc |= inv_imu_set_gyro_fsr(&icm_device,
                                   GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS);
        rc |= inv_imu_set_accel_frequency(
            &icm_device, ACCEL_CONFIG0_ACCEL_ODR_200_HZ);
        rc |= inv_imu_set_gyro_frequency(
            &icm_device, GYRO_CONFIG0_GYRO_ODR_200_HZ);
        /* 为加速度和陀螺仪开启 DIV_4 低通带宽并进入低噪声模式。 */
        rc |= inv_imu_set_accel_ln_bw(
            &icm_device, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
        rc |= inv_imu_set_gyro_ln_bw(
            &icm_device, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
        rc |= inv_imu_set_accel_mode(&icm_device,
                                     PWR_MGMT0_ACCEL_MODE_LN);
        rc |= inv_imu_set_gyro_mode(&icm_device,
                                    PWR_MGMT0_GYRO_MODE_LN);

        if (rc == 0) {
            HAL_Delay(ICM_SOFT_RESET_DELAY_MS);
            icm_data_ready = false;
            icm_initialized = true;
            return true;
        }

        HAL_Delay(ICM_INIT_RETRY_DELAY_MS);
    }

    return false;
}

/*
 * 读取官方驱动返回的原始寄存器值并换算：
 * 加速度输出 mg，角速度输出 °/s，温度输出 ℃。
 */
bool ICM45686_Read(ICM45686_Data *data)
{
    inv_imu_sensor_data_t raw;

    if ((!icm_initialized) || (data == NULL)) return false;
    if (inv_imu_get_register_data(&icm_device, &raw) != 0) return false;

    /* ±4 g 量程对应 16 位满量程；换算结果乘 1000 后以 mg 表示。 */
    data->accel_mg[0] = (float)(raw.accel_data[0] * 4) / 32.768f;
    data->accel_mg[1] = (float)(raw.accel_data[1] * 4) / 32.768f;
    data->accel_mg[2] = (float)(raw.accel_data[2] * 4) / 32.768f;
    /* ±1000 °/s 量程按 32768 个半量程计数换算。 */
    data->gyro_dps[0] = (float)(raw.gyro_data[0] * 1000) / 32768.0f;
    data->gyro_dps[1] = (float)(raw.gyro_data[1] * 1000) / 32768.0f;
    data->gyro_dps[2] = (float)(raw.gyro_data[2] * 1000) / 32768.0f;
    /* 数据手册温度换算：25 ℃ + 原始值 / 128。 */
    data->temperature_c = 25.0f + (float)raw.temp_data / 128.0f;
    icm_data_ready = false;
    return true;
}

/* 由 IMU INT1 的 GPIO 外部中断回调调用，只置位标志，不在中断内读 SPI。 */
void ICM45686_DataReadyIRQ(void)
{
    icm_data_ready = true;
}

/* 非阻塞查询是否收到过新的数据就绪中断。 */
bool ICM45686_IsDataReady(void)
{
    return icm_data_ready;
}
