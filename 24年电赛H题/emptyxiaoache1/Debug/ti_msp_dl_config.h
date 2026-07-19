/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_8
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM19)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_9
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM26)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM26_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_Scan */
#define TIMER_Scan_INST                                                  (TIMG6)
#define TIMER_Scan_INST_IRQHandler                              TIMG6_IRQHandler
#define TIMER_Scan_INST_INT_IRQN                                (TIMG6_INT_IRQn)
#define TIMER_Scan_INST_LOAD_VALUE                                        (124U)
/* Defines for TIMER_encoder */
#define TIMER_encoder_INST                                               (TIMA1)
#define TIMER_encoder_INST_IRQHandler                           TIMA1_IRQHandler
#define TIMER_encoder_INST_INT_IRQN                             (TIMA1_INT_IRQn)
#define TIMER_encoder_INST_LOAD_VALUE                                    (1249U)
/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (499U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C0
#define I2C_OLED_INST_IRQHandler                                 I2C0_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C0_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for I2C_MPU6050 */
#define I2C_MPU6050_INST                                                    I2C1
#define I2C_MPU6050_INST_IRQHandler                              I2C1_IRQHandler
#define I2C_MPU6050_INST_INT_IRQN                                  I2C1_INT_IRQn
#define I2C_MPU6050_BUS_SPEED_HZ                                          400000
#define GPIO_I2C_MPU6050_SDA_PORT                                          GPIOB
#define GPIO_I2C_MPU6050_SDA_PIN                                   DL_GPIO_PIN_3
#define GPIO_I2C_MPU6050_IOMUX_SDA                               (IOMUX_PINCM16)
#define GPIO_I2C_MPU6050_IOMUX_SDA_FUNC                IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_MPU6050_SCL_PORT                                          GPIOB
#define GPIO_I2C_MPU6050_SCL_PIN                                   DL_GPIO_PIN_2
#define GPIO_I2C_MPU6050_IOMUX_SCL                               (IOMUX_PINCM15)
#define GPIO_I2C_MPU6050_IOMUX_SCL_FUNC                IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_JY61P */
#define UART_JY61P_INST                                                    UART0
#define UART_JY61P_INST_FREQUENCY                                       32000000
#define UART_JY61P_INST_IRQHandler                              UART0_IRQHandler
#define UART_JY61P_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_JY61P_RX_PORT                                            GPIOB
#define GPIO_UART_JY61P_TX_PORT                                            GPIOB
#define GPIO_UART_JY61P_RX_PIN                                     DL_GPIO_PIN_1
#define GPIO_UART_JY61P_TX_PIN                                     DL_GPIO_PIN_0
#define GPIO_UART_JY61P_IOMUX_RX                                 (IOMUX_PINCM13)
#define GPIO_UART_JY61P_IOMUX_TX                                 (IOMUX_PINCM12)
#define GPIO_UART_JY61P_IOMUX_RX_FUNC                  IOMUX_PINCM13_PF_UART0_RX
#define GPIO_UART_JY61P_IOMUX_TX_FUNC                  IOMUX_PINCM12_PF_UART0_TX
#define UART_JY61P_BAUD_RATE                                            (115200)
#define UART_JY61P_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_JY61P_FBRD_32_MHZ_115200_BAUD                                  (23)





/* Port definition for Pin Group GPIO_LED */
#define GPIO_LED_PORT                                                    (GPIOB)

/* Defines for PIN_LED: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_LED_PIN_LED_PIN                                    (DL_GPIO_PIN_26)
#define GPIO_LED_PIN_LED_IOMUX                                   (IOMUX_PINCM57)
/* Port definition for Pin Group GPIO_Fmqi */
#define GPIO_Fmqi_PORT                                                   (GPIOB)

/* Defines for PIN_fmq: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_Fmqi_PIN_fmq_PIN                                   (DL_GPIO_PIN_27)
#define GPIO_Fmqi_PIN_fmq_IOMUX                                  (IOMUX_PINCM58)
/* Port definition for Pin Group GPIO_key */
#define GPIO_key_PORT                                                    (GPIOB)

/* Defines for PIN_4: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_key_PIN_4_PIN                                       (DL_GPIO_PIN_4)
#define GPIO_key_PIN_4_IOMUX                                     (IOMUX_PINCM17)
/* Defines for PIN_5: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_key_PIN_5_PIN                                       (DL_GPIO_PIN_5)
#define GPIO_key_PIN_5_IOMUX                                     (IOMUX_PINCM18)
/* Defines for PIN_6: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_key_PIN_6_PIN                                       (DL_GPIO_PIN_6)
#define GPIO_key_PIN_6_IOMUX                                     (IOMUX_PINCM23)
/* Defines for PIN_7: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_key_PIN_7_PIN                                       (DL_GPIO_PIN_7)
#define GPIO_key_PIN_7_IOMUX                                     (IOMUX_PINCM24)
/* Port definition for Pin Group GPIO_PWM1 */
#define GPIO_PWM1_PORT                                                   (GPIOA)

/* Defines for AIN1: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_PWM1_AIN1_PIN                                      (DL_GPIO_PIN_31)
#define GPIO_PWM1_AIN1_IOMUX                                      (IOMUX_PINCM6)
/* Defines for AIN2: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_PWM1_AIN2_PIN                                      (DL_GPIO_PIN_22)
#define GPIO_PWM1_AIN2_IOMUX                                     (IOMUX_PINCM47)
/* Defines for PIN_12: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_xunji_PIN_12_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_12_PIN                                   (DL_GPIO_PIN_12)
#define GPIO_xunji_PIN_12_IOMUX                                  (IOMUX_PINCM34)
/* Defines for PIN_13: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_xunji_PIN_13_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_13_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_xunji_PIN_13_IOMUX                                  (IOMUX_PINCM35)
/* Defines for PIN_14: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_xunji_PIN_14_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_14_PIN                                   (DL_GPIO_PIN_14)
#define GPIO_xunji_PIN_14_IOMUX                                  (IOMUX_PINCM36)
/* Defines for PIN_16: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_xunji_PIN_16_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_16_PIN                                   (DL_GPIO_PIN_16)
#define GPIO_xunji_PIN_16_IOMUX                                  (IOMUX_PINCM38)
/* Defines for PIN_17: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_xunji_PIN_17_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_17_PIN                                   (DL_GPIO_PIN_17)
#define GPIO_xunji_PIN_17_IOMUX                                  (IOMUX_PINCM39)
/* Defines for PIN_18: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_xunji_PIN_18_PORT                                           (GPIOA)
#define GPIO_xunji_PIN_18_PIN                                   (DL_GPIO_PIN_18)
#define GPIO_xunji_PIN_18_IOMUX                                  (IOMUX_PINCM40)
/* Defines for PIN_8: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_xunji_PIN_8_PORT                                            (GPIOB)
#define GPIO_xunji_PIN_8_PIN                                     (DL_GPIO_PIN_8)
#define GPIO_xunji_PIN_8_IOMUX                                   (IOMUX_PINCM25)
/* Port definition for Pin Group GPIO_Encoder */
#define GPIO_Encoder_PORT                                                (GPIOB)

/* Defines for PIN_right_A: GPIOB.10 with pinCMx 27 on package pin 62 */
// pins affected by this interrupt request:["PIN_right_A","PIN_right_B","PIN_left_A","PIN_left_B"]
#define GPIO_Encoder_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_Encoder_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_Encoder_PIN_right_A_IIDX                       (DL_GPIO_IIDX_DIO10)
#define GPIO_Encoder_PIN_right_A_PIN                            (DL_GPIO_PIN_10)
#define GPIO_Encoder_PIN_right_A_IOMUX                           (IOMUX_PINCM27)
/* Defines for PIN_right_B: GPIOB.11 with pinCMx 28 on package pin 63 */
#define GPIO_Encoder_PIN_right_B_IIDX                       (DL_GPIO_IIDX_DIO11)
#define GPIO_Encoder_PIN_right_B_PIN                            (DL_GPIO_PIN_11)
#define GPIO_Encoder_PIN_right_B_IOMUX                           (IOMUX_PINCM28)
/* Defines for PIN_left_A: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_Encoder_PIN_left_A_IIDX                        (DL_GPIO_IIDX_DIO15)
#define GPIO_Encoder_PIN_left_A_PIN                             (DL_GPIO_PIN_15)
#define GPIO_Encoder_PIN_left_A_IOMUX                            (IOMUX_PINCM32)
/* Defines for PIN_left_B: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_Encoder_PIN_left_B_IIDX                        (DL_GPIO_IIDX_DIO16)
#define GPIO_Encoder_PIN_left_B_PIN                             (DL_GPIO_PIN_16)
#define GPIO_Encoder_PIN_left_B_IOMUX                            (IOMUX_PINCM33)
/* Port definition for Pin Group GPIO_PWM2 */
#define GPIO_PWM2_PORT                                                   (GPIOB)

/* Defines for BIN1: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_PWM2_BIN1_PIN                                      (DL_GPIO_PIN_24)
#define GPIO_PWM2_BIN1_IOMUX                                     (IOMUX_PINCM52)
/* Defines for BIN2: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_PWM2_BIN2_PIN                                      (DL_GPIO_PIN_12)
#define GPIO_PWM2_BIN2_IOMUX                                     (IOMUX_PINCM29)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_TIMER_Scan_init(void);
void SYSCFG_DL_TIMER_encoder_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_I2C_MPU6050_init(void);
void SYSCFG_DL_UART_JY61P_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
