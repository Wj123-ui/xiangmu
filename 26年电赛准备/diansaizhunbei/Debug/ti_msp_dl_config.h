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


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_left */
#define PWM_left_INST                                                      TIMG0
#define PWM_left_INST_IRQHandler                                TIMG0_IRQHandler
#define PWM_left_INST_INT_IRQN                                  (TIMG0_INT_IRQn)
#define PWM_left_INST_CLK_FREQ                                          40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_left_C0_PORT                                              GPIOA
#define GPIO_PWM_left_C0_PIN                                      DL_GPIO_PIN_12
#define GPIO_PWM_left_C0_IOMUX                                   (IOMUX_PINCM34)
#define GPIO_PWM_left_C0_IOMUX_FUNC                  IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_left_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_left_C1_PORT                                              GPIOA
#define GPIO_PWM_left_C1_PIN                                      DL_GPIO_PIN_13
#define GPIO_PWM_left_C1_IOMUX                                   (IOMUX_PINCM35)
#define GPIO_PWM_left_C1_IOMUX_FUNC                  IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_left_C1_IDX                                 DL_TIMER_CC_1_INDEX

/* Defines for PWM_right */
#define PWM_right_INST                                                     TIMG7
#define PWM_right_INST_IRQHandler                               TIMG7_IRQHandler
#define PWM_right_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define PWM_right_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_right_C0_PORT                                             GPIOA
#define GPIO_PWM_right_C0_PIN                                     DL_GPIO_PIN_26
#define GPIO_PWM_right_C0_IOMUX                                  (IOMUX_PINCM59)
#define GPIO_PWM_right_C0_IOMUX_FUNC                 IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_PWM_right_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_right_C1_PORT                                             GPIOA
#define GPIO_PWM_right_C1_PIN                                     DL_GPIO_PIN_27
#define GPIO_PWM_right_C1_IOMUX                                  (IOMUX_PINCM60)
#define GPIO_PWM_right_C1_IOMUX_FUNC                 IOMUX_PINCM60_PF_TIMG7_CCP1
#define GPIO_PWM_right_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_12 */
#define TIMER_12_INST                                                   (TIMG12)
#define TIMER_12_INST_IRQHandler                               TIMG12_IRQHandler
#define TIMER_12_INST_INT_IRQN                                 (TIMG12_INT_IRQn)
#define TIMER_12_INST_LOAD_VALUE                                        (79999U)



/* Defines for SPI_OLED */
#define SPI_OLED_INST                                                      SPI1
#define SPI_OLED_INST_IRQHandler                                SPI1_IRQHandler
#define SPI_OLED_INST_INT_IRQN                                    SPI1_INT_IRQn
#define GPIO_SPI_OLED_PICO_PORT                                           GPIOB
#define GPIO_SPI_OLED_PICO_PIN                                    DL_GPIO_PIN_8
#define GPIO_SPI_OLED_IOMUX_PICO                                (IOMUX_PINCM25)
#define GPIO_SPI_OLED_IOMUX_PICO_FUNC                IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_OLED_POCI_PORT                                           GPIOB
#define GPIO_SPI_OLED_POCI_PIN                                    DL_GPIO_PIN_7
#define GPIO_SPI_OLED_IOMUX_POCI                                (IOMUX_PINCM24)
#define GPIO_SPI_OLED_IOMUX_POCI_FUNC                IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_OLED */
#define GPIO_SPI_OLED_SCLK_PORT                                           GPIOB
#define GPIO_SPI_OLED_SCLK_PIN                                    DL_GPIO_PIN_9
#define GPIO_SPI_OLED_IOMUX_SCLK                                (IOMUX_PINCM26)
#define GPIO_SPI_OLED_IOMUX_SCLK_FUNC                IOMUX_PINCM26_PF_SPI1_SCLK
/* Defines for SPI_ICM45686 */
#define SPI_ICM45686_INST                                                  SPI0
#define SPI_ICM45686_INST_IRQHandler                            SPI0_IRQHandler
#define SPI_ICM45686_INST_INT_IRQN                                SPI0_INT_IRQn
#define GPIO_SPI_ICM45686_PICO_PORT                                       GPIOA
#define GPIO_SPI_ICM45686_PICO_PIN                               DL_GPIO_PIN_14
#define GPIO_SPI_ICM45686_IOMUX_PICO                            (IOMUX_PINCM36)
#define GPIO_SPI_ICM45686_IOMUX_PICO_FUNC            IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_ICM45686_POCI_PORT                                       GPIOB
#define GPIO_SPI_ICM45686_POCI_PIN                               DL_GPIO_PIN_19
#define GPIO_SPI_ICM45686_IOMUX_POCI                            (IOMUX_PINCM45)
#define GPIO_SPI_ICM45686_IOMUX_POCI_FUNC            IOMUX_PINCM45_PF_SPI0_POCI
/* GPIO configuration for SPI_ICM45686 */
#define GPIO_SPI_ICM45686_SCLK_PORT                                       GPIOB
#define GPIO_SPI_ICM45686_SCLK_PIN                               DL_GPIO_PIN_18
#define GPIO_SPI_ICM45686_IOMUX_SCLK                            (IOMUX_PINCM44)
#define GPIO_SPI_ICM45686_IOMUX_SCLK_FUNC            IOMUX_PINCM44_PF_SPI0_SCLK



/* Port definition for Pin Group ICM_CS */
#define ICM_CS_PORT                                                      (GPIOA)

/* Defines for ICM_CS_OUT: GPIOA.15 with pinCMx 37 on package pin 8 */
#define ICM_CS_ICM_CS_OUT_PIN                                   (DL_GPIO_PIN_15)
#define ICM_CS_ICM_CS_OUT_IOMUX                                  (IOMUX_PINCM37)
/* Port definition for Pin Group ICM_INT */
#define ICM_INT_PORT                                                     (GPIOB)

/* Defines for ICM_DRDY: GPIOB.27 with pinCMx 58 on package pin 29 */
// groups represented: ["GPIO_Encoder","ICM_INT"]
// pins affected: ["PIN_right_A","PIN_right_B","PIN_left_A","PIN_left_B","ICM_DRDY"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ICM_INT_ICM_DRDY_IIDX                               (DL_GPIO_IIDX_DIO27)
#define ICM_INT_ICM_DRDY_PIN                                    (DL_GPIO_PIN_27)
#define ICM_INT_ICM_DRDY_IOMUX                                   (IOMUX_PINCM58)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOB)

/* Defines for RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define OLED_RES_PIN                                            (DL_GPIO_PIN_10)
#define OLED_RES_IOMUX                                           (IOMUX_PINCM27)
/* Defines for DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define OLED_DC_PIN                                             (DL_GPIO_PIN_11)
#define OLED_DC_IOMUX                                            (IOMUX_PINCM28)
/* Defines for CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define OLED_CS_PIN                                             (DL_GPIO_PIN_14)
#define OLED_CS_IOMUX                                            (IOMUX_PINCM31)
/* Port definition for Pin Group GPIO_Encoder */
#define GPIO_Encoder_PORT                                                (GPIOB)

/* Defines for PIN_right_A: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_Encoder_PIN_right_A_IIDX                       (DL_GPIO_IIDX_DIO13)
#define GPIO_Encoder_PIN_right_A_PIN                            (DL_GPIO_PIN_13)
#define GPIO_Encoder_PIN_right_A_IOMUX                           (IOMUX_PINCM30)
/* Defines for PIN_right_B: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_Encoder_PIN_right_B_IIDX                       (DL_GPIO_IIDX_DIO12)
#define GPIO_Encoder_PIN_right_B_PIN                            (DL_GPIO_PIN_12)
#define GPIO_Encoder_PIN_right_B_IOMUX                           (IOMUX_PINCM29)
/* Defines for PIN_left_A: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_Encoder_PIN_left_A_IIDX                        (DL_GPIO_IIDX_DIO15)
#define GPIO_Encoder_PIN_left_A_PIN                             (DL_GPIO_PIN_15)
#define GPIO_Encoder_PIN_left_A_IOMUX                            (IOMUX_PINCM32)
/* Defines for PIN_left_B: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_Encoder_PIN_left_B_IIDX                        (DL_GPIO_IIDX_DIO16)
#define GPIO_Encoder_PIN_left_B_PIN                             (DL_GPIO_PIN_16)
#define GPIO_Encoder_PIN_left_B_IOMUX                            (IOMUX_PINCM33)
/* Defines for PIN_1: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_xunji_PIN_1_PORT                                            (GPIOB)
#define GPIO_xunji_PIN_1_PIN                                    (DL_GPIO_PIN_26)
#define GPIO_xunji_PIN_1_IOMUX                                   (IOMUX_PINCM57)
/* Defines for PIN_2: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_xunji_PIN_2_PORT                                            (GPIOB)
#define GPIO_xunji_PIN_2_PIN                                    (DL_GPIO_PIN_25)
#define GPIO_xunji_PIN_2_IOMUX                                   (IOMUX_PINCM56)
/* Defines for PIN_3: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_xunji_PIN_3_PORT                                            (GPIOB)
#define GPIO_xunji_PIN_3_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_xunji_PIN_3_IOMUX                                   (IOMUX_PINCM52)
/* Defines for PIN_4: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_xunji_PIN_4_PORT                                            (GPIOB)
#define GPIO_xunji_PIN_4_PIN                                    (DL_GPIO_PIN_20)
#define GPIO_xunji_PIN_4_IOMUX                                   (IOMUX_PINCM48)
/* Defines for PIN_5: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_xunji_PIN_5_PORT                                            (GPIOA)
#define GPIO_xunji_PIN_5_PIN                                    (DL_GPIO_PIN_25)
#define GPIO_xunji_PIN_5_IOMUX                                   (IOMUX_PINCM55)
/* Defines for PIN_6: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_xunji_PIN_6_PORT                                            (GPIOA)
#define GPIO_xunji_PIN_6_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_xunji_PIN_6_IOMUX                                   (IOMUX_PINCM54)
/* Defines for PIN_7: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_xunji_PIN_7_PORT                                            (GPIOA)
#define GPIO_xunji_PIN_7_PIN                                    (DL_GPIO_PIN_22)
#define GPIO_xunji_PIN_7_IOMUX                                   (IOMUX_PINCM47)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_left_init(void);
void SYSCFG_DL_PWM_right_init(void);
void SYSCFG_DL_TIMER_12_init(void);
void SYSCFG_DL_SPI_OLED_init(void);
void SYSCFG_DL_SPI_ICM45686_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
