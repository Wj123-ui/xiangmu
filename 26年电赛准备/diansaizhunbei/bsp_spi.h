#ifndef _BSP_SPI_H__
#define _BSP_SPI_H__

#include "ti_msp_dl_config.h"

//CS引脚的输出控制 
//x=0时输出低电平
//x=1时输出高电平
#define SPI_CS(x)  ((x) ? DL_GPIO_setPins(ICM_CS_PORT, ICM_CS_ICM_CS_OUT_PIN) : \
                         DL_GPIO_clearPins(ICM_CS_PORT, ICM_CS_ICM_CS_OUT_PIN))
uint8_t spi_read_write_byte(uint8_t dat);
#endif

