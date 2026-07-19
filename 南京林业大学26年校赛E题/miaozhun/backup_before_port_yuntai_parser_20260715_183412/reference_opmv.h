#ifndef __opmv_H
#define __opmv_H

#include "main.h"
#include <stdint.h>

typedef struct {
    volatile int16_t err_x;
    volatile int16_t err_y;
    volatile uint8_t status;
    volatile uint8_t updated;
} UART_Data_t;

extern UART_Data_t uart_data;
extern volatile uint8_t rx_byte;

void UART_Parse_Init(void);
void UART_Parse_Process(uint8_t data);
void UART_Parse_String(char *str);

#endif
