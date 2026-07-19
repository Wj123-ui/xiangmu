#include "usart_parse.h"
#include <string.h>
#include <stdlib.h>

UART_Data_t uart_data = {0, 0, 0, 0};
volatile uint8_t rx_byte = 0;

static char rx_buf[32];
static uint8_t rx_index = 0;
static uint8_t parsing = 0;

void UART_Parse_Init(void)
{
    memset(&uart_data, 0, sizeof(uart_data));
    rx_index = 0;
    parsing = 0;
}

void UART_Parse_Process(uint8_t data)
{
    if(data == 'X')
    {
        rx_index = 0;
        rx_buf[rx_index++] = data;
        parsing = 1;
    }
    else if(parsing)
    {
        if(rx_index < sizeof(rx_buf) - 1)
        {
            rx_buf[rx_index++] = data;
            if(data == 'E')
            {
                rx_buf[rx_index] = '\0';
                UART_Parse_String(rx_buf);
                parsing = 0;
                rx_index = 0;
            }
        }
        else
        {
            parsing = 0;
            rx_index = 0;
        }
    }
}

void UART_Parse_String(char *str)
{
    int16_t err_x = 0;
    int16_t err_y = 0;
    uint8_t status = 0;
    char *ptr = str;

    if(*ptr != 'X') return;
    ptr++;
    err_x = (int16_t)strtol(ptr, &ptr, 10);

    if(*ptr != 'Y') return;
    ptr++;
    err_y = (int16_t)strtol(ptr, &ptr, 10);

    if(*ptr != 'Z') return;
    ptr++;
    status = (uint8_t)strtol(ptr, &ptr, 10);

    uart_data.err_x = err_x;
    uart_data.err_y = err_y;
    uart_data.status = status;
    uart_data.updated = 1;
}

void UART4_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_RXNE(UART4))
    {
        rx_byte = LL_USART_ReceiveData8(UART4);
        UART_Parse_Process(rx_byte);
        LL_USART_ClearFlag_RXNE(UART4);
    }
}
