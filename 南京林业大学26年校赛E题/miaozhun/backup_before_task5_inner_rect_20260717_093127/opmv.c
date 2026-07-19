/*
 * opmv.c — K230 视觉通信实现 (UART4, 115200)
 *
 * 协议解析:
 *   主协议: X<dx>Y<dy>Z<found>E\r\n
 *   发送: #<task_id>\n
 *
 * 解析方式移植自 C:\Users\Lenovo\Desktop\项目\yuntai\Core\Src\opmv.c
 */
#include "usart_parse.h"
#include "opmv.h"
#include <string.h>
#include <stdlib.h>

/* ========== 全局变量 ========== */
UART_Data_t uart_data = {0};
volatile uint8_t rx_byte = 0;
volatile uint32_t uart_rx_byte_count = 0;
volatile uint32_t uart_rx_frame_count = 0;
volatile uint32_t uart_rx_error_count = 0;
volatile uint8_t uart_last_rx_byte = 0;
volatile char uart_rx_preview[32] = {0};
volatile uint8_t uart_rx_preview_len = 0;

/* ========== 接收缓冲与解析状态（直接移植参考工程） ========== */
static char rx_buf[32];
static uint8_t rx_index = 0;
static uint8_t parsing = 0;

/* ========== 初始化 ========== */
void UART_Parse_Init(void)
{
    memset(&uart_data, 0, sizeof(uart_data));
    uart_data.digit = -1;
    rx_index = 0;
    parsing = 0;
    uart_rx_byte_count = 0;
    uart_rx_frame_count = 0;
    uart_rx_error_count = 0;
    uart_last_rx_byte = 0;
    uart_rx_preview_len = 0;
    uart_rx_preview[0] = '\0';
}

void K230_ClearLatestData(void)
{
    uart_data.type = 0;
    uart_data.err_x = 0;
    uart_data.err_y = 0;
    uart_data.status = 0;
    uart_data.digit = -1;
    uart_data.updated = 0;
}

/* ========== 逐字节解析 (UART4 中断调用) ========== */
void UART_Parse_Process(uint8_t data)
{
    if(data == 'X')
    {
        rx_index = 0;
        rx_buf[rx_index++] = (char)data;
        parsing = 1;
        uart_rx_preview_len = 1;
        uart_rx_preview[0] = 'X';
        uart_rx_preview[1] = '\0';
    }
    else if(parsing)
    {
        if(rx_index < sizeof(rx_buf) - 1U)
        {
            rx_buf[rx_index++] = (char)data;
            if (uart_rx_preview_len < sizeof(uart_rx_preview) - 1U) {
                uart_rx_preview[uart_rx_preview_len++] = (char)data;
                uart_rx_preview[uart_rx_preview_len] = '\0';
            }
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
            uart_rx_error_count++;
        }
    }
}

void UART_Parse_String(char *str)
{
    int16_t err_x = 0;
    int16_t err_y = 0;
    uint8_t status = 0;
    int16_t digit = -1;
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

    if(*ptr == 'D') {
        ptr++;
        digit = (int16_t)strtol(ptr, &ptr, 10);
    }
    if(*ptr != 'E') return;

    uart_data.type = 'A';
    uart_data.err_x = err_x;
    uart_data.err_y = err_y;
    uart_data.status = status;
    uart_data.digit = (digit >= 0 && digit <= 9) ? (int8_t)digit : -1;
    uart_data.updated = 1;
    uart_rx_frame_count++;
}

/* ========== 向 K230 发送任务模式命令 ========== */
/*
 * 发送: #<task_id>\n  (如 "#1\n", "#5\n")
 * 使用 UART4 (LL 库, 因为 UART4 由 LL 初始化)
 */
void K230_SendTaskMode(uint8_t task_id)
{
    char cmd[8];
    cmd[0] = '#';
    cmd[1] = '0' + (task_id % 10);
    cmd[2] = '\n';
    cmd[3] = '\0';

    for (int i = 0; i < 3 && cmd[i] != '\0'; i++) {
        /* 等待发送缓冲区空 */
        while (!LL_USART_IsActiveFlag_TXE(UART4)) {}
        LL_USART_TransmitData8(UART4, (uint8_t)cmd[i]);
    }
    /* 等待最后一字节发送完成 */
    while (!LL_USART_IsActiveFlag_TC(UART4)) {}
}

/* ========== UART4 接收中断 ========== */
void UART4_IRQHandler(void)
{
    /* 先处理溢出/帧/噪声错误；读 SR 后读 DR 即可清除。 */
    if (LL_USART_IsActiveFlag_ORE(UART4) ||
        LL_USART_IsActiveFlag_FE(UART4)  ||
        LL_USART_IsActiveFlag_NE(UART4)) {
        volatile uint32_t sr = UART4->SR;
        volatile uint32_t dr = UART4->DR;
        (void)sr;
        (void)dr;
        uart_rx_error_count++;
        return;
    }

    if (LL_USART_IsActiveFlag_RXNE(UART4)) {
        rx_byte = LL_USART_ReceiveData8(UART4);
        uart_last_rx_byte = rx_byte;
        uart_rx_byte_count++;
        UART_Parse_Process(rx_byte);
    }
}
