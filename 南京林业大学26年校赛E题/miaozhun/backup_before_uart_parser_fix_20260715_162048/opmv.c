/*
 * opmv.c — K230 视觉通信实现 (UART4, 115200)
 *
 * 协议解析:
 *   接收: $<type>,<p1>,<p2>,<p3>,<p4>\n
 *   发送: #<task_id>\n
 *
 * 解析状态机:
 *   WAIT_$ → READ_TYPE → READ_PARAMS → WAIT_NL → 完成
 */
#include "usart_parse.h"
#include "opmv.h"
#include <string.h>
#include <stdlib.h>

/* ========== 全局变量 ========== */
UART_Data_t uart_data = {0, 0, 0, 0, 0};
volatile uint8_t rx_byte = 0;

/* ========== 接收缓冲与解析状态 ========== */
#define RX_BUF_SIZE 32
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t parse_state = 0;   /* 0=等待$ 1=读取type 2=读取参数 3=等待\n */

/* ========== 初始化 ========== */
void UART_Parse_Init(void)
{
    memset(&uart_data, 0, sizeof(uart_data));
    rx_index = 0;
    parse_state = 0;
}

/* ========== 逐字节解析 (UART4 中断调用) ========== */
void UART_Parse_Process(uint8_t data)
{
    char ch = (char)data;

    switch (parse_state) {
    case 0:  /* 等待起始符 '$' */
        if (ch == '$') {
            rx_index = 0;
            rx_buf[rx_index++] = ch;
            parse_state = 1;
        }
        break;

    case 1:  /* 读取类型字符 (A/C/D/L/R) */
        rx_buf[rx_index++] = ch;
        if (ch == ',') {
            parse_state = 2;  /* 类型字符后跟逗号, 进入参数读取 */
        } else if (ch == '\n') {
            /* 无参数帧 (如 $R\n) */
            rx_buf[rx_index] = '\0';
            UART_Parse_String(rx_buf);
            parse_state = 0;
        } else if (rx_index >= RX_BUF_SIZE - 1) {
            parse_state = 0;  /* 缓冲溢出, 丢弃 */
        }
        break;

    case 2:  /* 读取参数, 直到 '\n' */
        if (ch == '\n') {
            rx_buf[rx_index] = '\0';
            UART_Parse_String(rx_buf);
            parse_state = 0;
        } else {
            if (rx_index < RX_BUF_SIZE - 1) {
                rx_buf[rx_index++] = ch;
            } else {
                parse_state = 0;  /* 缓冲溢出 */
            }
        }
        break;

    default:
        parse_state = 0;
        break;
    }
}

/* ========== 解析完整帧字符串 ========== */
/*
 * 帧格式: $<type>,<p1>,<p2>,<p3>,<p4>
 * 示例:   $A,15,-20,0,0
 *         $D,5,0,0,0
 *         $R,0,0,0,0
 */
void UART_Parse_String(char *str)
{
    if (str[0] != '$') return;

    char type = str[1];
    int16_t p1 = 0, p2 = 0, p3 = 0;

    /* 跳过 "$X," 找到第一个参数 */
    char *ptr = str + 2;
    if (*ptr == ',') ptr++;
    else return;  /* 格式错误 */

    /* 解析 4 个逗号分隔的整数 (不够则默认 0) */
    p1 = (int16_t)strtol(ptr, &ptr, 10);
    if (*ptr == ',') ptr++;
    p2 = (int16_t)strtol(ptr, &ptr, 10);
    if (*ptr == ',') ptr++;
    p3 = (int16_t)strtol(ptr, &ptr, 10);

    /* 存入全局结构 */
    uart_data.type   = (uint8_t)type;
    uart_data.err_x  = p1;
    uart_data.err_y  = p2;
    uart_data.status = (uint8_t)p3;
    uart_data.updated = 1;
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
    if (LL_USART_IsActiveFlag_RXNE(UART4)) {
        rx_byte = LL_USART_ReceiveData8(UART4);
        UART_Parse_Process(rx_byte);
        LL_USART_ClearFlag_RXNE(UART4);
    }
}
