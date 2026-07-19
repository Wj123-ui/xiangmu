/*
 * opmv.c — K230 视觉通信实现 (UART4, 115200)
 *
 * 协议解析:
 *   主协议: X<dx>Y<dy>Z<found>E\r\n
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
volatile uint32_t uart_rx_byte_count = 0;
volatile uint32_t uart_rx_frame_count = 0;
volatile uint32_t uart_rx_error_count = 0;
volatile uint8_t uart_last_rx_byte = 0;

/* ========== 接收缓冲与解析状态 ========== */
#define RX_BUF_SIZE 32
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_index = 0;
static uint8_t frame_receiving = 0;
static uint8_t frame_format = 0;  /* 1=旧 $A,..., 2=新 X..Y..Z..E */

static void UART_Parse_XYZ(char *str);

/* ========== 初始化 ========== */
void UART_Parse_Init(void)
{
    memset(&uart_data, 0, sizeof(uart_data));
    rx_index = 0;
    frame_receiving = 0;
    frame_format = 0;
    uart_rx_byte_count = 0;
    uart_rx_frame_count = 0;
    uart_rx_error_count = 0;
    uart_last_rx_byte = 0;
}

/* ========== 逐字节解析 (UART4 中断调用) ========== */
void UART_Parse_Process(uint8_t data)
{
    char ch = (char)data;

    /* 新协议以 X 开始，任意位置的 X 都可立即重新同步。 */
    if (ch == 'X') {
        rx_index = 0;
        rx_buf[rx_index++] = ch;
        frame_receiving = 1;
        frame_format = 2;
        return;
    }

    /* 保留旧协议兼容。 */
    if (ch == '$') {
        rx_index = 0;
        rx_buf[rx_index++] = ch;
        frame_receiving = 1;
        frame_format = 1;
        return;
    }

    if (!frame_receiving) {
        return;
    }

    /* 新协议收到 E 即完成，不依赖回车换行。 */
    if (frame_format == 2U && ch == 'E') {
        rx_buf[rx_index] = '\0';
        UART_Parse_XYZ(rx_buf);
        rx_index = 0;
        frame_receiving = 0;
        frame_format = 0;
        return;
    }

    /* 旧协议同时兼容 LF、CR 和 CRLF。 */
    if (ch == '\n' || ch == '\r') {
        if (frame_format == 1U && rx_index >= 3U) {
            rx_buf[rx_index] = '\0';
            UART_Parse_String(rx_buf);
        } else if (frame_format == 2U) {
            uart_rx_error_count++;
        }
        rx_index = 0;
        frame_receiving = 0;
        frame_format = 0;
        return;
    }

    if (rx_index < RX_BUF_SIZE - 1U) {
        rx_buf[rx_index++] = ch;
    } else {
        rx_index = 0;
        frame_receiving = 0;
        frame_format = 0;
        uart_rx_error_count++;
    }
}

/* 新协议: X<dx>Y<dy>Z<found>，帧尾 E 由接收状态机去除。 */
static void UART_Parse_XYZ(char *str)
{
    char *ptr;
    char *end;
    long dx;
    long dy;
    long found;

    if (str[0] != 'X') {
        uart_rx_error_count++;
        return;
    }

    ptr = &str[1];
    dx = strtol(ptr, &end, 10);
    if (end == ptr || *end != 'Y') {
        uart_rx_error_count++;
        return;
    }

    ptr = end + 1;
    dy = strtol(ptr, &end, 10);
    if (end == ptr || *end != 'Z') {
        uart_rx_error_count++;
        return;
    }

    ptr = end + 1;
    found = strtol(ptr, &end, 10);
    if (end == ptr || *end != '\0') {
        uart_rx_error_count++;
        return;
    }

    if (dx > 32767L) dx = 32767L;
    if (dx < -32768L) dx = -32768L;
    if (dy > 32767L) dy = 32767L;
    if (dy < -32768L) dy = -32768L;

    uart_data.type = 'A';
    uart_data.err_x = (int16_t)dx;
    uart_data.err_y = (int16_t)dy;
    uart_data.status = found ? 1U : 0U;
    uart_data.updated = 1;
    uart_rx_frame_count++;
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
    if (str[0] != '$') {
        uart_rx_error_count++;
        return;
    }

    char type = str[1];
    int16_t p1 = 0, p2 = 0, p3 = 0;

    /* 跳过 "$X," 找到第一个参数 */
    char *ptr = str + 2;
    if (*ptr == ',') ptr++;
    else {
        uart_rx_error_count++;
        return;
    }

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
