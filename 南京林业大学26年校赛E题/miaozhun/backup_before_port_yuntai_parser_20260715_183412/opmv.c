/*
 * opmv.c — K230 视觉通信实现 (UART4, 115200)
 *
 * 协议解析:
 *   主协议: X<dx>Y<dy>Z<found>E\r\n
 *   发送: #<task_id>\n
 *
 * 解析状态机:
 *   WAIT_X -> READ_X -> READ_Y -> READ_Z -> 完成
 */
#include "usart_parse.h"
#include "opmv.h"
#include <string.h>

/* ========== 全局变量 ========== */
UART_Data_t uart_data = {0, 0, 0, 0, 0};
volatile uint8_t rx_byte = 0;
volatile uint32_t uart_rx_byte_count = 0;
volatile uint32_t uart_rx_frame_count = 0;
volatile uint32_t uart_rx_error_count = 0;
volatile uint8_t uart_last_rx_byte = 0;

/* ========== X<dx>Y<dy>Z<found>E 逐字符解析状态 ========== */
typedef enum {
    UART_WAIT_X = 0,
    UART_READ_X,
    UART_READ_Y,
    UART_READ_Z
} UART_ParseState_t;

static UART_ParseState_t parse_state = UART_WAIT_X;
static int32_t parse_value = 0;
static int8_t parse_sign = 1;
static uint8_t parse_has_digit = 0;
static int16_t pending_x = 0;
static int16_t pending_y = 0;

static void UART_NumberReset(void)
{
    parse_value = 0;
    parse_sign = 1;
    parse_has_digit = 0;
}

static int16_t UART_NumberToInt16(void)
{
    int32_t value = parse_value * (int32_t)parse_sign;
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    return (int16_t)value;
}

static void UART_FrameAbort(void)
{
    parse_state = UART_WAIT_X;
    UART_NumberReset();
    uart_rx_error_count++;
}

/* ========== 初始化 ========== */
void UART_Parse_Init(void)
{
    memset(&uart_data, 0, sizeof(uart_data));
    parse_state = UART_WAIT_X;
    UART_NumberReset();
    pending_x = 0;
    pending_y = 0;
    uart_rx_byte_count = 0;
    uart_rx_frame_count = 0;
    uart_rx_error_count = 0;
    uart_last_rx_byte = 0;
}

/* ========== 逐字节解析 (UART4 中断调用) ========== */
void UART_Parse_Process(uint8_t data)
{
    char ch = (char)data;

    /* 任意位置收到 X 都立即开始新帧，断帧后可自动恢复。 */
    if (ch == 'X') {
        parse_state = UART_READ_X;
        UART_NumberReset();
        return;
    }

    if (parse_state == UART_WAIT_X) {
        /* 串口助手附加的 CR/LF 和其他帧外字符直接忽略。 */
        return;
    }

    /* 串口助手允许在字段周围插入空格或 Tab。 */
    if (ch == ' ' || ch == '\t') {
        return;
    }

    if (!parse_has_digit && (ch == '+' || ch == '-')) {
        parse_sign = (ch == '-') ? -1 : 1;
        return;
    }

    if (ch >= '0' && ch <= '9') {
        parse_has_digit = 1;
        /* 限制累加值，避免异常长数据导致 int32_t 溢出。 */
        if (parse_value < 1000000) {
            parse_value = parse_value * 10 + (int32_t)(ch - '0');
        }
        return;
    }

    if (!parse_has_digit) {
        UART_FrameAbort();
        return;
    }

    if (parse_state == UART_READ_X && ch == 'Y') {
        pending_x = UART_NumberToInt16();
        parse_state = UART_READ_Y;
        UART_NumberReset();
        return;
    }

    if (parse_state == UART_READ_Y && ch == 'Z') {
        pending_y = UART_NumberToInt16();
        parse_state = UART_READ_Z;
        UART_NumberReset();
        return;
    }

    if (parse_state == UART_READ_Z && ch == 'E') {
        int16_t found = UART_NumberToInt16();

        /* updated 最后置位，让主循环只读取已经完整发布的一帧。 */
        uart_data.type = 'A';
        uart_data.err_x = pending_x;
        uart_data.err_y = pending_y;
        uart_data.status = (found != 0) ? 1U : 0U;
        uart_data.updated = 1U;
        uart_rx_frame_count++;

        parse_state = UART_WAIT_X;
        UART_NumberReset();
        return;
    }

    UART_FrameAbort();
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
