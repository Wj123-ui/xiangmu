/*
 * opmv.h — K230 视觉通信接口 (替代 OpenMV)
 *
 * 通信协议 (UART4, 115200):
 *   STM32 → K230:  #<task_id>\n              (设置任务模式)
 *   K230 → STM32:  X<dx>Y<dy>Z<status>[D<digit>]E\r\n
 *
 *   dx/dy = 目标点 - 画面中心;
 *   status=1完整靶框/精确靶心, 2部分黑框/搜索引导, 0完全丢失。
 *   解析后内部使用 type='A'，供 task1~4 处理。
 *   type='C': 轮廓点    p1=x      p2=y      p3=status (task 5, 0=继续 1=完成)
 *   可选 D 字段: digit=0~9；digit=-1 表示未识别。
 *   type='L': 线条点    p1=x      p2=y      p3=status (task 7, 0=继续 1=完成)
 *   type='R': 就绪
 */
#ifndef __opmv_H
#define __opmv_H

#include "main.h"
#include <stdint.h>

/* K230 视觉数据包 */
typedef struct {
    volatile uint8_t type;      /* 数据类型: 'A'/'C'/'D'/'L'/'R' */
    volatile int16_t err_x;     /* p1: 瞄准误差X / 轮廓X / 线条X / 数字值 */
    volatile int16_t err_y;     /* p2: 瞄准误差Y / 轮廓Y / 线条Y */
    volatile uint8_t status;    /* p3: A帧0=丢失 1=精确 2=部分引导；其他任务自定义 */
    volatile int8_t digit;      /* 可选 D 字段: 0~9有效，-1=无数字 */
    volatile uint8_t updated;   /* 新数据标志 */
} UART_Data_t;

extern UART_Data_t uart_data;
extern volatile uint8_t rx_byte;
extern volatile uint32_t uart_rx_byte_count;   /* UART4 硬件收到的字节数 */
extern volatile uint32_t uart_rx_frame_count;  /* 成功解析的完整帧数 */
extern volatile uint32_t uart_rx_error_count;  /* 溢出/格式/串口错误数 */
extern volatile uint8_t uart_last_rx_byte;     /* 最后一个原始字节 */
extern volatile char uart_rx_preview[32];      /* 最近一帧/当前帧的原始 ASCII */
extern volatile uint8_t uart_rx_preview_len;

/* 初始化与解析 */
void UART_Parse_Init(void);
void UART_Parse_Process(uint8_t data);
void UART_Parse_String(char *str);

/* 向 K230 发送任务模式命令: #<task_id>\n */
void K230_SendTaskMode(uint8_t task_id);

#endif
