/*
 * opmv.h — K230 视觉通信接口 (替代 OpenMV)
 *
 * 通信协议 (UART4, 115200):
 *   STM32 → K230:  #<task_id>\n              (设置任务模式)
 *   K230 → STM32:  X<dx>Y<dy>Z<found>E\r\n
 *
 *   dx/dy = 靶心坐标 - 激光点坐标; found=1找到靶框, 0丢失
 *   新协议解析后内部使用 type='A', status=found，供 task1~4 处理。
 *   type='C': 轮廓点    p1=x      p2=y      p3=status (task 5, 0=继续 1=完成)
 *   type='D': 数字识别  p1=digit                      (task 6, 0~9)
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
    volatile uint8_t status;    /* p3: 状态标志 (0=继续 1=完成) */
    volatile uint8_t updated;   /* 新数据标志 */
} UART_Data_t;

extern UART_Data_t uart_data;
extern volatile uint8_t rx_byte;
extern volatile uint32_t uart_rx_byte_count;   /* UART4 硬件收到的字节数 */
extern volatile uint32_t uart_rx_frame_count;  /* 成功解析的完整帧数 */
extern volatile uint32_t uart_rx_error_count;  /* 溢出/格式/串口错误数 */
extern volatile uint8_t uart_last_rx_byte;     /* 最后一个原始字节 */

/* 初始化与解析 */
void UART_Parse_Init(void);
void UART_Parse_Process(uint8_t data);
void UART_Parse_String(char *str);

/* 向 K230 发送任务模式命令: #<task_id>\n */
void K230_SendTaskMode(uint8_t task_id);

#endif
