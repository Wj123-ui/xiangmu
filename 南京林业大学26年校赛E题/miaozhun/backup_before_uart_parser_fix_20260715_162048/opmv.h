/*
 * opmv.h — K230 视觉通信接口 (替代 OpenMV)
 *
 * 通信协议 (UART4, 115200):
 *   STM32 → K230:  #<task_id>\n              (设置任务模式)
 *   K230 → STM32:  $<type>,<p1>,<p2>,<p3>,<p4>\n  (视觉数据)
 *
 *   type='A': 瞄准误差  p1=err_x  p2=err_y           (task 1/2/3/4)
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

/* 初始化与解析 */
void UART_Parse_Init(void);
void UART_Parse_Process(uint8_t data);
void UART_Parse_String(char *str);

/* 向 K230 发送任务模式命令: #<task_id>\n */
void K230_SendTaskMode(uint8_t task_id);

#endif
