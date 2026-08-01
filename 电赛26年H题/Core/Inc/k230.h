#ifndef K230_H
#define K230_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * K230 视觉串口接收模块。
 * K230 按“X<dx>Y<dy>Z<found>E”格式发送目标位置，STM32 采用
 * 单字节串口中断接收并在后台完成解析，主循环只读取完整的新数据帧。
 */
typedef struct {
    int16_t dx;       /* 目标相对画面中心的水平偏差，当前允许范围 -500~500。 */
    int16_t dy;       /* 目标相对画面中心的垂直偏差，当前允许范围 -500~500。 */
    uint8_t found;    /* 是否识别到目标：1=识别到，0=未识别到。 */
} K230_Frame;

/* 绑定 K230 所用串口并启动 1 字节中断接收。 */
void K230_Init(UART_HandleTypeDef *huart);

/*
 * 非阻塞读取最新完整帧。
 * last_sequence 由调用者保存；只有收到比它更新的帧时才返回 true。
 */
bool K230_GetNewFrame(K230_Frame *frame, uint32_t *last_sequence);

/* 在 HAL_UART_RxCpltCallback 中转调，用于解析收到的单个字节。 */
void K230_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* 在 HAL_UART_ErrorCallback 中转调，用于清除错误并恢复接收。 */
void K230_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
