#ifndef BOARD_H
#define BOARD_H

#include "main.h"
#include <stdint.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

/* DWT 实现的阻塞微秒延时，参数为无符号微秒数。 */
void Board_DelayUs(uint32_t us);
/* 以下函数用于兼容原驱动库；非正参数会被忽略。 */
void delay_us(int us);
void delay_ms(int ms);
/* 与 delay_us()/delay_ms() 等价的旧接口别名。 */
void delay_1us(int us);
void delay_1ms(int ms);

#endif
