#include "k230.h"

/* 钢球识别脚本以 800 像素宽画面中心为零点，dx 合理范围约为 -400～400。 */
#define K230_DX_ABS_LIMIT 500
#define K230_DY_ABS_LIMIT 500

/*
 * K230 串口帧格式：X<dx>Y<dy>Z<found>E。
 * 示例“X-12Y35Z1E”表示目标横向偏差 -12、纵向偏差 35，且识别有效。
 * 解析在串口接收完成中断中逐字节进行，不会阻塞主循环。
 */
typedef enum {
    K230_WAIT_X = 0,  /* 等待一帧的起始字符 X。 */
    K230_READ_DX,     /* 读取有符号水平偏差，直到遇到 Y。 */
    K230_READ_DY,     /* 读取有符号垂直偏差，直到遇到 Z。 */
    K230_READ_FOUND,  /* 读取识别标志 0 或 1。 */
    K230_WAIT_E       /* 等待帧结束字符 E。 */
} K230_ParseState;

static UART_HandleTypeDef *k230_uart;        /* K230 当前绑定的 UART 句柄。 */
static uint8_t rx_byte;                     /* 中断每次接收的一个字节。 */
static volatile K230_Frame latest_frame;    /* 最近一次校验通过的完整数据帧。 */
static volatile uint32_t frame_sequence;    /* 完整帧序号，用于判断是否有新数据。 */
static K230_ParseState parse_state;          /* 当前字节解析状态。 */
static int16_t parse_value;                 /* 当前数字字段的绝对值累加器。 */
static int16_t parse_dx;                    /* 正在解析的数据帧中的 dx。 */
static int16_t parse_dy;                    /* 正在解析的数据帧中的 dy。 */
static int8_t parse_sign;                   /* 当前数字字段的符号：1 或 -1。 */
static uint8_t parse_found;                 /* 正在解析的数据帧中的识别标志。 */
static bool parse_has_digit;                /* 当前数字字段是否已收到至少一位数字。 */

/* 收到起始字符 X 后，清空数字缓存并进入 dx 读取状态。 */
static void K230_StartFrame(void)
{
    parse_state = K230_READ_DX;
    parse_value = 0;
    parse_sign = 1;
    parse_has_digit = false;
}

/*
 * 当前帧格式错误时回到等待状态；若错误字节本身就是 X，立即把它作为
 * 下一帧的开头，从而尽快恢复同步。
 */
static void K230_ResetOrRestart(uint8_t data)
{
    parse_state = K230_WAIT_X;
    if (data == (uint8_t)'X') {
        K230_StartFrame();
    }
}

/* 将符号或十进制数字追加到当前字段，并检查该字段的允许上限。 */
static bool K230_AppendNumber(uint8_t data, int16_t limit)
{
    if ((!parse_has_digit) && (data == (uint8_t)'-')) {
        parse_sign = -1;
        return true;
    }
    if ((data < (uint8_t)'0') || (data > (uint8_t)'9')) {
        return false;
    }
    parse_has_digit = true;
    parse_value = (int16_t)(parse_value * 10 + (int16_t)(data - (uint8_t)'0'));
    return parse_value <= limit;
}

/* 根据当前状态消费一个串口字节，只有完整收到 E 后才发布新数据帧。 */
static void K230_ParseByte(uint8_t data)
{
    switch (parse_state) {
    case K230_WAIT_X:
        if (data == (uint8_t)'X') K230_StartFrame();
        break;
    case K230_READ_DX:
        if (data == (uint8_t)'Y') {
            if (!parse_has_digit) { K230_ResetOrRestart(data); break; }
            parse_dx = (int16_t)(parse_sign * parse_value);
            parse_state = K230_READ_DY;
            parse_value = 0; parse_sign = 1; parse_has_digit = false;
        } else if (!K230_AppendNumber(data, K230_DX_ABS_LIMIT)) {
            K230_ResetOrRestart(data);
        }
        break;
    case K230_READ_DY:
        if (data == (uint8_t)'Z') {
            if (!parse_has_digit) { K230_ResetOrRestart(data); break; }
            parse_dy = (int16_t)(parse_sign * parse_value);
            parse_state = K230_READ_FOUND;
            parse_value = 0; parse_sign = 1; parse_has_digit = false;
        } else if (!K230_AppendNumber(data, K230_DY_ABS_LIMIT)) {
            K230_ResetOrRestart(data);
        }
        break;
    case K230_READ_FOUND:
        if ((data == (uint8_t)'0') || (data == (uint8_t)'1')) {
            parse_found = (uint8_t)(data - (uint8_t)'0');
            parse_state = K230_WAIT_E;
        } else {
            K230_ResetOrRestart(data);
        }
        break;
    case K230_WAIT_E:
        if (data == (uint8_t)'E') {
            /* 先更新全部字段，最后递增序号，保证主循环只看到完整帧。 */
            latest_frame.dx = parse_dx;
            latest_frame.dy = parse_dy;
            latest_frame.found = parse_found;
            frame_sequence++;
            parse_state = K230_WAIT_X;
        } else {
            K230_ResetOrRestart(data);
        }
        break;
    default:
        parse_state = K230_WAIT_X;
        break;
    }
}

/* 初始化解析状态，并开启 UART 单字节中断接收。 */
void K230_Init(UART_HandleTypeDef *huart)
{
    k230_uart = huart;
    parse_state = K230_WAIT_X;
    frame_sequence = 0U;
    if (k230_uart != NULL) {
        HAL_UART_Receive_IT(k230_uart, &rx_byte, 1U);
    }
}

/*
 * 使用“读取序号—复制数据—再次读取序号”的方式避免中断更新到一半时
 * 主循环取得混合数据；若复制期间来了新帧，则重新复制最新一帧。
 */
bool K230_GetNewFrame(K230_Frame *frame, uint32_t *last_sequence)
{
    uint32_t before;
    uint32_t after;

    if ((frame == NULL) || (last_sequence == NULL)) return false;
    do {
        before = frame_sequence;
        if ((before == 0U) || (before == *last_sequence)) return false;
        frame->dx = latest_frame.dx;
        frame->dy = latest_frame.dy;
        frame->found = latest_frame.found;
        after = frame_sequence;
    } while (before != after);
    *last_sequence = after;
    return true;
}

/* UART 接收完成回调：解析本字节后立即挂起下一字节接收。 */
void K230_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((k230_uart != NULL) && (huart->Instance == k230_uart->Instance)) {
        K230_ParseByte(rx_byte);
        HAL_UART_Receive_IT(k230_uart, &rx_byte, 1U);
    }
}

/* UART 异常恢复：清除溢出标志并重新启动单字节接收。 */
void K230_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((k230_uart != NULL) && (huart->Instance == k230_uart->Instance)) {
        __HAL_UART_CLEAR_OREFLAG(k230_uart);
        HAL_UART_Receive_IT(k230_uart, &rx_byte, 1U);
    }
}
