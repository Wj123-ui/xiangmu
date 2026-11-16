#include "stdio.h"
#include "main.h"
#include "lanya.h"
#include "JY61P.h"
#include "PID.h"
uint8_t lanya_data=0;
void WJ_lanya_Sendbyte(uint8_t byte)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&byte, 1,100); 
}
int fputc(int ch, FILE *f) 
{ 
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,100); 
    return (ch); 
}

/* ======================== 蓝牙遥控状态 ========================
 * 由 ISR 更新, 主循环/TIM1 中断只读, 单字节写入本身是原子的
 */
#define REMOTE_ANGLE_MAX   8.0f		/* 目标倾角最大 8 度, 防止冲太猛 */
#define REMOTE_DIFF_MAX  400.0f		/* 差速最大 400, 占满 PWM 1000 的 40% */
#define REMOTE_SPEED_MAX 300.0f		/* 速度环目标最大 300rpm */
#define REMOTE_ANGLE_STEP  2.5f		/* 不带数值时的默认倾角(度) */
#define REMOTE_DIFF_STEP 200.0f		/* 不带数值时的默认差速 */
#define REMOTE_GO_SIGN     1.0f		/* 前进/后退方向反了就改 -1.0f */
#define REMOTE_TURN_SIGN   1.0f		/* 左右转方向反了就改 -1.0f */

/* 遥控链路超时(ms): 这么久没收到任何字节, 就认为手机断开/App 被杀,
 * 把 remote_angle 和 remote_diff 清掉, 车回到直立, 不再朝最后的方向跑。
 *
 * 注意语义变化: 超时后 $f/$b/$l/$r 是"活动的", 必须持续发(或由 App 定时重发)
 * 才能维持, 单发一次只会生效 REMOTE_LINK_TIMEOUT 这么久。$s 仍然立即停。
 * 500ms 偏安全: 断开后 0.5 秒内必停, 但单发一次 $f 只跑 0.5 秒,
 * 前进/转向要靠 App 按住连发(间隔 <500ms)。
 *
 * remote_speed 是"设定速度"语义($v,200 设好后不用一直发), 不做超时清空。 */
#define REMOTE_LINK_TIMEOUT  500U

volatile float remote_angle = 0.0f;	/* 叠加到角度环目标上的倾角(度) */
volatile float remote_diff  = 0.0f;	/* 左右轮差速 PWM */
volatile float remote_speed = 0.0f;	/* 速度环目标(rpm) */

/* 最近一次收到蓝牙字节的时刻(HAL_GetTick), 由接收中断更新 */
static volatile uint32_t lanya_last_rx_tick = 0;
static volatile uint8_t  lanya_rx_alive = 0;	/* 0 = 还没收到过任何字节 */

/* 遥控失联保护, 由控制环周期调用(Timer.c 里每 10ms 一次)
 * 超过 REMOTE_LINK_TIMEOUT 没收到字节, 清掉"按住式"的两个遥控量。
 * 用减法比较, 天然兼容 HAL_GetTick 回绕(~49 天)。 */
void lanya_remote_watchdog(void)
{
	if (lanya_rx_alive == 0) { return; }		/* 从没连过, 不用管 */

	if ((uint32_t)(HAL_GetTick() - lanya_last_rx_tick) > REMOTE_LINK_TIMEOUT)
	{
		remote_angle = 0.0f;
		remote_diff  = 0.0f;
	}
}

/* 数值转 PWM/角度, 并做限幅 */
static float lanya_clamp(float v, float lim)
{
	if (v >  lim) { return  lim; }
	if (v < -lim) { return -lim; }
	return v;
}

/* ======================== 蓝牙调参 ========================
 * 角度环:  $a,kp,ki,kd\r\n   例如 $a,15,0.4,56\r\n
 * 速度环:  $v,kp,ki,kd\r\n   例如 $v,0.8,0.05,0\r\n
 * 差速环:  $d,kp,ki,kd\r\n   例如 $d,2,0.05,0\r\n
 * 旧写法:  $kp,ki,kd\r\n     不带前缀 = 调角度环(兼容)
 * 收满一整行后写入对应 PID, 并回 "$OK\r\n" 或 "$ERR\r\n"
 * 不用 sscanf/atof: Keil 精简版 libc 不支持浮点输入
 *
 * ======================== 蓝牙遥控 ========================
 * 单字母命令(以 '\r' 或 '\n' 结尾), 可带数值:
 *   $f      前进        $f,2.5   前进, 目标倾角 2.5 度
 *   $b      后退        $b,2.5   后退, 目标倾角 2.5 度
 *   $l      左转        $l,200   左转, 目标转速差(=200*0.15=30rpm)
 *   $r      右转        $r,200
 *   $v,200  速度环目标 200rpm     $v,0 或 $s 停止
 *   $s      停止(倾角、差速、目标速度都清零)
 * 前进后退 = 给角度环叠加目标倾角, 车靠倾斜驱动(闭环, 能测角度环)
 * 左右转   = 给差速环目标转速差, 差速环输出 PWM 叠加到左右轮(闭环)
 * 方向和设想相反时, 改上面 REMOTE_GO_SIGN / REMOTE_TURN_SIGN 的符号
 *
 * 失联保护: $f/$b/$l/$r 是"按住式"的, 超过 REMOTE_LINK_TIMEOUT 没收到
 * 任何字节就自动清零(见 lanya_remote_watchdog)。所以要维持前进/转向,
 * App 需要持续发或定时重发; 一旦断开, 车会自己回正而不是一直冲。
 * $v 设的速度不受影响。
 */

#define LANYA_BUF_SIZE 32

static char    lanya_buf[LANYA_BUF_SIZE];
static uint8_t lanya_len     = 0;
static uint8_t lanya_inframe = 0;

static void lanya_sendstr(const char *s)
{
	uint16_t n = 0;
	while (s[n] != '\0') { n++; }
	HAL_UART_Transmit(&huart2, (uint8_t *)s, n, 100);
}

/**
 * @brief 解析一个十进制浮点数(可带符号和小数点)
 * @param s 起始地址
 * @param out 解析结果
 * @retval 数字之后的第一个字符, 解析失败返回 0
 */
static char *lanya_atof(char *s, float *out)
{
	float   v      = 0.0f;
	float   frac   = 0.1f;
	uint8_t neg    = 0;
	uint8_t digits = 0;

	while (*s == ' ' || *s == '\t') { s++; }
	if (*s == '+' || *s == '-')
	{
		neg = (*s == '-');
		s++;
	}
	while (*s >= '0' && *s <= '9')
	{
		v = v * 10.0f + (float)(*s - '0');
		s++;
		digits = 1;
	}
	if (*s == '.')
	{
		s++;
		while (*s >= '0' && *s <= '9')
		{
			v += (float)(*s - '0') * frac;
			frac *= 0.1f;
			s++;
			digits = 1;
		}
	}
	if (digits == 0) { return 0; }

	*out = neg ? -v : v;
	return s;
}

/**
 * @brief 解析 "kp,ki,kd" 并写入指定 PID
 * @param s "kp,ki,kd"
 * @param p 目标 PID
 * @retval 1 成功, 0 格式错误
 */
static uint8_t lanya_parse_pid(char *s, PID_t *p)
{
	float kp, ki, kd;
	char *q;

	q = lanya_atof(s, &kp);
	if (q == 0 || *q != ',') { return 0; }

	q = lanya_atof(q + 1, &ki);
	if (q == 0 || *q != ',') { return 0; }

	q = lanya_atof(q + 1, &kd);
	if (q == 0) { return 0; }
	if (*q != '\0' && *q != ',') { return 0; }	/* 兼容 "$kp,ki,kd," 带尾逗号的写法 */

	p->Kp = kp;
	p->Ki = ki;
	p->Kd = kd;

	/* 速度环积分限幅随 Ki 自动重算: IntMax = 0.6*OutMax/Ki
	 * 保证积分项最多顶到 OutMax 的 60%, 不会顶死在限幅上 */
	if (p == &speed || p == &dif)
	{
		p->IntMax = (ki > 0.0f) ? (0.6f * p->OutMax / ki) : 0.0f;
	}
	return 1;
}

/**
 * @brief 解析遥控命令 "$f" / "$f,2.5" 的数值部分
 * @param s 整行(已去掉 '$'), s[0] 是命令字母
 * @param dflt 不带数值时用的默认值
 * @param out 解析结果
 * @retval 1 成功, 0 格式错误
 */
static uint8_t lanya_parse_cmd(char *s, float dflt, float *out)
{
	char *p;
	float v;

	p = s + 1;
	if (*p == '\0')					/* 只有字母, 用默认值 */
	{
		*out = dflt;
		return 1;
	}
	if (*p != ',') { return 0; }

	p = lanya_atof(p + 1, &v);
	if (p == 0) { return 0; }
	if (*p != '\0' && *p != ',') { return 0; }

	*out = v;
	return 1;
}

/* 处理一整行('$' 和行尾已去掉) */
static void lanya_handle_line(char *line)
{
	float v;

	switch (line[0])
	{
		case 'a':	/* 角度环调参: $a,kp,ki,kd 或简写 $a2 无效, 必须带逗号 */
			if (line[1] == ',')
			{
				if (lanya_parse_pid(line + 2, &angle)) { lanya_sendstr("$OK\r\n"); }
				else                                   { lanya_sendstr("$ERR\r\n"); }
				return;
			}
			break;
		case 'v':	/* 速度环调参: $v,kp,ki,kd   目标速度: $v,200 */
			if (line[1] == ',')
			{
				char *q;
				float t;
				uint8_t has_comma = 0;

				/* 有三个数就是调参, 只有一个数就是设目标速度 */
				for (q = line + 2; *q != '\0'; q++)
				{
					if (*q == ',') { has_comma = 1; break; }
				}
				if (has_comma)
				{
					if (lanya_parse_pid(line + 2, &speed)) { lanya_sendstr("$OK\r\n"); }
					else                                   { lanya_sendstr("$ERR\r\n"); }
					return;
				}
				if (lanya_atof(line + 2, &t) != 0)
				{
					remote_speed = lanya_clamp(t, REMOTE_SPEED_MAX);
					lanya_sendstr("$OK\r\n");
					return;
				}
			}
			break;
		case 'd':	/* 差速环调参: $d,kp,ki,kd */
			if (line[1] == ',')
			{
				if (lanya_parse_pid(line + 2, &dif)) { lanya_sendstr("$OK\r\n"); }
				else                                 { lanya_sendstr("$ERR\r\n"); }
				return;
			}
			break;
		case 'f':
			if (lanya_parse_cmd(line, REMOTE_ANGLE_STEP, &v) == 0) { break; }
			remote_angle =  REMOTE_GO_SIGN * lanya_clamp(v, REMOTE_ANGLE_MAX);
			lanya_sendstr("$OK\r\n");
			return;
		case 'b':
			if (lanya_parse_cmd(line, REMOTE_ANGLE_STEP, &v) == 0) { break; }
			remote_angle = -REMOTE_GO_SIGN * lanya_clamp(v, REMOTE_ANGLE_MAX);
			lanya_sendstr("$OK\r\n");
			return;
		case 'l':
			if (lanya_parse_cmd(line, REMOTE_DIFF_STEP, &v) == 0) { break; }
			remote_diff =  REMOTE_TURN_SIGN * lanya_clamp(v, REMOTE_DIFF_MAX);
			lanya_sendstr("$OK\r\n");
			return;
		case 'r':
			if (lanya_parse_cmd(line, REMOTE_DIFF_STEP, &v) == 0) { break; }
			remote_diff = -REMOTE_TURN_SIGN * lanya_clamp(v, REMOTE_DIFF_MAX);
			lanya_sendstr("$OK\r\n");
			return;
		case 's':
			remote_angle = 0.0f;
			remote_diff  = 0.0f;
			remote_speed = 0.0f;
			lanya_sendstr("$OK\r\n");
			return;
		default:
			break;
	}

	/* 兼容旧写法: $kp,ki,kd 默认调角度环 */
	if (lanya_parse_pid(line, &angle)) { lanya_sendstr("$OK\r\n"); }
	else                               { lanya_sendstr("$ERR\r\n"); }
}

/* 逐字节收: '$' 开始, '\r' 或 '\n' 结束 */
static void lanya_rx_byte(uint8_t byte)
{
	/* 只要收到任何字节就认为链路还活着(不管是不是合法命令),
	 * 失联判定看的是"有没有数据", 不是"命令对不对" */
	lanya_last_rx_tick = HAL_GetTick();
	lanya_rx_alive = 1;

	if (byte == '$')
	{
		lanya_inframe = 1;
		lanya_len = 0;
		return;
	}
	if (lanya_inframe == 0) { return; }

	if (byte == '\r' || byte == '\n')
	{
		if (lanya_len > 0)				/* 空行忽略, "\r\n" 只算一次 */
		{
			lanya_buf[lanya_len] = '\0';
			lanya_inframe = 0;
			lanya_handle_line(lanya_buf);
			lanya_len = 0;
		}
		return;
	}

	if (lanya_len < LANYA_BUF_SIZE - 1)
	{
		lanya_buf[lanya_len] = (char)byte;
		lanya_len++;
	}
	else								/* 超长帧丢弃 */
	{
		lanya_inframe = 0;
		lanya_len = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(huart,&lanya_data,1); // 重启下一次接收
        lanya_rx_byte(lanya_data);                // 处理 rx_data
    }
	else if(huart->Instance == USART1)
	{
		JY61P_callback();
	}
}

/* 串口错误恢复:
 * 中断里同步发回执($OK)时, 这段时间来不了新的 RXNE; 只要对端在回执期间
 * 又发进 1~2 个字节就会置 ORE(溢出)。HAL 检测到 ORE 会执行
 * UART_EndRxTransfer() 关掉 RXNEIE, 如果不在这里重新武装接收,
 * 蓝牙就从那一刻起永久失联, 只能复位。
 *
 * 只有 ORE 是"阻塞型"错误(HAL 已中止接收), 必须重新武装;
 * FE/NE 是非阻塞的, 接收仍在进行, 这时只需要丢掉可能被截断的半帧。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint8_t overrun = (huart->ErrorCode & HAL_UART_ERROR_ORE) ? 1 : 0;

	if (huart->Instance == USART2)
	{
		lanya_inframe = 0;			/* 出错时的字节可能把一帧切断, 丢弃半帧 */
		lanya_len = 0;
		if (overrun)
		{
			__HAL_UART_CLEAR_OREFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			HAL_UART_Receive_IT(huart, &lanya_data, 1);
		}
	}
	else if (huart->Instance == USART1)
	{
		if (overrun)
		{
			__HAL_UART_CLEAR_OREFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			JY61P_Reset();			/* 帧同步丢失, 重新找 0x55 0x53 帧头 */
		}
		/* 非 ORE: 接收仍在进行, 让校验和去丢掉坏帧, 不要动接收指针 */
	}
}
