#ifndef _LanYa_H_
#define _LanYa_H_
#include "stdio.h"
#include "main.h"
void WJ_lanya_Sendbyte(uint8_t byte);
extern uint8_t lanya_data;
extern volatile float remote_angle;	/* 蓝牙遥控: 叠加到角度环的目标倾角(度) */
extern volatile float remote_diff;	/* 蓝牙遥控: 左右轮差速 PWM */
extern volatile float remote_speed;	/* 蓝牙遥控: 速度环目标(rpm) */

/* 遥控失联保护:
 * $f/$b/$l/$r 是"按住式"遥控, 一旦蓝牙断开或 App 被杀, 不能让最后一条
 * 命令永久生效(否则车会一直朝那个方向冲)。
 * 控制环每隔一段时间调一次 lanya_remote_watchdog(), 太久没收到任何字节
 * 就清掉 remote_angle / remote_diff。
 * remote_speed 是"设定速度"语义($v,200 设好后不用一直发), 不做超时清空。 */
void lanya_remote_watchdog(void);
#endif
