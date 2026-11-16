#ifndef _JY61P_H_
#define _JY61P_H_
#include "main.h"
extern uint8_t JY61P_Data[11];
extern float Roll;
extern float Pitch;
extern float Yaw;
extern volatile uint32_t JY61P_FrameCount;	/* 校验通过的有效帧计数, 控制环用它判断 IMU 是否掉线 */
void JY61P_Init(void);
void JY61P_callback(void);
void JY61P_Reset(void);		/* 串口出错后重新同步帧头并重启接收 */
#endif
