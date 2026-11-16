#include "JY61P.h"
uint8_t JY61P_Data[11]={0};
uint8_t JY61P_state=0;
static uint8_t n=0;
float Roll;
float Pitch;
float Yaw;
volatile uint32_t JY61P_FrameCount=0;
void JY61P_jiesuo(void)
{
	static uint8_t a[5]={0XFF,0XAA,0X69,0X88,0XB5};
	HAL_UART_Transmit(&huart1,a,5,100);
}
void JY61P_save(void)
{
	static uint8_t a[5]={0XFF,0XAA,0X00,0X00,0X00};
	HAL_UART_Transmit(&huart1,a,5,100);
}
void JY61P_Sendcmd(uint8_t cmd)
{
	HAL_UART_Transmit(&huart1,&cmd,1,100);
}
void JY61P_Init(void)
{
	JY61P_jiesuo();
	HAL_Delay(200);
	JY61P_Sendcmd(0xff);
	JY61P_Sendcmd(0xaa);
	JY61P_Sendcmd(0x01);
	JY61P_Sendcmd(0x08);
	JY61P_Sendcmd(0x00);
	HAL_Delay(3000);
	JY61P_save();
	
	JY61P_jiesuo();
	HAL_Delay(200);
	JY61P_Sendcmd(0xff);
	JY61P_Sendcmd(0xaa);
	JY61P_Sendcmd(0x01);
	JY61P_Sendcmd(0x04);
	JY61P_Sendcmd(0x00);
	HAL_Delay(3000);
	JY61P_save();
}
void JY61P_callback(void)
{
	switch(JY61P_state)
	{
		case 0:
			if(JY61P_Data[0]==0x55)
			{	n=1;
				JY61P_state=1;}
			else
			{n=0;JY61P_state=0;}
			break;
		case 1:
			if(JY61P_Data[1]==0x53)
			{	n=2;
				JY61P_state=2;}
			else
			{n=0;JY61P_state=0;}
			break;
		case 2: n++;
			if(n>=10)
			{
				JY61P_state=3;
			}
			break;
		case 3:  if(JY61P_Data[10]==(uint8_t)(0x55+0x53+JY61P_Data[2]+JY61P_Data[3]+JY61P_Data[4]+JY61P_Data[5]+JY61P_Data[6]+JY61P_Data[7]+JY61P_Data[8]+JY61P_Data[9]))
		{
			Roll=((float)((uint16_t)JY61P_Data[2]|((uint16_t)JY61P_Data[3])<<8))/32768*180;
			Pitch=((float)((uint16_t)JY61P_Data[4]|((uint16_t)JY61P_Data[5])<<8))/32768*180;
			Yaw=((float)((uint16_t)JY61P_Data[6]|((uint16_t)JY61P_Data[7])<<8))/32768*180;
			if(Pitch>=180)
			{
				Pitch=Pitch-360;
			}
			JY61P_FrameCount++;		/* 校验通过, 更新有效帧计数(控制环的看门狗) */
			n=0;JY61P_state=0;
		}
		else
		{
			n=0;JY61P_state=0;
		}
	}
	HAL_UART_Receive_IT(&huart1,&JY61P_Data[n],1);
}

/* 串口出错(ORE/FE/NE)后调用: 丢掉半帧, 重新从 0x55 帧头开始收
 * 注意 huart1 的 RxState 此时已被 UART_EndRxTransfer 置回 READY */
void JY61P_Reset(void)
{
	n = 0;
	JY61P_state = 0;
	JY61P_Data[0] = 0;
	HAL_UART_Receive_IT(&huart1, &JY61P_Data[0], 1);
}
