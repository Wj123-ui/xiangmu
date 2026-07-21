#include "xunji.h"
#include "PID.h"
volatile uint16_t xunji_yanshi=0;//转弯延时计数
volatile uint8_t xunji_yanshi_flag=0;//转弯开始标志位,1为执行计数
extern float icm_ypr[3];
#define P1 DL_GPIO_readPins(GPIO_xunji_PIN_1_PORT, GPIO_xunji_PIN_1_PIN)
#define P2 DL_GPIO_readPins(GPIO_xunji_PIN_2_PORT, GPIO_xunji_PIN_2_PIN)
#define P3 DL_GPIO_readPins(GPIO_xunji_PIN_3_PORT, GPIO_xunji_PIN_3_PIN)
#define P4 DL_GPIO_readPins(GPIO_xunji_PIN_4_PORT, GPIO_xunji_PIN_4_PIN)
#define P5 DL_GPIO_readPins(GPIO_xunji_PIN_5_PORT, GPIO_xunji_PIN_5_PIN)
#define P6 DL_GPIO_readPins(GPIO_xunji_PIN_6_PORT, GPIO_xunji_PIN_6_PIN)
#define P7 DL_GPIO_readPins(GPIO_xunji_PIN_7_PORT, GPIO_xunji_PIN_7_PIN)
int target_angle=0;//目标角度
uint8_t xunji_flag=0;//循迹标志位,0为普通弯道,1为直角弯
uint8_t xunji_angle_counter=0;//第几个直角弯
static uint8_t xunji_corner_latched=0;//是否处于直角弯的标志位

static float WJ_angle_error(float target, float current)//目标角度误差,归一化处理
{
	float error=target-current;

	while(error>180.0f)
	{
		error-=360.0f;
	}
	while(error<-180.0f)
	{
		error+=360.0f;
	}
	return error;
}

float WJ_xunji_fangxing(void)//普通弯道循迹
{
    if(P4!=0)           // 中心
	{
		return 0;      
	}
	else if(P3!=0)      // 左1
	{
		return -30;
	}
	else if(P5!=0)      // 右1
	{
		return 30;
	}
	else if(P2!=0)      // 左2
	{
		return -45;
	}
	else if(P6!=0)      // 右2
	{
		return 45;
	}
	else if(P1!=0)      // 最左
	{
		return -60;     
	}
	else if(P7!=0)      // 最右
	{
		return 60;      
	}
    return 0;
}
void xunji_all(void)//直角弯循迹
{
	uint8_t corner_detected=(P1!=0 && P2!=0 && P3!=0);
	float angle_error;

	/* 离开上一个直角区域后，才允许识别下一个直角。 */
	if((xunji_flag==0) && (corner_detected==0))
	{
		xunji_corner_latched=0;
	}

	/* 每个直角只触发一次，避免主循环反复累加角点计数。 */
	if((xunji_flag==0) && (corner_detected!=0) &&
	   (xunji_corner_latched==0))
	{
		xunji_corner_latched=1;
		xunji_flag=1;
		xunji_yanshi=0;
		xunji_yanshi_flag=1;

		xunji_angle_counter=(uint8_t)((xunji_angle_counter+1U)%4U);
		switch(xunji_angle_counter)
		{
			case 0:
				target_angle=0;
				break;
			case 1:
				target_angle=-90;
				break;
			case 2:
				target_angle=-180;
				break;
			default: /* case 3，航向角越过 -180 度后变为正角度 */
				target_angle=90;
				break;
		}
	}

	/* 前行延时结束后才判断转弯完成，角度误差兼容 +/-180 度跳变。 */
	if((xunji_flag==1) && (xunji_yanshi>=200U))
	{
		angle_error=WJ_angle_error((float)target_angle,icm_ypr[0]);
		if((angle_error>=-2.0f) && (angle_error<=2.0f))
		{
			xunji_flag=0;
			xunji_yanshi_flag=0;
			xunji_yanshi=0;
		}
	}
}
