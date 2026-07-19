#include "ti_msp_dl_config.h"
#include "OLED.h"
#include "PWM.h"
#include "JY61P.h"
float xiaozhunyaw=0.0;
#define key1 DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_4)
#define key2 DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_5)
#define key3 DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_6)
#define key4 DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_7)
uint8_t key_num,clear=1;
uint8_t eum1,eum2,eum3;
uint8_t flag1,flag2,flag3,flag4;
uint8_t eumlevel;
uint8_t Mode;
uint8_t Key_GetStatus(void)
{
	if (key1==0)
	{
        flag1=1;
		return 1;
	}
	if (key2==0)
	{
		flag2=1;
		return 2;
	}
	if (key3==0)
	{
		flag3=1;
		return 3;
	}
    if (key4==0)
	{
		flag4=1;
		return 4;
	}
	return 0;
}
void tick(void)
{
	static uint8_t count;
	static uint8_t bencistatus,shangcistatus;
	count++;
	if(count>=20)
	{
		count=0;
		shangcistatus=bencistatus;
		bencistatus=Key_GetStatus();
		if(bencistatus==0 && shangcistatus!=0)
		{
			key_num=shangcistatus;
            if(key_num==1 && flag1==1 && eumlevel==0)
        {
            if(eumlevel==0)
		{
			eum1++;
            eum1%=6;
		}
		
            flag1=0;
        }
		 if(key_num==2 && flag2==1)
		 {
			eumlevel++;
			eumlevel%=2;
			flag2=0;
			clear=1;
		 }
        if(key_num==3 && flag3==1)
		 {
			xiaozhunyaw=Yaw;
		 }
		 if(key_num==4 && flag4==1)
		 {
			
		 }
		}
        
	}
}
void eumshow(void)
{
	if(eumlevel==0)
	{
		Mode=0;
		if(clear==1)
		{
			OLED_Clear();
			clear=0;
		}
	switch (eum1)
	{
		case 0:
		OLED_ShowString(1,1,"*Mode1");
		OLED_ShowString(2,1," Mode2");
		OLED_ShowString(3,1," Mode3");
		OLED_ShowString(4,1," Mode4");
		OLED_ShowString(1,7," ShowPID");
		OLED_ShowString(2,7," Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;
		case 1:
		OLED_ShowString(1,1," Mode1");
		OLED_ShowString(2,1,"*Mode2");
		OLED_ShowString(3,1," Mode3");
		OLED_ShowString(4,1," Mode4");
		OLED_ShowString(1,7," ShowPID");
		OLED_ShowString(2,7," Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;
		case 2:
		OLED_ShowString(1,1," Mode1");
		OLED_ShowString(2,1," Mode2");
		OLED_ShowString(3,1,"*Mode3");
		OLED_ShowString(4,1," Mode4");
		OLED_ShowString(1,7," ShowPID");
		OLED_ShowString(2,7," Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;
		case 3:
		OLED_ShowString(1,1," Mode1");
		OLED_ShowString(2,1," Mode2");
		OLED_ShowString(3,1," Mode3");
		OLED_ShowString(4,1,"*Mode4");
		OLED_ShowString(1,7," ShowPID");
		OLED_ShowString(2,7," Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;
		case 4:
		OLED_ShowString(1,1," Mode1");
		OLED_ShowString(2,1," Mode2");
		OLED_ShowString(3,1," Mode3");
		OLED_ShowString(4,1," Mode4");
		OLED_ShowString(1,7,"*ShowPID");
		OLED_ShowString(2,7," Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;
		case 5:
		OLED_ShowString(1,1," Mode1");
		OLED_ShowString(2,1," Mode2");
		OLED_ShowString(3,1," Mode3");
		OLED_ShowString(4,1," Mode4");
		OLED_ShowString(1,7," ShowPID");
		OLED_ShowString(2,7,"*Showsudu");
		OLED_ShowSignedNum(3,7,Yaw,6);
		break;

	}
	}
	else if (eumlevel==1) 
	{
		if(clear==1)
		{
			OLED_Clear();
			clear++;
		}
		if(clear==2)
		{
			OLED_Clear();
			clear=0;
		}
		switch (eum1)
	{
		case 0:
		Mode=1;
		OLED_ShowString(1,1,"Handle 1");
		OLED_ShowSignedNum(2,1,Yaw,6);
		break;
		case 1:
		Mode=2;
		OLED_ShowString(1,1,"Handle 2");
		OLED_ShowSignedNum(2,1,Yaw,6);
		break;
		case 2:
		Mode=3;
		OLED_ShowString(1,1,"Handle 3");
		OLED_ShowSignedNum(2,1,Yaw,6);
		break;
		case 3:
		Mode=4;
		OLED_ShowString(1,1,"Handle 4");
		OLED_ShowSignedNum(2,1,Yaw,6);
		break;
		case 4:
		OLED_ShowString(1,1,"Kp:");
		OLED_ShowString(2,1,"Ki:");
		OLED_ShowString(3,1,"Kd:");
		OLED_ShowString(4,1,"PWMduty:");
		break;
		case 5:
		OLED_ShowString(1,1,"sudu:");
		OLED_ShowSignedNum(1,6,(int32_t)getencoderleft(),6);
		OLED_ShowSignedNum(2,6,(int32_t)getencoderright(),6);
		
		break;
	}
	}
}
void TIMER_Scan_INST_IRQHandler(void)
{
	if(DL_Timer_getPendingInterrupt(TIMER_Scan_INST)==DL_TIMER_IIDX_ZERO)
	{
		tick();
		DL_Timer_clearInterruptStatus(TIMER_Scan_INST,DL_TIMER_IIDX_ZERO);
	}
}