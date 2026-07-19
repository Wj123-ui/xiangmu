#include "ti_msp_dl_config.h"
#include "eum.h"
float Kp_l=0.637;
float Ki_l=0;
float Kd_l=0.1;

float Kp_r=0.6;
float Ki_r=0;
float Kd_r=0.1;
uint8_t i=1,t;
int8_t xiaozhun=0;
uint8_t cv=0;
#define AIN10 DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_31) 
#define AIN11 DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_31) 
#define AIN20 DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_22) 
#define AIN21 DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_22)  

#define BIN10 DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_24) 
#define BIN11 DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_24) 
#define BIN20 DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_12) 
#define BIN21 DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_12) 

#define P1 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_12)
#define P2 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_13)
#define P3 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_14)
#define P4 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_16)
#define P5 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_17)
#define P6 DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_18)
#define P7 DL_GPIO_readPins(GPIOB,DL_GPIO_PIN_8)

volatile uint32_t gpioA,gpioB;
volatile int32_t gEncoderCount_left = 0, gEncoderVal_left0 = 0,gEncoderVal_left1 = 0,gEncoderVal_left2=0;          //左轮编码器计数值；左轮编码器记录值，50ms定时中断输出
volatile int32_t gEncoderCount_right = 0, gEncoderVal_right0 = 0,gEncoderVal_right1 = 0,gEncoderVal_right2=0;        //右轮编码器计数值；右轮编码器记录值，50ms定时中断输出
volatile uint32_t gDebugRightIRQCount = 0;        // 右侧编码器中断调试计数器
float CurrentA, CurrentB;			//编码器测得当前速度
float targetA=0, targetB=0;			//当前目标速度
float Speed_diff; 					//当前差速
int flag=1,n=1,whiteflag=0,whiteflag1=0,whiteflag2=0;         //白色区域计数标志位；白色区域计数值；陀螺仪模式标志位
int timebegin=0,timenum=0;          //陀螺仪模式启动延时标志位；延时计数值
int timebegin1=0,timenum1=0;          //陀螺仪模式启动延时标志位；延时计数值
int timebegin2=0,timenum2=0;          //陀螺仪模式启动延时标志位；延时计数值
int ledbegin=0,lednum=0,ledflag=0,ledflag1=0,ledflag2=0;        //声光模块延时标志位；延时计数值；声光模块启动标志位
float error=180;                                                //两个方向的偏转
int m=0;                                                        //模式切换计数值，用以判断是否跑完全程
int a=0;                                                        //AB模式停止标志位
bool pwmstart=1;
float tim=0;
int timbegin=0,timnum=0;
float xunji(void);
float PWM_Limit(float IN,float max,float min)                  //pwm限幅
{
	float OUT = IN;
	if(OUT > max) OUT = max;
	if(OUT < min) OUT = min;
	return OUT;
} 

void setduty(float duty,uint8_t channel)//设置占空比
{
    uint16_t compareval;
    compareval=(uint16_t)(25.0*duty);
    if(channel==1)
    {
        DL_Timer_setCaptureCompareValue(TIMA0,compareval,DL_TIMER_CC_0_INDEX);
    }
    else if(channel==2)
    {
        DL_Timer_setCaptureCompareValue(TIMA0,compareval,DL_TIMER_CC_1_INDEX);
    }

}
int myabs(int a)//取绝对值函数
{
    if(a>=0)
    {
        return a;
    }
    else if(a<0)
    {
        return -a;
    }
}
void Set_Pwm(float motor_left,float motor_right)//控制左右轮
{
	if(motor_left > 0)	//前进
	{
        AIN11;
        AIN20;
	}
	else				//后退
	{
		AIN10;
        AIN21;
	}
    setduty(myabs(motor_left),1);

	if(motor_right > 0) //前进
	{
		BIN11;
        BIN20;
	}
	else				//后退
	{
        BIN10;
        BIN21;
	}
    setduty(myabs(motor_right),2);
}
float PID_left(float encoder_left,float target_left) 
{
    static float Bias, Last_bias,Last2_bias,Pwm;
    float Increment;
    
    Bias = target_left - encoder_left;
    
    // 增量式PID公式
    Increment = Kp_l * (Bias - Last_bias) + Ki_l * Bias + Kd_l * (Bias - 2 * Last_bias + Last2_bias);
    Pwm += Increment;  // 累加增量
    
    Last2_bias = Last_bias;  // 先保存上上次
    Last_bias = Bias;        // 再保存上次
    
    return PWM_Limit(Pwm, 85, -85);  // 在这里限幅，避免积分饱和                   				                                        					//返回增量值
}
float PID_right(float encoder_right ,float target_right)
{
    static float Bias, Last_bias,Last2_bias,Pwm;
    float Increment;
    
    Bias = target_right - encoder_right;
    
    // 增量式PID公式
    Increment = Kp_r * (Bias - Last_bias) + Ki_r * Bias + Kd_r * (Bias - 2 * Last_bias + Last2_bias);
    Pwm += Increment;  // 累加增量
    
    Last2_bias = Last_bias;  // 先保存上上次
    Last_bias = Bias;        // 再保存上次
    
    return PWM_Limit(Pwm, 85, -85);  // 在这里限幅，避免积分饱和                   				          
}












void Mode1(float Yaw)
{
    Kd_l=0.15;
    Kd_r=0.15;
    float Speed_Middle;				        //中值速度
    float Motor_Left, Motor_Right;            //电机赋值
    float bias=0;
    bias = Yaw;
    Speed_Middle=55;
    if (((P1!=0 || P2!=0 || P3!=0 || P4!=0 || P5!=0 || P6!=0 || P7!=0))&&a==0)
    {
        timbegin=0;
        pwmstart=0;//是mode1PWM停止位
        ledbegin=1;
        a=1;//a是mode1停止位
    }
    if (ledflag==1)
    {
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_27);
    }
    else 
    {
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_27);
    }
    targetA = Speed_Middle+bias;
	targetB = Speed_Middle-bias;
    CurrentA = (float)((gEncoderVal_left2)/3.60f); //left
	CurrentB = (float)((gEncoderVal_right2)/3.60f); //right
    Motor_Left=PID_left(CurrentA,targetA);
	Motor_Right=PID_right(CurrentB,targetB);		//PWM限幅

    if(pwmstart==1)
    {
        Set_Pwm(Motor_Left,Motor_Right);
    }
    else if(pwmstart==0) 
    {
        Set_Pwm(0, 0);
    }



}
void Mode2(float Yaw)
{
    Kd_l=0.15;
    Kd_r=0.15;
    float Speed_Middle=55;				//中值速度
	int Motor_Left, Motor_Right;
    float bias;
    if (ledflag==1)
    {
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_27);
    }
    else 
    {
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_27);
    }
    if (P1==0 && P2==0 && P3==0 && P4==0 && P5==0 && P6==0 && P7==0) {timebegin=1;}  
    else {whiteflag=0;}
    if (whiteflag==1) 
    {
        ledflag2=0;
        if(ledflag1==0)
        {
            ledflag1=1;
            ledbegin=1;
            m++;
        }

        if (n%2==0)
        {
            if(Yaw<0)
            {
                bias = error - myabs(Yaw);
            }
            else
            {
                bias = Yaw - error;
            }
        }
        else if(n%2==1)
        {
            bias = Yaw;
        }
        flag=0;
    }
    else if(whiteflag==0)
    {
        ledflag1=0;
        if(ledflag2==0)
        {
            ledflag2=1;
            ledbegin=1;
            m++;
        }

        if(flag==0)
        {
            flag=1;
            n=n+1;
        }
        bias = xunji();
        whiteflag=0; 
    }
    targetA = Speed_Middle+bias;
	targetB = Speed_Middle-bias;
    CurrentA = (float)((gEncoderVal_left2)/3.60f); //left
	CurrentB = (float)((gEncoderVal_right2)/3.60f); //right
    Motor_Left=PID_left(CurrentA,targetA);
	Motor_Right=PID_right(CurrentB,targetB);		//PWM限幅
    if(m==6) 
    {
        Set_Pwm(0,0);
        timbegin=0;
    }
	else Set_Pwm(Motor_Left, Motor_Right);
     
    

}

void Mode3(float Yaw)
{
    Kd_l=0.15;
    Kd_r=0.15;
    float Speed_Middle=50;				//中值速度
    int Motor_Left, Motor_Right;
    float bias;
    if (ledflag==1)
    {
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_27);
    }
    else 
    {
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_27);
    }
    if (P1==0 && P2==0 && P3==0 && P4==0 && P5==0 && P6==0 && P7==0) timebegin1=1;  
    else whiteflag1=0;

    if (whiteflag1==1) 
    {
        ledflag2=0;
        if(ledflag1==0)
        {
            ledflag1=1;
            ledbegin=1;
            m++;
        }

        if (n%2==0)
        {
            bias = Yaw+109;
        }
        else if(n%2==1)
        {
            bias = Yaw;
        }
        flag=0;
    }
    else if(whiteflag1==0)
    {
        ledflag1=0;
        if(ledflag2==0)
        {
            ledflag2=1;
            ledbegin=1;
            m++;
        }

        if(flag==0)
        {
            flag=1;
            n=n+1;
        }

        if(n%2==1)
        {
            bias = xunji_highspeed_right();
        }
        else if(n%2==0)
        {
            bias = xunji_highspeed_left();
        }
        
        whiteflag1=0;
    }
    targetA = Speed_Middle+bias;
	targetB = Speed_Middle-bias;
    CurrentA = (float)((gEncoderVal_left2)/3.60f); //left
	CurrentB = (float)((gEncoderVal_right2)/3.60f); //right
    Motor_Left=PID_left(CurrentA,targetA);
	Motor_Right=PID_right(CurrentB,targetB);		//PWM限幅
    if(m==6) 
    {
        Set_Pwm(0,0);
        timbegin=0;
    }
	else {Set_Pwm(Motor_Left, Motor_Right);}
}

void Mode4(float Yaw)
{
    Kd_l=0.15;
    Kd_r=0.15;
    float Speed_Middle=55;				//中值速度
    int Motor_Left, Motor_Right;
    float bias;

     if (ledflag==1)
    {
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_27);
    }
    else 
    {
        DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_26);
        DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_27);
    }

    if (P1==0 && P2==0 && P3==0 && P4==0 && P5==0 && P6==0 && P7==0) timebegin2=1;  // 改为7路
    else whiteflag2=0;

    if (whiteflag2==1) 
    {
        ledflag2=0;
        if(ledflag1==0)
        {
            ledflag1=1;
            ledbegin=1;
            m++;
        }

        if (n%2==0)
        {
            bias = Yaw+112.5-t;
            i=0;
        }
        else if(n%2==1)
        {
            if(i==0)
            {
                t+=3;
                cv+=3;
                i=1;
            }
            bias = Yaw-cv;
        }
        flag=0;
    }
    else if(whiteflag2==0)
    {
        ledflag1=0;
        if(ledflag2==0)
        {
            ledflag2=1;
            ledbegin=1;
            m++;
        }

        if(flag==0)
        {
            flag=1;
            n=n+1;
        }
        
        if(n%2==1)
        {
            bias = xunji_lowspeed_right();
        }
        else if(n%2==0)
        {
            bias = xunji_lowspeed_left();
        }

        whiteflag2=0;
    }
    targetA = Speed_Middle+bias;
	targetB = Speed_Middle-bias;
    CurrentA = (float)((gEncoderVal_left2)/3.60f); //left
	CurrentB = (float)((gEncoderVal_right2)/3.60f); //right
    Motor_Left=PID_left(CurrentA,targetA);
	Motor_Right=PID_right(CurrentB,targetB);		//PWM限幅
    if(m==18) 
    {
        Set_Pwm(0,0);
        timbegin=0;
    }
	else {Set_Pwm(Motor_Left, Motor_Right);}
}


































// 7路循迹：P4为中心，P1最左，P7最右
float xunji(void)			//输出差速，左轮速度为middle+x，右轮速度为middle-x
{
    if(P4!=0)           // 中心
	{
		return 0;      
	}
	else if(P3!=0)      // 左1
	{
		return -10;
	}
	else if(P5!=0)      // 右1
	{
		return 10;
	}
	else if(P2!=0)      // 左2
	{
		return -20;
	}
	else if(P6!=0)      // 右2
	{
		return 20;
	}
	else if(P1!=0)      // 最左
	{
		return -28;     
	}
	else if(P7!=0)      // 最右（原P8位置）
	{
		return 28;      
	}
    return 0;
}

// 7路高速左转：P1最左，P4中心
int xunji_highspeed_left(void)			
{
    if(P1!=0 && P2!=0)  // 最左两路
    {
        return -35;    
    }
    else if(P1!=0)      // 最左单路
	{
		return -25;     
	}
    else if(P2!=0)      // 左2
	{
		return -20;     
	}
    else if(P3!=0)      // 左1
	{
		return -10;
	}
    else if(P4!=0)      // 中心
	{
		return 0;      
	}
	return 0;
}

// 7路高速右转：P4中心，P7最右
int xunji_highspeed_right(void)			
{
    if(P7!=0 && P6!=0)  // 最右两路（原P8+P7改为P7+P6）
    {
        return 35;     
    }
    else if(P7!=0)      // 最右单路（原P8改为P7）
	{
		return 25;      
	}
    else if(P6!=0)      // 右2
	{
		return 20;      
	}
    else if(P5!=0)      // 右1
	{
		return 10;      
	}
	else if(P4!=0)      // 中心
	{
		return 0;       
	}
	return 0;
}

// 7路低速左转
int xunji_lowspeed_left(void)			
{
    if(P1!=0)           // 最左
	{
		return -25;     
	}
    else if(P2!=0)      // 左2
	{
		return -15;     
	}
    else if(P3!=0)      // 左1
	{
		return -10;     
	}
    else if(P4!=0)      // 中心
	{
		return -5;      
	}
	
	return 0;
}

// 7路低速右转
int xunji_lowspeed_right(void)			
{
    if(P7!=0)           // 最右（原P8改为P7）
	{
		return 25;      
	}
    else if(P6!=0)      // 右2
	{
		return 15;      
	}
    else if(P5!=0)      // 右1
	{
		return 10;      
	}
	else if(P4!=0)      // 中心
	{
		return 5;       
	}
	
	return 0;
}

float getencoderleft(void)
{
    return gEncoderVal_left2;
}
float getencoderright(void)
{
    return gEncoderVal_right2;
}
void GROUP1_IRQHandler(void)
{
    gpioA=DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_PORT,GPIO_Encoder_PIN_left_A_PIN | GPIO_Encoder_PIN_left_B_PIN );
    gpioB=DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_PORT,GPIO_Encoder_PIN_right_A_PIN | GPIO_Encoder_PIN_right_B_PIN );
    if (gpioB != 0) gDebugRightIRQCount++;

    if((gpioA & GPIO_Encoder_PIN_left_A_PIN)==GPIO_Encoder_PIN_left_A_PIN)
    { 
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_left_B_PIN))
            {
                gEncoderCount_left++;
            }
            else 
            {
                gEncoderCount_left--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_left_A_PIN);
    }
    if((gpioA & GPIO_Encoder_PIN_left_B_PIN)==GPIO_Encoder_PIN_left_B_PIN)
    {
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_left_A_PIN))
            {
                gEncoderCount_left++;
            }
            else 
            {
                gEncoderCount_left--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_left_B_PIN);    
}
    
    if((gpioB & GPIO_Encoder_PIN_right_A_PIN)==GPIO_Encoder_PIN_right_A_PIN)
    { 
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_right_B_PIN))
            {
                gEncoderCount_right++;
            }
            else 
            {
                gEncoderCount_right--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_right_A_PIN);
    }
    if((gpioB & GPIO_Encoder_PIN_right_B_PIN)==GPIO_Encoder_PIN_right_B_PIN)
    {
            if(DL_GPIO_readPins(GPIOB,GPIO_Encoder_PIN_right_A_PIN))
            {
                gEncoderCount_right++;
            }
            else 
            {
                gEncoderCount_right--;
            }
        DL_GPIO_clearInterruptStatus(GPIOB,GPIO_Encoder_PIN_right_B_PIN);    
}
}
volatile int32_t gEncoderVal_left[4] = {0};   // 4点平均，100ms×4=400ms窗口
volatile int32_t gEncoderVal_right[4] = {0};
volatile uint8_t filter_index = 0;
void TIMA1_IRQHandler(void)
{
    if(DL_Timer_getPendingInterrupt(TIMER_encoder_INST)==DL_TIMER_IIDX_ZERO)
	{
        static uint16_t counter,counterxiaozhun;
        if(counter>=5)
       {  // 保存当前50ms计数
        gEncoderVal_left[filter_index] =gEncoderCount_left;
        gEncoderVal_right[filter_index] = gEncoderCount_right;
        gEncoderCount_right = 0;
        gEncoderCount_left = 0;
        // 4点滑动平均（更平滑，延迟可接受）
        int32_t sum_left = 0, sum_right = 0;
        for(int i=0; i<4; i++) {
            sum_left += gEncoderVal_left[i];
            sum_right += gEncoderVal_right[i];
        }
        gEncoderVal_left2 = sum_left / 4;
        gEncoderVal_right2 = sum_right / 4;
        
        filter_index = (filter_index + 1) % 4;
        counter=0;
        }
        counter++;
        }
        
		DL_Timer_clearInterruptStatus(TIMER_encoder_INST,DL_TIMER_IIDX_ZERO);
	}

void TIMG0_IRQHandler(void)
{
    if(DL_Timer_getPendingInterrupt(TIMER_0_INST)==DL_TIMER_IIDX_ZERO)
    {
        
        if (ledbegin==1)
            {
                ledflag=1;
                if (lednum==200)
                {
                    ledflag=0; 
                    ledbegin=0;
                    lednum=0;
               
                }
                lednum++;
            }
    
     if (timebegin==1)
            {
                if (timenum==50)
                {
                    whiteflag=1;
                    timebegin=0;
                    timenum=0;
                }
                timenum++;
            }
            if (timebegin1==1)
            {
                if (timenum1==3)
                {
                    whiteflag1=1;
                    timebegin1=0;
                    timenum1=0;
                }
                timenum1++;
            }

            if (timebegin2==1)
            {
                if (timenum2==19)
                {
                    whiteflag2=1;
                    timebegin2=0;
                    timenum2=0;
                }
                timenum2++;
            }

            if (timbegin==1)
            {
                tim = tim + 0.01;
            }
    }
		DL_Timer_clearInterruptStatus(TIMER_0_INST,DL_TIMER_IIDX_ZERO);
	}
