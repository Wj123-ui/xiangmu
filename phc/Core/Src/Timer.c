#include "Timer.h"
#include "PWM.h"
#include "PID.h"
#include "JY61P.h"
#include "lanya.h"

/* 编码器线数 13, 减速比 20.409, x4 计数 => 每圈 13*20.409*4 个计数 */
#define ENCODER_CPR     (13.0f * 20.409f * 4.0f)

/* 蓝牙差速命令(remote_diff)是 PWM 力度单位(±400), 差速环反馈是 rpm,
 * 这里换算: 1 个力度单位 = 0.15rpm 转速差, 即满量程 400 -> 60rpm */
#define DIFF_RPM_PER_PWM 0.15f

/* 车身倾角超过这个值就认为已经倒下: 切掉输出并清积分 */
#define FALL_ANGLE      45.0f

/* IMU 看门狗: 连续多少个 10ms 周期没收到有效帧就认为掉线(10 = 100ms)。
 * 取 100ms 是为了让 20Hz 及以上的模块(2 帧间隔远小于 10 拍)不会误判 */
#define IMU_LOST_TICKS  10

/* 角度环目标上限(度): speed.Out 和 remote_angle 叠加后不能无限加大 */
#define ANGLE_TARGET_MAX 12.0f

uint8_t count1=0;
uint16_t count2=0;
uint8_t count3=0;
int16_t leftspeed_0;
int16_t rightspeed_0;
float leftspeed;
float rightspeed;
float difspeed;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int16_t last3 = 0;	/* 上一次的计数值, 用差值法避免清零时丢脉冲 */
	static int16_t last4 = 0;
	static uint32_t imu_last_frames = 0;
	/* 初值就设成"已掉线": 上电后必须收到真实 IMU 数据才闭环,
	 * 否则前几十毫秒会拿 Pitch=0 这个假姿态去驱动电机 */
	static uint8_t  imu_lost = IMU_LOST_TICKS;
	static uint8_t  angle_safe = 1;	/* 1 = 当前处于保护态(倒下/IMU 掉线), 不输出
									 * 初值给 1: 上电第一拍先对齐 Actual1, 避免微分踢腿 */
	int16_t now3;
	int16_t now4;

    if (htim->Instance == TIM1) 
	{
		count1++;
		count2++;
		count3++;
		if(count1==20)
		{
			now3 = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
			now4 = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
			leftspeed_0  = (int16_t)(now3 - last3);
			rightspeed_0 = (int16_t)(now4 - last4);
			last3 = now3;
			last4 = now4;
			leftspeed  = -((float)leftspeed_0  / ENCODER_CPR) * 3000.0f;	/* 20ms 采样 -> x3000 得 rpm */
			rightspeed = ((float)rightspeed_0 / ENCODER_CPR) * 3000.0f;
			difspeed=leftspeed-rightspeed;
			count1=0;
		}
		if(count2>=50)
		{
			count2=0;

			/* 保护态下外环也停算, 否则 $v 的目标还在时速度环会一直积分,
			 * 一扶正就把攒下的大倾角目标一次性灌给角度环 */
			if (imu_lost >= IMU_LOST_TICKS
			 || angle.Actual >  FALL_ANGLE
			 || angle.Actual < -FALL_ANGLE)
			{
				speed.Out = 0;
				speed.ErrorInt = 0;
				dif.Out   = 0;
				dif.ErrorInt = 0;
				difpwm = 0;
			}
			else
			{
				speed.Actual=(leftspeed+rightspeed)*0.5;
				speed.Target=remote_speed;		/* 蓝牙 $v,<rpm> 给速度环目标, $s 清零 */
				PID_Update(&speed);

				/* 差速环(转向环): 反馈 = 左右轮转速差, 输出 = 差速 PWM
				 * 和速度环同周期, 目标由蓝牙 $l/$r 给(remote_diff) */
				dif.Actual=difspeed;
				dif.Target=remote_diff*DIFF_RPM_PER_PWM;
				PID_Update(&dif);
				difpwm=dif.Out;
			}
		}
		if(count3>=10)
	  {
		  float target;

		  count3=0;

		  /* 遥控失联保护: 手机断开/App 被杀后, 不能让最后一条 $f/$l 永久生效 */
		  lanya_remote_watchdog();

		  /* IMU 看门狗: 有效帧计数不再增长就认为串口掉线 */
		  if (JY61P_FrameCount == imu_last_frames)
		  {
			  if (imu_lost < 255) { imu_lost++; }
		  }
		  else
		  {
			  imu_last_frames = JY61P_FrameCount;
			  imu_lost = 0;
		  }

		  angle.Actual=Pitch-BALANCE_ANGLE;

		  /* 角度环目标 = 速度环输出(度) + 蓝牙遥控倾角(度),
		   * 两者叠加后限幅, 防止 $f 和大速度目标同时给导致目标倾角过大 */
		  target = speed.Out + remote_angle;
		  if (target >  ANGLE_TARGET_MAX) { target =  ANGLE_TARGET_MAX; }
		  if (target < -ANGLE_TARGET_MAX) { target = -ANGLE_TARGET_MAX; }

		  /* 保护: 车身倒下或 IMU 掉线时不闭环, 清积分并输出 0
		   * 这样扶起来时不会带着上电/摔倒期间攒下的积分往外窜 */
		  if (imu_lost >= IMU_LOST_TICKS
		   || angle.Actual >  FALL_ANGLE
		   || angle.Actual < -FALL_ANGLE)
		  {
			  angle.Target   = 0;
			  angle.Out      = 0;
			  angle.Error0   = 0;
			  angle.Error1   = 0;
			  angle.Actual1  = angle.Actual;
			  angle.ErrorInt = 0;
			  speed.Out      = 0;
			  speed.Error0   = 0;
			  speed.Error1   = 0;
			  speed.Actual1  = speed.Actual;
			  speed.ErrorInt = 0;
			  dif.Out      = 0;
			  dif.ErrorInt = 0;
			  difpwm    = 0;
			  avepwm    = 0;
			  leftpwm   = 0;
			  rightpwm  = 0;
			  angle_safe = 1;
		  }
		  else
		  {
			  /* 刚从保护态恢复: 丢掉摔倒期间的目标, 从当前姿态重新开始 */
			  if (angle_safe)
			  {
				  angle.ErrorInt = 0;
				  angle.Error0   = 0;
				  angle.Error1   = 0;
				  angle.Actual1  = angle.Actual;
				  angle_safe = 0;
			  }
			  angle.Target=target;
			  PID_Update(&angle);
			  avepwm=angle.Out;
		  }

		  leftpwm=avepwm+difpwm/2;
		  rightpwm=avepwm-difpwm/2;
		  WJ_Set_leftPWM(leftpwm);
		  WJ_Set_rightPWM(rightpwm);
	  }
    }
}
