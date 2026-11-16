#include "PID.h"
void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Actual1 = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	p->IntMax = 0;
}

void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		/* 积分限幅, 防止积分饱和(输出长时间顶在 OutMax/OutMin) */
		if (p->IntMax > 0)
		{
			if (p->ErrorInt >  p->IntMax) {p->ErrorInt =  p->IntMax;}
			if (p->ErrorInt < -p->IntMax) {p->ErrorInt = -p->IntMax;}
		}
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	/* 微分作用在"测量值的变化"上, 而不是误差的变化:
	 * 目标突变时不会产生一个巨大的微分脉冲(平衡车会因此踢腿并来回振荡)。
	 * 符号取负, 因为本函数里 Error = Target - Actual。 */
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   - p->Kd * (p->Actual - p->Actual1);

	p->Actual1 = p->Actual;

	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
