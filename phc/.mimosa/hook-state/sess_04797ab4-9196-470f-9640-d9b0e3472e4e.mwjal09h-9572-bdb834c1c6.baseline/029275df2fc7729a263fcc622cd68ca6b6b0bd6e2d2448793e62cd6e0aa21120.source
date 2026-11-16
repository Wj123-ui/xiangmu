#ifndef _PID_H_
#define _PID_H_
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	
	float Actual;
	float Actual1;	/* 上一拍的实际值: 微分只对测量值求差, 目标阶跃不会踢腿 */
	float Target;
	float Out;
	
	float OutMax;
	float OutMin;
	
	float Error0;
	float Error1;
	float ErrorInt;
	float IntMax;	/* 积分限幅(正负对称); <=0 表示不限幅 */
} PID_t;
void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
#endif
