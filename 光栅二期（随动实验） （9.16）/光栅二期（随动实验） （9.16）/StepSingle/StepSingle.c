#include "StepSingle.h"
#include "math.h"

//跟踪微分器（TD）
//PositionTargetFW = StepSingleDataFW.Position;
//VelocityTarget=PositionTargetFW;
//StepSingleDataFW.Position = PX_fw;
//PositionLast=StepSingleDataFW.Position
//r=30
//StepSingleDataFW.Velocity=0
//VelocityLast= StepSingleDataFW.Velocity;
StepSingleData StepSingle(float VelocityTarget, float PositionLast, float VelocityLast, float r, float h, float T)
{
	float PositionNext;
	float VelocityNext;
	float f;
	StepSingleData StepSingleDataTemp;
	f = fhan(PositionLast - VelocityTarget, VelocityLast, r, h);
	PositionNext = PositionLast + T * VelocityLast;
	VelocityNext = VelocityLast + T * f;
	StepSingleDataTemp.Position = PositionNext;
	StepSingleDataTemp.Velocity = VelocityNext;
	return StepSingleDataTemp;
}

//采样周期：h=0.01
//r=30
float fhan(float x1, float x2, float r, float h)
{
	float d;
	float d0;
	float y;
	float a0;
	int SignY = 0;
	int SignA = 0;
	float a;
	float fhan;
	d = r * h;
	d0 = h * d;
	y = x1 + h * x2;
	a0 = sqrtf(d * d + 8.0f * r * fabs(y));
	if(y >= 0)
	{
		SignY = 1;
	}
	else
	{
		SignY = -1;
	}
	if (fabs(y) > d0)
	{
		a = x2 + ((a0 - d) / 2) * SignY;
	}
	else
	{
		a = x2 + y / h;
	}
	if(a >= 0)
	{
		SignA = 1;
	}
	else
	{
		SignA = -1;
	}
	if(fabs(a) >= d)
	{
		fhan = -r * SignA; 
	}
	else
	{
		fhan = -r * a / d;
	}
	return fhan;
}
