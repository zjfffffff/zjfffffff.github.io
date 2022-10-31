#ifndef __StepSingle_H
#define __StepSingle_H

typedef struct
{
	float Position;
	float Velocity;
}StepSingleData;

StepSingleData StepSingle(float VelocityTarget, float PositionLast, float VelocityLast, float r, float h, float T);//
float fhan(float x1, float x2, float r, float h);

#endif
