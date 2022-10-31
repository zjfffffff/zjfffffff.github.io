#include "stm32f4xx.h"	
#include "math.h"
#include "RMS.h"

#define N 1000

float Data[N]={0};
float Data_1[N] = {0};

float CalculateRMS1(float Final)
{
	float Sum1 = 0.0,Sum2= 0.0,RMS= 0.0,DataBar= 0.0;
	u32 i = 0;
	for(i=0;i<=N-2;i++)
	{
		Data[i]=Data[i+1];
	}
	Data[N-1]=Final;
	for(i=0;i<=N-1;i++)
	{
		Sum1=Sum1+Data[i];
	}
	DataBar=Sum1/N;
	for(i=0;i<=N-1;i++)
	{
		Sum2=Sum2+(Data[i]-DataBar)*(Data[i]-DataBar);
	}
	RMS=sqrt(Sum2/N);
	return RMS;
}	

float CalculateRMS2(float Final)
{
	float Sum3 = 0.0,Sum4= 0.0,RMS_1= 0.0,DataBar= 0.0;
	u32 i = 0;
	for(i=0;i<=N-2;i++)
	{
		Data_1[i]=Data_1[i+1];
	}
	Data_1[N-1]=Final;
	for(i=0;i<=N-1;i++)
	{
		Sum3=Sum3+Data_1[i];
	}
	DataBar=Sum3/N;
	for(i=0;i<=N-1;i++)
	{
		Sum4=Sum4+(Data_1[i]-DataBar)*(Data_1[i]-DataBar);
	}
	RMS_1=sqrt(Sum4/N);
	return RMS_1;
}	


