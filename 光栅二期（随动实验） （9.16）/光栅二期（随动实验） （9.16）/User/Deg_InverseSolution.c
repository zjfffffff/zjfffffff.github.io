//////////////////////////////
//光栅项目指向公式（2022.5.7）
/////////////////////////////
#include "math.h"
#include "Receive_Send_Data.h"
#include "stm32f4xx.h"
#include "elmo.h"
#include "Deg_InverseSolution.h"
#include "PointFunction.h"

float sinBodeCount=0,sinStepAdd=0,sinBodeNUM=0;
double FW_Degree_GS=0.01;//水平方向
double FY_Degree_GS=0.01;//垂直方向
double FW_Degree_Before,FY_Degree_Before;//上一时刻坐标设定参数
double Deg_1_Before,Deg_2_Before;//上一时刻光栅角度最优解
double *Deg_1_1,*Deg_2_1;//光栅1与光栅2的最优解
double Deg_shang,Deg_shang;//此刻光栅角度
double jiao1,jiao2;//di 1 zu jie
double jiao3,jiao4;//di 2 zu jie
double jiao_1,jiao_2;//此刻最优值
double g=7.7274;;//周期与波长的比值
//double g=7.63251147;//周期与波长的比值
double fai;//偏折角
double fangwei;	//fangweijiao
int i_gs=0;//计数
double jianchashuzhi;
int Deadscale_Flag=0;
int check_flag;

int Deg_count1=0;
int Deg_count2=0;

int count__1[30]={0};

double degree1_before,degree2_before;
extern u8 system_mode; 
void Deg_CloseLoop(double x)//角度扣圈判断
{
double *a1;
double a2;	
if(x>=360)
a2=x-360;
a1=&a2;
x=*a1;
if(x<=-360)
a2=x+360;
a1=&a2;
x=*a1;
}
void Deg_InverseSolution(double FW_Degree_GS,double FY_Degree_GS)
{   double PI =3.14159265;
    double b1,b2;
	double turn1,turn2,turn3,turn4;
	
	if(FW_Degree_GS==0&&FY_Degree_GS==0)
	{
		FW_Degree_GS=0.01;
		FY_Degree_GS=0.01;
	}
	
//	if(FY_Degree_GS>=14.9||FY_Degree_GS<=-14.9)
//	{
//		Deg_count2=1;
////		FW_Degree_GS = -9.5;
////		FY_Degree_GS = 0.01;
//	}
//	if((FW_Degree_GS!=FW_Degree_Before)||(FY_Degree_GS!=FY_Degree_Before))
//	{i_gs++;}
    
	fai=sqrt(FW_Degree_GS * FW_Degree_GS + FY_Degree_GS * FY_Degree_GS);
	if (FY_Degree_GS>=0)
	{
	fangwei=acosf(FW_Degree_GS/fai)*(180/PI);
	}
	else
	{
	fangwei=360.0-acosf(FW_Degree_GS/fai)*(180/PI);
	}
	jiao1=fangwei-(asinf(g*sinf(fai*(PI/180))*0.5))*(180/PI);
	jiao2=fangwei+(asinf(g*sinf(fai*(PI/180))*0.5))*(180/PI);
	
	jiao3=fangwei+(asinf(g*sinf(fai*(PI/180))*0.5))*(180/PI)+180;
	jiao4=fangwei-(asinf(g*sinf(fai*(PI/180))*0.5))*(180/PI)+180;
	/////////////////////////////////////////////////////////////////
	if(fabs(jiao1-Deg_1_Before)>180)
		{
			if(jiao1-Deg_1_Before<0)
			{
				jiao1=jiao1+360;
			
			
			}
			if(jiao1-Deg_1_Before>0)
			{

				jiao1=jiao1-360;
		
			}
		}
		
		if(fabs(jiao2-Deg_2_Before)>180)
		{
			if(jiao2-Deg_2_Before<0)
			{
				jiao2=jiao2+360;
			
			
			}
			if(jiao2-Deg_2_Before>0)
			{

				jiao2=jiao2-360;
			
			}
		}
		
		if(fabs(jiao3-Deg_1_Before)>180)
		{
			if(jiao3-Deg_1_Before<0)
			{
				jiao3=jiao3+360;
			
			
			}
			if(jiao3-Deg_1_Before>0)
			{

				jiao3=jiao3-360;
			
			}
		}
		
		if(fabs(jiao4-Deg_2_Before)>180)
		{
			if(jiao4-Deg_2_Before<0)
			{
				jiao4=jiao4+360;
				
			
			}
			if(jiao4-Deg_2_Before>0)
			{

				jiao4=jiao4-360;
				
			}
		}
	
	b1=jiao1-Deg_1_Before;
	b2=jiao3-Deg_1_Before;
////	if(i_gs>0)
////	
//		if(system_mode==17)
//		{
//			jiao_1=jiao1;
//	   jiao_2=jiao2;
//		}
//		else
//{
	if((b1*b1)<=(b2*b2))
	{
		
	   jiao_1=jiao1;
	   jiao_2=jiao2;
	}
	else
	{
		jiao_1=jiao3;
		jiao_2=jiao4;
	}
//}
		
//////////////////////////////////////////////////


	
//	if(jiao_2==jianchashuzhi)
//	{check_flag=1;}
//	else
//	{check_flag=0;}
//	jianchashuzhi=jiao_2;
	Deg_1_1=&(jiao_1);
	Deg_2_1=&(jiao_2);
	Deg_1_Before=jiao_1;
	Deg_2_Before=jiao_2;
//	if((b1*b1)<=(b2*b2))
//	{
//	   Deg_1_Before=jiao1;
//	   Deg_2_Before=jiao2;		 
//	}
//	else
//	{
//	Deg_1_Before=jiao3;
//	Deg_2_Before=jiao4;
//	}
  FW_Degree_Before=FW_Degree_GS;
	FY_Degree_Before=FY_Degree_GS;
}

float SinBode(float amp, float frequency, float offest)
{
 float sinOut=0;
  sinBodeCount = 1000/frequency;
  sinStepAdd = 2*3.14/sinBodeCount;  
  sinOut = amp*sin(sinStepAdd*sinBodeNUM)+offest;
//  sinOut = amp*sin(2*3.14*frequency*sinBodeNUM/1000)+offest;
//  cosOut = amp2*cos(2*3.14*freq2*sinBodeNUM/1000)+offest;
  sinBodeNUM++;
  if (sinBodeNUM >= sinBodeCount)
  {  
   sinBodeNUM = 0;
  } 
  return sinOut;
}