
#include "SaoMiao.h"

//extern int n;
#define n_LYJ 285		

extern float ab[n_LYJ][2];

extern u8 counter_data;

float FW_angle_X=0;
float FY_angle_Y=0;

extern float FW_zero_degree1_fu ;        //-0.3 361
extern float FY_zero_degree1_zheng;     //0.02 《- 201.2021    361

float FW_X=0.5;
float FY_Y=0.5;
 
typedef enum
{
	dir_up,
	dir_down,
	dir_right,
	dir_left
}num_dir;


float SaoMiao_Init() 
{
	u32 x=0,y=0,z=0,v=0;
	u32 j = 0;
	u32 flag=0;
	num_dir now_dir;
	now_dir = dir_right;
	FW_angle_X = FW_zero_degree1_fu;
	FY_angle_Y = FY_zero_degree1_zheng;

	while(j < n_LYJ)  
	{
		switch(now_dir)
			{
				case dir_right:	//向右走
						x++;
						j++;
						if(x <= 1+flag)
							{
								FW_angle_X=FW_angle_X+FW_X;
								FY_angle_Y=FY_angle_Y;
								ab[j][0]=FW_angle_X;
								ab[j][1]=FY_angle_Y;
							}
						else
							{
								x = 0;
								j--;
								now_dir = dir_up;
							}
						break;
					case dir_up:	//向下走
								y++;
							j++;
						 if(y <= (1+flag))
							 {
									 FW_angle_X=FW_angle_X;	
									 FY_angle_Y=FY_angle_Y-FY_Y;
									 ab[j][0]=FW_angle_X;
									 ab[j][1]=FY_angle_Y;
							 }
						 else 
							 {
								 y = 0;
								 j--;
								 now_dir = dir_left;  
							 }
						break;
					case dir_left:  //向左走
						j++;
						z++;
						if(z <= (2+flag))
							{
									FW_angle_X=FW_angle_X-FW_X; 
									FY_angle_Y=FY_angle_Y;
									ab[j][0]=FW_angle_X;
									ab[j][1]=FY_angle_Y;
							}
						else
							{
								z = 0;
								j--;
								now_dir = dir_down;
							}
							
						break;
					case dir_down:  //向上走
						v++;
						j++;
						if(v <= (2+flag))
							{
									FW_angle_X=FW_angle_X;
									FY_angle_Y=FY_angle_Y+FY_Y;
									ab[j][0]=FW_angle_X;
									ab[j][1]=FY_angle_Y;
							}
						else
							{
								v = 0;
								j--;
								flag=flag+2;
								now_dir = dir_right;
							}	
						break;
					default: break;
		}
	}
	counter_data = 0;
}	



//	printf("FW_angle_X=%f\r\n",FW_angle_X);
//	printf("FY_angle_Y=%f\r\n",FY_angle_Y);
//	printf("j=%d\r\n",j);
//	for(a=0;a<2;a++)
//	{
//			for(i=0;i<=n;i++)
//			{
////				printf("X[%d]=%f   Y[%d]=%f\r\n",i,ab[i][0],i,ab[i][1]);
//					return ab[n][a];
//			}
//	}
//	return FY_angle_Y;
//}

