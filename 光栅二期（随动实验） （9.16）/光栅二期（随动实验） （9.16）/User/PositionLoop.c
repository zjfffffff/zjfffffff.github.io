/*文件包含方位和俯仰位置闭环函数，
输入全局变量：FW_location_set、FY_location_set，单位：度
              FY_encoder_degrees_lowpass、FW_encoder_degrees_lowpass，编码器滤波后的值 单位：度
输出全局变量：FY_revise_out、FW_revise_out      单位：度/秒
2020.03.01*/
#include "stm32f4xx.h"
#include "FsmCompensator.h"
#include "PositionLoop.h"

float FX_set_prepass_xiuzheng = 0;	//零点修正
float FY_set_prepass_xiuzheng = 0;	//零点修正
///////////////////////////////////////////////
//陀螺
extern float FW_tuoluo_para;
extern float FY_tuoluo_para;
float FW_tuoluo_out = 0;
float FY_tuoluo_out = 0;

float FW_tuoluo_kp = 300;		//15
float FY_tuoluo_kp = 300;		//20

float FW_tuoluo_ki = 0;
float FY_tuoluo_ki = 0;

float FW_tuoluo_kd = 0;
float FY_tuoluo_kd = 0;

float FW_tuoluo_loop_error_diff = 0;
float FY_tuoluo_loop_error_diff = 0;

float FW_tuoluo_loop_error_sum = 0;
float FY_tuoluo_loop_error_sum = 0;

extern float FW_Tuoluo_set;
extern float FY_Tuoluo_set;

extern float FW_Tuoluo_last;
extern float FY_Tuoluo_last;

extern float FW_Tuoluo_bit_float;
extern float FY_Tuoluo_bit_float;

float FW_tuoluo_loop_error_last = 0;
float FY_tuoluo_loop_error_last = 0;

float FW_tuoluo_loop_error = 0;
float FY_tuoluo_loop_error = 0;

float jifen_FW = 0;
float jifen_FY = 0;

///////////////////////////////////////////////////////
float W_FW = 0.01;	//内模控制滤波时间参数
float W_FY = 0.01;	//内模控制滤波时间参数
extern float Km ;
extern float Tm ;
extern float Te ;

int Gain = 1;

float FW_revise_out_1 = 0;
float FW_revise_out_2 = 0;
float FW_revise_out_3 = 0;
float FY_revise_out_1 = 0;
float FY_revise_out_2 = 0;
float FY_revise_out_3 = 0;

extern float FY_location_set ;             //编码器闭环设定值
extern float FW_location_set;    //方位设定值
extern float FY_encoder_degrees_lowpass ;  //俯仰编码器转换为角度
extern float FY_encoder_degrees;  //俯仰编码器转换为角度
extern u8 FY_location_para;
extern u8 system_mode;
float FW_revise_out = 0;               //方位校正值（速度）
float FY_revise_out = 0;               //校正输出乘K
float FY_location_loop_k = 150;           //位置闭环仅有开环增益K☆
float FY_location_loop_k_1 = 30;          //位置闭环一阶校正开环增益K
float FY_location_loop_k_2 = 50;     	  //位置闭环二阶校正开环增益K	
float FY_location_low_pass = 0;           //80Hz低通滤波值
float FY_leadlag1_out = 0;                //一阶校正输出未乘K
float FY_leadlag2_out = 0;                //二阶校正输出未乘K
float FY_location_loop_error = 0;         //设定值 - 编码器值    




extern float FW_encoder_degrees_lowpass;  //俯仰编码器转换为角度
extern float FW_location_actual_set;    //方位设定值
extern u8 FW_location_para;
float FW_location_loop_error = 0;    //设定值与编码器值误差
float FW_location_loop_actual_error = 0;    //设定值与编码器值误差
float FW_location_low_pass = 0;      //80Hz滤波限制速度环谐振峰
float FW_location_loop_k = 150;       //位置闭环仅有开环增益K
float FW_location_loop_k_1 = 30;     //位置闭环一阶校正开环增益K
float FW_location_loop_k_2 = 1000;   //位置闭环二阶校正开环增益K
float FW_leadlag1_out = 0;           //方位一阶滞后超前校正输出       
float FW_leadlag2_out = 0;           //方位二阶滞后超前校正输出  
float FW_location_loop_error_sum = 0;
float FW_location_loop_error_last = 0;
float FW_location_loop_error_diff = 0;
float error_sum_limit = 10000;
float jifen = 0;
float jifen_fy = 0;
float jifen_limit = 0;
float FW_location_ki = 0.05;//0.01
float FY_location_ki = 0.05;//0
float FW_location_kd = 0;
float FW_location_kp = 220;
float FY_Watch_shuyin_error = 0;
float Position_k = 150;
float Watch_location_loop = 0;

int FY_k_1 = 150;

float FY_location_loop_error_sum = 0;
float FY_location_loop_error_diff = 0;
float FY_location_loop_error_last = 0;
float FY_location_kp = 220;
float FY_location_kd = 0;

extern float FW_encoder_degrees;  //俯仰编码器转换为角度

extern float X10_fsmTao1;		//w2调节抗干扰能力

extern float Y10_fsmTao1;		//w2调节抗干扰能力

u8 count_dazhi_FY = 0;
u8 count_dazhi_FW = 0;

extern float kp_guangshan_Y;
extern float kp_guangshan_tuob_Y; 
extern float kp_guangshan;
extern float kp_guangshan_tuob_X; 
void fy_close_loop(void)
{
		//编码器闭环设定值 	FY_location_set = 322.8715	
		//俯仰编码器转换为角度 	FY_encoder_degrees_lowpass = 0;  
		//设定值 编码器值	=	编码器闭环设定值 + 零点修正值	-	 俯仰编码器转换为角度
		FY_location_loop_error = (FY_location_set + FY_set_prepass_xiuzheng) - FY_encoder_degrees; 
		if(FY_location_loop_error>40||FY_location_loop_error<-40)
		{
			count_dazhi_FY++;
		}
	
//		FY_Watch_shuyin_error	 = FY_location_set - FY_encoder_degrees_lowpass;
//		Watch_location_loop 	 = FY_location_set - FY_encoder_degrees;
//	
		FY_location_low_pass 	 = FY_location_loop_error;       //省略滤波程序 
		if(FY_location_para == 1)       //只有K
		{
				if(system_mode == 10 || system_mode == 4|| system_mode == 3)
				{
						FY_location_loop_k = 100;	//位置闭环仅有开环增益K☆
				}
				else
				{
						FY_location_loop_k = FY_k_1;//FY_k_1 = 150
				}
				//校正输出乘K	=	80Hz低通滤波值	*	开环增益K☆
				FY_revise_out = FY_location_low_pass * FY_location_loop_k;
		}
		else if(FY_location_para == 2)  //一阶校正
		{
	//		FY_location_loop_k_1 = Position_k;	
				//一阶校正输出未乘K
				FY_leadlag1_out = FsmLeadLag1(FY_location_low_pass,FSM_Y,5); //一阶滞后超前校正，FSM_Y=1
				//校正输出乘K	=	位置闭环一阶校正开环增益K	*	一阶校正输出未乘K	= 30 * 
				FY_revise_out = FY_location_loop_k_1 * FY_leadlag1_out;
		}
		else if(FY_location_para == 3)  //二阶校正 
		{
				//二阶校正输出未乘K	
				FY_leadlag2_out = FsmLeadLag2(FY_location_low_pass,FSM_Y,3); //二阶滞后超前校正
				//校正输出乘K	=	位置闭环二阶校正开环增益K	*	二阶校正输出未乘K	= 50 *
				FY_revise_out = FY_location_loop_k_2 * FY_leadlag2_out;
		}
		else if(FY_location_para == 4)  //PID校正
		{
			  
				//80Hz滤波限制速度环谐振峰
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_loop_error;			//积分
				//设定值 编码器值 FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //求导
				FY_location_loop_error_last = FY_location_loop_error;
				

				//	= 0.1	*	
				jifen_fy = FY_location_ki * FY_location_loop_error_sum;
				//校正输出乘K	=	115	*	编码器值 + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * FY_location_low_pass + jifen_fy+FY_location_kd * FY_location_loop_error_diff;
		}
		else if(FY_location_para == 5)  //PID校正+观测器
		{
				//80Hz滤波限制速度环谐振峰
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//积分
				//设定值 编码器值 FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //求导
				FY_location_loop_error_last = FY_location_loop_error;
				
				if(FY_location_loop_error_sum > error_sum_limit)//error_sum_limit=10
				{
						FY_location_loop_error_sum = error_sum_limit;
				}
				else if(FY_location_loop_error_sum < -error_sum_limit)
				{
						FY_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FY_location_loop_error_sum = FY_location_loop_error_sum;
				}
				//	= 0.1	*	
				jifen_fy = FY_location_ki * FY_location_loop_error_sum;
				//校正输出乘K	=	115	*	编码器值 + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * FY_location_low_pass + jifen_fy+FY_location_kd * FY_location_loop_error_diff;
				FY_revise_out_2 = FsmLeadLag3(FY_revise_out,FSM_Y,8);
				FY_revise_out_1 = FsmLeadLag3(FY_revise_out,FSM_Y,9);
				FY_revise_out_3 = FY_revise_out_1 - FY_revise_out_2;
				FY_revise_out = FY_revise_out - Gain*FY_revise_out_3;
		}
		else if(FY_location_para == 6)  //内模控制+观测器
		{
				//80Hz滤波限制速度环谐振峰
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//积分
				//设定值 编码器值 FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //求导
				FY_location_loop_error_last = FY_location_loop_error;
				
				if(FY_location_loop_error_sum > error_sum_limit)//error_sum_limit=10
				{
						FY_location_loop_error_sum = error_sum_limit;
				}
				else if(FY_location_loop_error_sum < -error_sum_limit)
				{
						FY_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FY_location_loop_error_sum = FY_location_loop_error_sum;
				}
				FY_location_kp = (Te + Tm)/(Km*W_FY);
				FY_location_ki = 1/(Te + Tm);
				FY_location_kd = (Te*Tm)/(Te + Tm);
				//	= 0.1	*	
				jifen_fy = FY_location_ki * FY_location_loop_error_sum;
				//校正输出乘K	=	115	*	编码器值 + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * (FY_location_low_pass + jifen_fy + FY_location_kd * FY_location_loop_error_diff);
				FY_revise_out_2 = FsmLeadLag3(FY_revise_out,FSM_Y,8);
				FY_revise_out_1 = FsmLeadLag3(FY_revise_out,FSM_Y,9);
				FY_revise_out_3 = FY_revise_out_1 - FY_revise_out_2;
				FY_revise_out = FY_revise_out - Gain*FY_revise_out_3;
		}
		else if(FY_location_para == 7)  //2自由度内模控制+观测器
		{
				//80Hz滤波限制速度环谐振峰
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//积分
				//设定值 编码器值 FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //求导
				FY_location_loop_error_last = FY_location_loop_error;
				
				if(FY_location_loop_error_sum > error_sum_limit)//error_sum_limit=10
				{
						FY_location_loop_error_sum = error_sum_limit;
				}
				else if(FY_location_loop_error_sum < -error_sum_limit)
				{
						FY_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FY_location_loop_error_sum = FY_location_loop_error_sum;
				}
				FY_location_kp = (Te + Tm)/(Y10_fsmTao1);
				FY_location_ki = 1/(Y10_fsmTao1);
				FY_location_kd = (Te*Tm)/(Y10_fsmTao1);
				//	= 0.1	*	
				jifen_fy = FY_location_ki * FY_location_loop_error_sum;
				//校正输出乘K	=	115	*	编码器值 + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * FY_location_low_pass + jifen_fy + FY_location_kd * FY_location_loop_error_diff;
				FY_revise_out_2 = FsmLeadLag3(FY_revise_out,FSM_Y,8);
				FY_revise_out_1 = FsmLeadLag3(FY_revise_out,FSM_Y,9);
				FY_revise_out_3 = FY_revise_out_1 - FY_revise_out_2;
				FY_revise_out = FY_revise_out - Gain*FY_revise_out_3;				
		}
}
	
u8 Begin = 0;

void fw_close_loop(void)
{
		//设定值与编码器值误差 = 方位设定值 + 零点修正值 -	俯仰编码器转换为角度 = 203.6674 - 0
		FW_location_loop_error = (FW_location_set + FX_set_prepass_xiuzheng) - FW_encoder_degrees;
		
		if(FW_location_loop_error>40||FW_location_loop_error<-40)
		{
			count_dazhi_FW++;
		}
		//设定值与编码器值误差 = 方位设定值 - 俯仰编码器转换为角度
		FW_location_loop_actual_error = FW_location_actual_set - FW_encoder_degrees_lowpass;
		//FW_location_low_pass = FsmLeadLag1(FW_location_loop_error,FSM_X,1);      //一阶滞后校正20Hz滤波
		FW_location_low_pass = FW_location_loop_error;//设定值与编码器值误差
	
		if(FW_location_para == 1)    //只有K
		{
				//方位校正值（速度） = 80Hz滤波限制速度环谐振峰 * 位置闭环仅有开环增益K
				FW_revise_out = FW_location_low_pass * FW_location_loop_k;//FW_location_loop_k=150
		}
		else if(FW_location_para == 2)  //一阶校正
		{
				//方位一阶滞后超前校正输出 
				FW_leadlag1_out = FsmLeadLag1(FW_location_low_pass,FSM_X,5);//一阶滞后超前校正
				//方位校正值（速度） = 位置闭环一阶校正开环增益K * 方位一阶滞后超前校正输出
				FW_revise_out = FW_location_loop_k_1 * FW_leadlag1_out;
		}
		else if(FW_location_para == 3)  //二阶校正 
		{
				//方位二阶滞后超前校正输出  
				FW_leadlag2_out = FsmLeadLag2(FW_location_low_pass,FSM_X,7);//二阶滞后超前校正
				//方位校正值（速度） = 位置闭环二阶校正开环增益K * 方位二阶滞后超前校正输出  
				FW_revise_out = FW_location_loop_k_2 * FW_leadlag2_out;
		}
		else if(FW_location_para == 4)  //PID校正
		{
				//80Hz滤波限制速度环谐振峰FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//设定值与编码器值误差 FW_location_loop_error
				FW_location_loop_error_last = FW_location_loop_error;
				
				//jifen = 0.1 * sum
				jifen = FW_location_ki * FW_location_loop_error_sum;

				//方位校正值（速度） = 30 * 80Hz滤波限制速度环谐振峰 + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
		}
		else if(FW_location_para == 5)  //PID校正+观测器
		{
				//80Hz滤波限制速度环谐振峰FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//设定值与编码器值误差 FW_location_loop_error
				FW_location_loop_error_last = FW_location_loop_error;
				
				if(FW_location_loop_error_sum > error_sum_limit)//error_sum_limit = 10
				{
						FW_location_loop_error_sum = error_sum_limit;
				}
				else if(FW_location_loop_error_sum < -error_sum_limit)
				{
						FW_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FW_location_loop_error_sum = FW_location_loop_error_sum;
				}
				//jifen = 0.1 * sum
				jifen = FW_location_ki * FW_location_loop_error_sum;

				//方位校正值（速度） = 30 * 80Hz滤波限制速度环谐振峰 + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;
		}
		else if(FW_location_para == 6)  //内模控制+观测器
		{
				//80Hz滤波限制速度环谐振峰FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//设定值与编码器值误差 FW_location_loop_error
				FW_location_loop_error_last = FW_location_loop_error;
				
				if(FW_location_loop_error_sum > error_sum_limit)//error_sum_limit = 10
				{
						FW_location_loop_error_sum = error_sum_limit;
				}
				else if(FW_location_loop_error_sum < -error_sum_limit)
				{
						FW_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FW_location_loop_error_sum = FW_location_loop_error_sum;
				}				
				FW_location_kp = (Te + Tm)/(Km*W_FW);
				FW_location_ki = 1/(Te + Tm);
				FW_location_kd = (Te*Tm)/(Te + Tm);
				//jifen = 0.1 * sum
				jifen = FW_location_ki * FW_location_loop_error_sum;

				//方位校正值（速度） = 30 * 80Hz滤波限制速度环谐振峰 + jifen + kd * diff
				FW_revise_out = FW_location_kp * (FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff);
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;
		}
		else if(FW_location_para == 7)			//2自由度内模控制+观测器
		{
				//80Hz滤波限制速度环谐振峰FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//设定值与编码器值误差 FW_location_loop_error
				FW_location_loop_error_last = FW_location_loop_error;
				
				if(FW_location_loop_error_sum > error_sum_limit)//error_sum_limit = 10
				{
						FW_location_loop_error_sum = error_sum_limit;
				}
				else if(FW_location_loop_error_sum < -error_sum_limit)
				{
						FW_location_loop_error_sum = -error_sum_limit; 
				}
				else
				{
						FW_location_loop_error_sum = FW_location_loop_error_sum;
				}				
				FW_location_kp = (Te + Tm)/(X10_fsmTao1);
				FW_location_ki = 1/(X10_fsmTao1);
				FW_location_kd = (Te*Tm)/(X10_fsmTao1);
				//jifen = 0.1 * sum
				jifen = FW_location_ki * FW_location_loop_error_sum;

				//方位校正值（速度） = 30 * 80Hz滤波限制速度环谐振峰 + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);   //Q  		out
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);		//Q/Gn  out
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;		//Q/Gn - Q
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;		//		
		}
}

//方位陀螺校正函数
void fw_tuoluo_close_loop(void)
{
//		FW_tuoluo_loop_error = FW_Tuoluo_last - FW_Tuoluo_bit_float;
		FW_tuoluo_loop_error = (FW_location_set + FX_set_prepass_xiuzheng) - FW_encoder_degrees;
		
		
		if(FW_tuoluo_para == 1)    //只有K
		{
				FW_tuoluo_out = FW_tuoluo_kp * FW_Tuoluo_bit_float;
		}
		else if(FW_tuoluo_para == 2)  //一阶校正
		{
				
		}
		else if(FW_tuoluo_para == 3)  //二阶校正 
		{
			
		}
		else if(FW_tuoluo_para == 4)  //PID校正
		{
				FW_tuoluo_loop_error_sum = FW_tuoluo_loop_error_sum + FW_tuoluo_loop_error;
				FW_tuoluo_loop_error_diff = FW_tuoluo_loop_error - FW_tuoluo_loop_error_last;
				FW_tuoluo_loop_error_last = FW_tuoluo_loop_error;
			
//				if(FW_tuoluo_loop_error_sum > error_sum_limit)//error_sum_limit = 10
//				{
//						FW_tuoluo_loop_error_sum = error_sum_limit;
//				}
//				else if(FW_tuoluo_loop_error_sum < -error_sum_limit)
//				{
//						FW_tuoluo_loop_error_sum = -error_sum_limit; 
//				}
//				else
//				{
//						FW_tuoluo_loop_error_sum = FW_tuoluo_loop_error_sum;
//				}
				
//				kp_guangshan=0.019;
//				kp_guangshan_tuob_X = 0.037;
			
				jifen_FW = FW_tuoluo_ki * FW_tuoluo_loop_error_sum;
				//FW_tuoluo_out
				FW_revise_out = FW_tuoluo_kp * FW_tuoluo_loop_error + jifen_FW +  FW_tuoluo_kd * FW_tuoluo_loop_error_diff;
		}
}

//俯仰陀螺校正函数		
void fy_tuoluo_close_loop(void)
{
		FY_tuoluo_loop_error = (FY_location_set + FY_set_prepass_xiuzheng) - FY_encoder_degrees; 
//		FY_tuoluo_loop_error = FY_limit_acc_value - FY_Tuoluo_bit_float;
		
		if(FY_tuoluo_para == 1)    //只有K
		{
				FY_tuoluo_out = FY_tuoluo_kp * FY_Tuoluo_bit_float;
		}
		else if(FY_tuoluo_para == 2)  //一阶校正
		{
				
		}
		else if(FY_tuoluo_para == 3)  //二阶校正 
		{
				
		}
		else if(FY_tuoluo_para == 4)  //PID校正
		{
				FY_tuoluo_loop_error_sum = FY_tuoluo_loop_error_sum + FY_tuoluo_loop_error ;
				FY_tuoluo_loop_error_diff = (FY_tuoluo_loop_error - FY_tuoluo_loop_error_last);
				FY_tuoluo_loop_error_last = FY_tuoluo_loop_error;
			
//				if(FY_tuoluo_loop_error_sum > error_sum_limit)//error_sum_limit = 10
//				{
//						FY_tuoluo_loop_error_sum = error_sum_limit;
//				}
//				else if(FY_tuoluo_loop_error_sum < -error_sum_limit)
//				{
//						FY_tuoluo_loop_error_sum = -error_sum_limit; 
//				}
//				else
//				{
//						FY_tuoluo_loop_error_sum = error_sum_limit;
//				}
				
//				kp_guangshan_Y=0.019;
//				kp_guangshan_tuob_Y = 0.037;
				
				jifen_FY = FY_tuoluo_ki * FY_tuoluo_loop_error_sum;
				//FY_tuoluo_out
				FY_revise_out = FY_tuoluo_kp * FY_tuoluo_loop_error + jifen_FY +  FY_tuoluo_kd * FY_tuoluo_loop_error_diff ;
		}
		else if(FY_tuoluo_para == 5)  //PID校正
		{
				
		}
}