#include "ServoFunction.h"
#include "stm32f4xx.h"
#include "math.h"
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"
#include "RMS.h"
void miss_distance_track(void);

float miss_distance_X_float = 0;
float miss_distance_Y_float = 0;
float miss_distance_Y_float_to_angle = 0;
float miss_distance_X_float_to_angle = 0;
u8 track_axis_select = 0;      //跟踪轴选择，0：跟方位俯仰 1;跟方位 2：跟俯仰 
u8 miss_cnt_10ms = 0;
u8 track_type_select = 1;        //跟踪类型   1：脱靶量   2：数引
float FW_miss_distance_error_lowpass = 0;      //方位脱靶量80Hz低通滤波
float FY_miss_distance_error_lowpass = 0;      //俯仰脱靶量80Hz低通滤波
float FW_moni_miss = 0;
float FY_moni_miss = 0;
float FW_light_loop_k = 1;       //理想2000
float FW_light_loop_revise = 0;                 //方位校正输出
float FW_light_loop_k_1 = 100;
float FW_miss_distance_error = 0;              //方位脱靶量统一单位+K
float FW_light_loop_k_2 = 4000;  //4   //默认二阶校正    90    45
float FW_pi_tao = 0;
float FW_pi_w = 0;
float FW_pi_hz = 1;
float FW_miss_distance_error_lowpass_sum = 0;      //方位脱靶量80Hz低通滤波
float FW_pi_K = 2;
u8 miss_distance_X_flag = 1;
float FY_pi_d1 = 0;
float FY_pi_i1 = 200;
float FY_pi_K1 = 100;
float FW_pi_i1 = 200;
float FW_pi_K1 = 100;
float FY_light_loop_k1 = -50;
float FY_light_loop_k = 200;       //光闭环开环增益K★   -0.1,-7000
float FY_light_loop_k_1 = 10;
float FY_light_loop_k_2 = 1;      //默认二阶校正※   -50   -25
float FY_light_loop_revise = 0;                //俯仰校正输出
float FY_miss_distance_error = 0;              //俯仰脱靶量统一单位+K
float FY_pi_tao = 0;
float FY_pi_w = 0;
float FY_pi_hz = 1;
float FY_miss_distance_error_lowpass_sum = 0;      //俯仰脱靶量80Hz低通滤波
float FY_miss_distance_error_lowpass_last = 0;
float FY_miss_distance_light_diff = 0;
float FY_pi_K = 1;
u8 FW_light_para_set = 1;      //光闭环校正参数选择 1：K  2:一阶校正  3：二阶校正   4：PI校正
u8 FY_light_para_set = 1;

extern float FW_Miss_distance;//方位脱靶量
extern float FY_Miss_distance;//俯仰脱靶量
extern u8 output_open_flag;
extern u8 FY_location_loop_open_flag;
extern u8 FW_location_loop_open_flag;
extern u8 FW_light_loop_open_flag;
extern u8 FY_light_loop_open_flag;
extern float FW_zero_degree_zheng;     //0.02 《- 201.2021
extern float FY_zero_degree;         //0.06 《- 323.02
extern float FY_location_set;    //编码器闭环设定值
extern float FW_location_set;    //方位设定值
extern float velocity_limit_value_set;
extern u8 FW_light_para;
extern u8 FY_light_para;
extern float FW_encoder_degrees;  //俯仰编码器转换为角度
extern float FY_encoder_degrees;  //俯仰编码器转换为角度
extern float sin_value;          //sin计算值
extern u32 sin_cnt;              //sin计数
extern float Z_slave_1ms_distance;
extern float X_slave_1ms_distance;
extern float X_slave_1ms_distance_SV;
u8 Song = 2;    

extern int FW_LY_CNT;
extern float FY_output_value_da;
extern float FY_limit_acc_value;
extern float FY_output_value;
extern float FY_output_angle_pa_now;
extern float FY_output_value_last;
extern float FY_output_angle_pa_last;     
extern float FY_revise_out;               //校正输出乘K
extern float FY_Target_Position;


//
float miss_offset_X=0;//设置的偏置方位脱靶量的变量
float miss_offset_Y=0;//设置的偏置方位脱靶量的变量
extern float z_axis_velocity_float;
//系统模式
extern u8 system_mode;

void ServoFunction(void)
{

	//X轴脱靶量
//	miss_distance_X_float = FW_Miss_distance; //（像素点）
	if(system_mode == 7)	//脱靶量跟踪
	{
		miss_distance_X_float_to_angle = (miss_distance_X_float + miss_offset_X) * 29.5 * 0.000057295; //（脱靶量转换为度）
		miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
	}
	if(system_mode == 8)   //惯导跟踪
	{
		miss_distance_X_float_to_angle = (z_axis_velocity_float + miss_offset_X); //（脱靶量转换为度）
		miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
	}
//	miss_distance_X_float_to_angle = (z_axis_velocity_float + miss_offset_X) * 29.5 * 0.000057295; //（脱靶量转换为度）
	
	//          miss_distance_X_float_to_angle = miss_distance_X_float_to_angle * 0.5;
	//Y轴脱靶量
//	miss_distance_Y_float = FY_Miss_distance;
//	miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
//	miss_distance_Y_float_to_angle = miss_distance_Y_float_to_angle * 0.5;	

	output_open_flag = 1;
	if(track_axis_select == 0)    //跟踪轴选择，0：跟方位俯仰 1;跟方位 2：跟俯仰 
																//track_axis_select = 0  跟方位俯仰 
	{
		FY_location_loop_open_flag = 0;
		FW_location_loop_open_flag = 0;
		FW_light_loop_open_flag = 1;
		FY_light_loop_open_flag = 1;
	}
	else if(track_axis_select == 1)       //跟方位，俯仰锁定
	{
		FY_location_loop_open_flag = 1;
		FW_location_loop_open_flag = 0;
		FW_light_loop_open_flag = 1;
		FY_light_loop_open_flag = 0;
		
		//FW_location_set = FW_zero_degree_zheng;			
		FY_location_set = FY_zero_degree - 30;	
	}
	else if(track_axis_select == 2)   //跟俯仰，方位锁定
	{
			FY_location_loop_open_flag = 0;
			FW_location_loop_open_flag = 1;
	    FW_light_loop_open_flag = 0;
	    FY_light_loop_open_flag = 1;

			FW_location_set = FW_zero_degree_zheng ;			
//			FY_location_set = FY_zero_degree; 				
	}

	velocity_limit_value_set = 90;             //60
	FW_light_para = FW_light_para_set;		//光闭环校正参数选择 1：K  2:一阶校正  3：二阶校正   4：PI校正
	FY_light_para = FY_light_para_set;    //光闭环校正参数选择 1：K  2:一阶校正  3：二阶校正   4：PI校正

	miss_distance_track();   //内含二阶校正 + 陀螺速度滤波
}

float Amp_sin = 3;
float FW_moni_offset = 294.9;
float FW_moni_mrad = 0;
float FY_moni_mrad = 0;
float FW_RMS_mrad = 0;
float FY_RMS_mrad = 0;
void miss_distance_track(void)
{
			//正弦模拟脱靶量，1ms产生一次前馈速度，10ms给一次位置误差
		sin_value = Amp_sin*sin(sin_cnt / 2000.0 * 2 * 3.1415926);
		sin_cnt++;
		if(sin_cnt == 2000)
		{
			sin_cnt = 0;
		}
		
//		miss_cnt_10ms++;	
//		if(miss_cnt_10ms == 10)  //每10ms用脱靶量跟踪一次，因为脱靶量输出为100Hz
		{
					miss_cnt_10ms = 0;
			//		if(read_usart5[2] == 0x10) //判决光斑在相机视场
					if(track_type_select == 1)   //跟踪脱靶量   
					{
						//80Hz低通滤波
						//FW_miss_distance_error_lowpass = FsmLeadLag1_servo(FW_moni_miss,FSM_X,1);
						//FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle * cos(FW_encoder_degrees - FW_zero_degree_zheng) + miss_distance_Y_float_to_angle * sin(FW_encoder_degrees - FW_zero_degree_zheng);
//						FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle + cos(FY_encoder_degrees)*tan(FW_encoder_degrees);
						FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle;
						
						//80Hz低通滤波
//						FY_miss_distance_error_lowpass = FsmLeadLag1_servo(miss_distance_Y_float_to_angle,FSM_Y,1);   
						//FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle * cos(FW_encoder_degrees - FW_zero_degree_zheng) - miss_distance_X_float_to_angle * sin(FW_encoder_degrees - FW_zero_degree_zheng); 
						//FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle + cos(FW_encoder_degrees)*tan(FY_encoder_degrees);
						FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle;
						
//						FW_RMS_mrad = CalculateRMS1(miss_distance_X_float);
//						FW_RMS_mrad = FW_RMS_mrad * 29.5;
//						FY_RMS_mrad = CalculateRMS2(miss_distance_Y_float);
//						FY_RMS_mrad = FY_RMS_mrad * 29.5;
					}
					else if(track_type_select == 2)  //跟踪数引
					{
							FW_moni_miss = FW_moni_offset + sin_value - FW_encoder_degrees;
							FY_moni_miss = 180 + sin_value - FY_encoder_degrees;
							FW_miss_distance_error_lowpass = FW_moni_miss;
							FY_miss_distance_error_lowpass = FY_moni_miss;	
						
							FW_moni_mrad = FW_moni_miss * 17.5;
							FY_moni_mrad = FY_moni_miss * 17.5;
			//			FW_RMS_mrad = CalculateRMS1(FW_moni_mrad);
			//			FY_RMS_mrad = CalculateRMS2(FY_moni_miss);	
					}
					//方位脱靶量跟踪计算
					//光闭环校正参数选择 1：K  
					if(FW_light_para == 1)	
					{
							FW_light_loop_revise = FW_light_loop_k * FW_miss_distance_error_lowpass / 10.0;   
					}
					//光闭环校正参数选择 2：一阶超前滞后校正 
					else if(FW_light_para == 2)		
					{
							FW_miss_distance_error = FW_light_loop_k_1 * FW_miss_distance_error_lowpass / 10.0;   
							FW_light_loop_revise = FsmLeadLag2_servo(FW_miss_distance_error,FSM_X,2);
					}
					//光闭环校正参数选择 3：二阶校正
					else if(FW_light_para == 3)		 
					{
							FW_miss_distance_error = FW_light_loop_k_2 * FW_miss_distance_error_lowpass / 10.0;   
							if(Song == 1)
							{
									FW_light_loop_revise = FsmLeadLag2_servo(FW_miss_distance_error,FSM_X,7);
							}
							if(Song == 2)		//Song = 2
							{
									FW_light_loop_revise = FW_miss_distance_error;
							}
					}
					else if(FW_light_para == 4)		//光闭环校正参数选择 4：PD 校正 
					{
						FW_pi_w =  FW_pi_hz * 2.0 * 3.1415926;
						FW_pi_tao = 1.0 / FW_pi_w;
						FW_miss_distance_error_lowpass_sum = FW_miss_distance_error_lowpass_sum + FW_miss_distance_error_lowpass;
						FW_light_loop_revise = FW_pi_K * (FW_pi_tao * FW_miss_distance_error_lowpass + 0.01 * FW_miss_distance_error_lowpass_sum);      //0.01是采样时间
					}
					else if(FW_light_para == 5)		//光闭环校正参数选择 4：PD 校正 
					{
						if((miss_distance_X_float > 5 || miss_distance_X_float < -5) && (miss_distance_X_flag == 1))
						{
							FW_light_loop_revise = FW_light_loop_k * FW_miss_distance_error_lowpass / 10.0;  
						}
						else
						{
//							miss_distance_X_flag = 0;
							FW_miss_distance_error_lowpass_sum = FW_miss_distance_error_lowpass_sum + FW_miss_distance_error_lowpass * 0.01;	//0.01是采样时间
							FW_light_loop_revise = FW_pi_K1 * FW_miss_distance_error_lowpass + FW_pi_i1 * FW_miss_distance_error_lowpass_sum;      
						}
					}
		
	//俯仰脱靶量跟踪计算
					if(FY_light_para == 1)
					{
							
							FY_light_loop_revise = FY_light_loop_k * FY_miss_distance_error_lowpass / 10.0; 
					}
					else if(FY_light_para == 2)
					{
							FY_miss_distance_error = FY_light_loop_k_1 * FY_miss_distance_error_lowpass / 10.0;     //脱靶量单位是度，DA输出是1V=10度/秒，统一单位
							FY_light_loop_revise = FsmLeadLag1_servo(FY_miss_distance_error,FSM_Y,2);	  //二阶校正 (T0.2涛1)
					}
					else if(FY_light_para == 3)
					{
							FY_miss_distance_error = FY_light_loop_k_2 * FY_miss_distance_error_lowpass / 10.0;     //脱靶量单位是度，DA输出是1V=10度/秒，统一单位
							if(Song == 1)
							{
									FY_light_loop_revise = FsmLeadLag2_servo(FY_miss_distance_error,FSM_Y,7);	  //二阶校正 (T0.2涛1)
							}
							if(Song == 2)
							{
									FY_light_loop_revise = FY_miss_distance_error;
							}
					}	
					else if(FY_light_para == 4)
					{
							FY_pi_w =  FY_pi_hz * 2.0 * 3.1415926;
							FY_pi_tao = 1.0 / FY_pi_w;
							FY_miss_distance_error_lowpass_sum = FY_miss_distance_error_lowpass_sum + FY_miss_distance_error_lowpass;
							FY_light_loop_revise = FY_pi_K * (FY_pi_tao * FY_miss_distance_error_lowpass + 0.01 * FY_miss_distance_error_lowpass_sum);      //0.01是采样时间
					}
					else if(FY_light_para == 5)		//光闭环校正参数选择 4：PD 校正 
					{
						if((miss_distance_Y_float > 5 || miss_distance_Y_float < -5) && (miss_distance_X_flag == 1))
						{
							FY_light_loop_revise = FY_light_loop_k1 * FY_miss_distance_error_lowpass / 10.0; 
						}
						else
						{
							FY_light_loop_revise = FY_light_loop_k * FY_miss_distance_error_lowpass / 10.0; 
						}
					}
					else if(FY_light_para == 6)		//光闭环校正参数选择 4：PD 校正 
					{
						if((miss_distance_Y_float > 10 || miss_distance_Y_float < -10) && (miss_distance_X_flag == 1))
						{
							FY_light_loop_revise = FY_light_loop_k1 * FY_miss_distance_error_lowpass / 10.0; 
						}
						else
						{
							FY_miss_distance_error_lowpass_sum = FY_miss_distance_error_lowpass_sum + FY_miss_distance_error_lowpass * 0.001;	//0.01是采样时间,积分
							FY_miss_distance_light_diff = (FY_miss_distance_error_lowpass - FY_miss_distance_error_lowpass_last)*1000.f;
							
							FY_light_loop_revise = FY_pi_K1 * FW_miss_distance_error_lowpass  + FY_pi_i1 * FY_miss_distance_error_lowpass_sum + FY_pi_d1 * FY_miss_distance_light_diff; 
							FY_miss_distance_error_lowpass_last = FY_miss_distance_error_lowpass;
						}
						
					}
		}	
//		else  //无脱靶量时用陀螺补偿
//		{
//	//		FW_light_loop_revise = Z_slave_1ms_distance;
//	//		FY_light_loop_revise = X_slave_1ms_distance_SV;
//		}
		
		if(FW_LY_CNT >= 990)
		{
				FW_LY_CNT = 990;
		}
		FW_LY_CNT++;
		
/*
	////前馈速度计算
	//	if(track_miss_flag == 1)
	//	{
	//		//跟踪速度滤波
	//		//陀螺速度前馈，俯仰跟踪                             
	//		X_axis_palstance_float = X_axis_palstance_float * 1 * -0.5;
	//		FY_track_qiankui_speed_lowpass = FsmLeadLag1(X_axis_palstance_float,FSM_Y,4);     //跟踪速度前馈滤波
	//		
	//		//陀螺速度前馈，方位跟踪
	//		Z_axis_palstance_float = Z_axis_palstance_float * 1;
	//		FW_track_qiankui_speed_lowpass = FsmLeadLag1(Z_axis_palstance_float,FSM_X,4);     //跟踪速度前馈滤波	
	//	}
	//	else if(track_miss_flag == 2)
	//	{
	//		//稳定速度滤波
	//		//IMU积分再微分出速度（每秒走的位置已取反）俯仰
	//		X_slave_1ms_distance_sub = sin_value - X_slave_1ms_distance_sum_last;
	//		FY_qiankui_speed = X_slave_1ms_distance_sub * 1000 * 1;         //统一单位
	//		X_slave_1ms_distance_sum_last = sin_value;
	//		FY_track_qiankui_speed_lowpass = FsmLeadLag1(FY_qiankui_speed,FSM_Y,2);     //稳定速度前馈滤波
	//		
	//		//IMU积分再微分出速度（每秒走的位置已取反）方位
	//		Z_slave_1ms_distance_sub = sin_value - Z_slave_1ms_distance_sum_last;
	//		FW_qiankui_speed = Z_slave_1ms_distance_sub * 1000 * 1;
	//		Z_slave_1ms_distance_sum_last = sin_value;
	//		FW_track_qiankui_speed_lowpass = FsmLeadLag1(FW_qiankui_speed,FSM_X,2);     //稳定速度前馈滤波 
	//	}
*/
}