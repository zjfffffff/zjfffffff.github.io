/*文件包含伺服输出函数，
输入全局变量：FW_revise_out、FY_revise_out，        位置闭环校正输出 单位：度/秒
			  FW_wending_qiankui_speed_lowpass、FY_wending_qiankui_speed_lowpass   稳定前馈输出  单位：度/秒
		      FW_track_qiankui_speed_lowpass、FY_track_qiankui_speed_lowpass       跟踪前馈输出  单位：度/秒
输出全局变量：FW_output_value_da、FY_output_value_da      单位：伏
2020.03.02*/
#include "MotorOutputPosse.h"
#include "stm32f4xx.h"	
#include "FSMC.h"
#include "ioinit.h"
extern float GS_Suidong_Flag;
extern float kp_guangshan;
extern float miss_distance_X_float_sum;

#define STEP_SCAN_MODE 10       
#define TRACKING_MODE 7  

float FW_limit_tuoluo_value = 0;
extern float FW_tuoluo_out;
float velocity_tuoluo_limit_value_set = 10;
float GS_Suidong_velocity_limit_set=60;
float limit_accelerate(float input_velocity,float velocity_last,float acc_limit_value);
float limit_velocity(float input_velocity,float velocity_limit_value);
float velocity_to_voltage(float velocity ,float ratio);
void FeedForwardFW(void);
void FeedForwardFY(void);

float FW_limit_acc_value = 0;
float FY_limit_acc_value = 0;
float FW_output_value  = 0;
float FY_output_value  = 0;
float FW_output_value_last = 0;
float FY_output_value_last = 0;
float accelerate_limit_1ms = 0;
extern float velocity_limit_value_set;
float FW_output_value_da = 0;
float FY_output_value_da = 0;
float DA_Test_LY = 0;
float FW_revise_out_speed = 0;         //方位校正值加前馈速度
float FY_revise_out_speed = 0;         //校正 + 速度前馈
float FY_track_qiankui_speed_k = 0.95;
float FY_track_qiankui_speed_lowpass = 0;
float FY_wending_qiankui_speed_k = 0.95;      //位置闭环前馈K☆   跟踪0.95
float FY_wending_qiankui_speed_lowpass = 0;   //稳定前馈速度滤波
float FW_wending_qiankui_speed_k = -0.95;     //                 跟踪 -0.95
float FW_track_qiankui_speed_k = 0.95;   //光闭环前馈系数Gain★
float FW_track_qiankui_speed_lowpass = 0;
float FW_wending_qiankui_speed_lowpass = 0;  //方位前馈速度低通滤波

extern float FW_encoder_degrees;  //俯仰编码器转换为角度
extern float FW_zero_degree_zheng;     //0.02 《- 201.2021
extern float FY_encoder_degrees;  //俯仰编码器转换为角度
extern u8 system_mode; 
extern float FY_revise_out;               //校正输出乘K
extern float FW_revise_out;               //方位校正值（速度）


float QK_k=1;
float QK_k_FW = 0.95;
extern float FW_Speed_Position;
extern u8 FW_Choose_Mode;
extern u8 FW_light_loop_open_flag;
extern u8 FY_light_loop_open_flag;
extern u8 FY_Choose_Mode;
extern float FY_Speed_Position;

//void BeginMortor_tuoluoFW(void)
//{
//	FW_limit_tuoluo_value = limit_velocity(FW_tuoluo_out,velocity_tuoluo_limit_value_set);      //限完加速度的速度，限制的速度
//	//速度转输出电压
//	FW_output_value_da = velocity_to_voltage(FW_limit_tuoluo_value ,10.0);  //原始
//	
//	DA12_out(FW_output_value_da,9999);
//}

extern float miss_distance_X_float;
extern float miss_distance_Y_float;
float kp_guangshan_tuob_X = 0;//0.037
float kp_guangshan_tuob_Y = 0;


void BeginMortorFW(void)
{
	FeedForwardFW();
	//限加速度
//	FW_limit_acc_value = limit_accelerate(FW_output_value,FW_output_value_last,accelerate_limit_1ms);    //速度，上一次速度，限制的加速度
	FW_limit_acc_value = FW_output_value;   
//	FW_output_value_last = FW_limit_acc_value;
//	//限速度
if(system_mode==17&&GS_Suidong_Flag==1)
{
	FW_limit_acc_value = limit_velocity(FW_limit_acc_value,GS_Suidong_velocity_limit_set);      //限完加速度的速度，限制的速度
	//速度转输出电压
}
	FW_output_value_da = velocity_to_voltage(FW_limit_acc_value ,15.0);  //原始
//FW_output_value_da=FW_output_value_da*0.5;
	DA12_out(FW_output_value_da,9999);
//	DA_Test_LY = FW_encoder_degrees - FW_zero_degree_zheng;    //FY_zero_degree			
//	DA12_out(FW_output_value_da,DA_Test_LY);
}


void BeginMortorFY(void)
{
		FeedForwardFY();
	//限加速度
//	if(system_mode == 7)
//	{
//			accelerate_limit_1ms = 5;
//	}	
		
//		FY_limit_acc_value = limit_accelerate(FY_output_value,FY_output_value_last,accelerate_limit_1ms); 
	    FY_limit_acc_value = FY_output_value; 
//		FY_output_value_last = FY_limit_acc_value;
//		//限速度	
	if(system_mode==17&&GS_Suidong_Flag==1)
	{
		FY_limit_acc_value = limit_velocity(FY_limit_acc_value,GS_Suidong_velocity_limit_set);	//velocity_limit_value_set=10
		//速度转输出电压
	}
		FY_output_value_da = velocity_to_voltage(FY_limit_acc_value ,15.0);	 //原始
//	  FY_output_value_da=FY_output_value_da*0.5;
		DA12_out(9999,FY_output_value_da);	
//		wBG_FW(0x37F);
//	DA_Test_LY = (FY_encoder_degrees - 323.01)*100;    //FY_zero_degree			
//	DA12_out(DA_Test_LY,FY_output_value_da);
}

void StopMortorFW(void)
{
	
}

void StopMortorFY(void)
{
	
}

void FeedForwardFW(void)
{
	//当system_mode == 7，FY_Choose_Mode=2；当system_mode ！= 7，FY_Choose_Mode=1.
	if(FW_light_loop_open_flag == 1 && FW_Choose_Mode != 1)
	{
//		FW_Speed_Position = FW_revise_out;
		FW_output_value = FW_Speed_Position;
	}
	else
	{			
		if(system_mode == 25)
		{
			FW_revise_out_speed = FW_revise_out + kp_guangshan_tuob_X*miss_distance_X_float;//方位校正值（速度）
			FW_output_value = FW_revise_out_speed;
		}
		else
		{
			FW_revise_out_speed = FW_revise_out;
			FW_output_value = FW_revise_out_speed;
		}		
	}	
}

void FeedForwardFY(void)
{
		//当system_mode == 7，FY_Choose_Mode=2；当system_mode ！= 7，FY_Choose_Mode=1.
		if(FY_light_loop_open_flag == 1 && FY_Choose_Mode != 1)
		{
//				FY_Speed_Position = FY_revise_out;
				FY_output_value = FY_Speed_Position;
		}
		else
		{		
			if(system_mode == 25)
			{
				FY_output_value = FY_revise_out + kp_guangshan_tuob_Y*miss_distance_Y_float;	//校正输出乘K	：FY_revise_out
			}
			else
			{
				FY_output_value = FY_revise_out;
			}
		}
//	//校正值 + 加入前馈速度
//	if(system_mode == TRACKING_MODE)        //跟踪模式
//	{
//		FY_revise_out_speed = FY_revise_out + FY_track_qiankui_speed_k * FY_track_qiankui_speed_lowpass;  //加入前馈速度				
//	}
//	else 
//	{
//		if(system_mode == STEP_SCAN_MODE)   //阶跃扫描模式
//		{
////			FY_wending_qiankui_speed_k = 1;
//			FY_wending_qiankui_speed_k = QK_k;
//		}
//		else
//		{
////			FY_wending_qiankui_speed_k = 0.95;
//			FY_wending_qiankui_speed_k = QK_k;
//		}
//		FY_revise_out_speed = FY_revise_out + FY_wending_qiankui_speed_k * FY_wending_qiankui_speed_lowpass;  //加入前馈速度		
//	}
	
//	FY_output_value = FY_revise_out;	
}


float acc_output;

/*
			input_velocity 	= FY_output_value
			velocity_last  	= FW_limit_acc_value
			acc_limit_value = accelerate_limit_1ms = accelerate_limit * 0.001 = 1;
*/
float limit_accelerate(float input_velocity,float velocity_last,float acc_limit_value)
{

	if((input_velocity - velocity_last) > acc_limit_value)
	{
		acc_output = velocity_last + acc_limit_value;
	}
	else if((input_velocity - velocity_last) < -acc_limit_value)
	{
		acc_output = velocity_last - acc_limit_value;
	}
	else
	{
		acc_output = input_velocity;
	}
    return acc_output;
}


/*
					input_velocity 		= FW_limit_acc_value
			velocity_limit_value  = velocity_limit_value_set = 10；
*/
float velocity_output;
float limit_velocity(float input_velocity,float velocity_limit_value)
{

	if(input_velocity > velocity_limit_value)	// velocity_limit_value = 10
	{
		velocity_output = velocity_limit_value;
	}
	else if(input_velocity < -velocity_limit_value)
	{
		velocity_output = -velocity_limit_value; 
	}
	else
	{
		velocity_output = input_velocity;
	}
	return velocity_output;

}

/*
		velocity = FY_limit_acc_value
		ratio    = 10
*/
float output_voltage;
float velocity_to_voltage(float velocity ,float ratio)
{

	output_voltage = velocity / ratio;
	return output_voltage;
}

