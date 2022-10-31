#include "TestFunction.h"
#include "stm32f4xx.h"	
#include "AttitudeAlgorithm.h"
#include "PositionLoop.h"
#include "MotorOutputPosse.h"
#include "FsmCompensator.h"
#include "timer.h"

void space_scan_2(void);

float watch_slave = 0;
float watch_zhijie = 0;
float FW_location_kp_step = 120;
u8 open_camera_value = 60;
u8 FW_rms_cnt = 0;
float FW_rms[100];
float FW_rms_sum;
float FW_actual_rms[100];
float FW_actual_rms_sum;
float FW_rms_ave = 0;
float FW_actual_rms_ave = 0;
u8 change_kp_value = 60;
float FW_location_kp_stable = 250;
float X_slave_1ms_distance_sum1_2 = 0;

extern u8 FY_location_loop_open_flag;
extern u8 FW_location_loop_open_flag;
extern u8 FW_light_loop_open_flag;
extern u8 FY_light_loop_open_flag;
extern u8 FY_location_para;
extern u8 FW_location_para;
extern u8 FY_location_para_set;
extern float accelerate_limit;
extern float velocity_limit_value_set;

extern float Z_slave_1ms_distance;
extern float Z_slave_1ms_distance_sum;
extern float X_slave_1ms_distance;
extern float X_slave_1ms_distance_sum;
extern float pitch_attitude_float;
extern u8 TuoLuo_Flag;
extern float CarrierFW;
extern float CarrierFY;
extern float CarrierHG;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
extern float FW_zero_degree_zheng;     //0.02 《- 201.2021
extern float FY_zero_degree; 
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float FW_location_set;    //方位设定值
extern float FY_location_set;    //编码器闭环设定值
extern unsigned int diaoyong_cnt;
extern float FW_location_kp;
extern float FW_location_loop_error ;    //设定值与编码器值误差
extern float FW_location_loop_actual_error;    //设定值与编码器值误差
extern float FW_step_chafen_speed;
extern float FY_step_chafen_speed;
extern float FW_wending_qiankui_speed_lowpass;  //方位前馈速度低通滤波
extern float FY_wending_qiankui_speed_lowpass;   //稳定前馈速度滤波
extern float global_FW_step_piror;
extern float global_FY_step_piror;
extern float yaw_attitude_float;
extern float roll_attitude_float;

float buchang_offset = 0.0012;
float buchang_offset_ly = 0;
extern float step_design_set;

void TestFunction(void)
{
	FY_location_loop_open_flag = 1;
	FW_location_loop_open_flag = 1;
	FW_light_loop_open_flag = 0;
	FY_light_loop_open_flag = 0;

	FW_location_para = 4;
	FY_location_para = FY_location_para_set;
	velocity_limit_value_set = 1000;
	accelerate_limit = 10000;

	space_scan_2();
}




void space_scan_2(void)
{
	Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;  //Z方位
	X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;
	
	watch_slave  = X_slave_1ms_distance_sum;
	watch_zhijie = pitch_attitude_float;
	
	if(TuoLuo_Flag == 0)
	{
		CarrierFW = Z_slave_1ms_distance_sum;
		CarrierFY = X_slave_1ms_distance_sum;
		CarrierHG = -0.15;
	
		SpaceSetValueFW = 0;
		SpaceSetValueFY = 0;
	}
	if(TuoLuo_Flag == 1)
	{
		CarrierFW = yaw_attitude_float;
//		CarrierFW = 0;
		CarrierFY = (-1) * pitch_attitude_float;
		CarrierHG = roll_attitude_float;
	
		SpaceSetValueFW = 0;
		SpaceSetValueFY = 0;
	}

	
	
    SpaceToAxiseAttitude();
	if(step_design_set>0)
	{
		buchang_offset_ly = buchang_offset;
	}
	
	else if(step_design_set<0)
	{
		buchang_offset_ly = -buchang_offset;
	}
	
	FW_location_set = FW_zero_degree_zheng - (FW_buchang_degree + buchang_offset_ly); //FW_buchang_degree内含阶跃
	FY_location_set = FY_zero_degree - FY_buchang_degree;
//改Kp,记初始脱靶量，累计求平均误差	
	if(diaoyong_cnt == 1)
	{
//		GPIO_SetBits(GPIOE,GPIO_Pin_2);//计时电平上拉
		FW_location_kp = FW_location_kp_step;    //阶跃阶段Kp=130
		//记阶跃初始脱靶量1
		//miss_distance_X_float_start1 = miss_distance_X_float;
	}
	else if(diaoyong_cnt == open_camera_value)                
	{
//		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//计时电平下拉
	}
	else if(diaoyong_cnt > open_camera_value && diaoyong_cnt< 110)  //稳定拍照时间
	{
		FW_rms_cnt++;
		FW_rms[FW_rms_cnt] = FW_location_loop_error;                //编码器误差存入数组
		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;               //误差累加
		
		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;         //前置滤波前编码器误差存入数组
		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum; //前置滤波前编码器误差累加
	}
	
	else if(diaoyong_cnt == 110)                         //计时电平上拉       
	{
//		GPIO_SetBits(GPIOE,GPIO_Pin_2);
		FW_rms_ave = FW_rms_sum * 0.025;//40个点                //编码器误差求均值
		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;  //前置滤波前编码器误差求均值
		//FW_rms_ave = FW_rms[1];
		//FW_actual_rms_ave = FW_actual_rms[1];
		
		FW_rms_sum = 0; 
		FW_actual_rms_sum = 0;
		FW_rms_cnt = 0;
		FW_location_kp = FW_location_kp_step;   //阶跃阶段Kp=130
		//记阶跃初始脱靶量2
		//miss_distance_X_float_start2 = miss_distance_X_float;
	}
	else if(diaoyong_cnt == (110 + open_camera_value))   
	{
//		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//计时电平下拉 
	}
	else if(diaoyong_cnt > (110 + open_camera_value) && diaoyong_cnt< 220)   //稳定拍照时间
	{
		FW_rms_cnt++;
		FW_rms[FW_rms_cnt] = FW_location_loop_error;     //编码器误差存入数组
		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;    //误差累加
		
		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;
		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum;
	}
	else if(diaoyong_cnt == 220)                      //清零
	{
		diaoyong_cnt = 220;                 //需要清零
	    FW_rms_ave = FW_rms_sum * 0.025;
		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;
		//FW_rms_ave = FW_rms[1];
		//FW_actual_rms_ave = FW_actual_rms[1];	
		FW_rms_sum = 0;
	    FW_actual_rms_sum = 0;
		FW_rms_cnt = 0;
	}
	
//稳定拍照阶段切换高开环增益
	if(diaoyong_cnt > change_kp_value && diaoyong_cnt< 110)
	{
		FW_location_kp = FW_location_kp_stable; 
	}
    else if(diaoyong_cnt > (110 + change_kp_value) && diaoyong_cnt< 220)
	{
		FW_location_kp = FW_location_kp_stable; 
	}
	
////稳定拍照时的脱靶量误差
//	if(diaoyong_cnt == 70)
//    {
//		miss_distance_X_float_end1 = miss_distance_X_float;  //endx存的是稳定后
//		//start_end_arry[]数组存的是脱靶量相对误差
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end1 - miss_distance_X_float_start1;
//		//对脱靶量相对误差转存，可进行操作
//		miss_distance_X_float_sub1 = start_end_arry[start_end_cnt];
//		start_end_cnt++;   //start_end_arry[]数组存200个数
//	}
//	else if(diaoyong_cnt == 80)
//	{
//		miss_distance_X_float_end2 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end2 - miss_distance_X_float_start1;
//		miss_distance_X_float_sub2 = start_end_arry[start_end_cnt];
//		start_end_cnt++;
//	}
//    else if(diaoyong_cnt == 90)
//	{
//		miss_distance_X_float_end3 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end3 - miss_distance_X_float_start1;
//		miss_distance_X_float_sub3 = start_end_arry[start_end_cnt];			
//		start_end_cnt++;
//	}
//    else if(diaoyong_cnt == 100)
//	{
//		miss_distance_X_float_end4 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end4 - miss_distance_X_float_start1;
//		miss_distance_X_float_sub4 = start_end_arry[start_end_cnt];		
//		start_end_cnt++;
//	}
//	else if(diaoyong_cnt == 110)
//	{
//		miss_distance_X_float_end5 = miss_distance_X_float;
//	    start_end_arry[start_end_cnt] = miss_distance_X_float_end5 - miss_distance_X_float_start1;
//		miss_distance_X_float_sub5 = start_end_arry[start_end_cnt];		
//		start_end_cnt++;
//		
//		//一次阶跃计算5个点，累加求和再求平均

//		//********
//        if(store_open_flag == 1)
//		{
//			if(miss_distance_X_float_ave > error_set_value || miss_distance_X_float_ave < -error_set_value)	
//			{
//				store0 = miss_distance_X_float_start1;
//				store1 = miss_distance_X_float_end1;
//				store2 = miss_distance_X_float_end2;
//				store3 = miss_distance_X_float_end3;
//				store4 = miss_distance_X_float_end4;
//			    store5 = miss_distance_X_float_end5;	
//                store_open_flag	= 0;				
//			}
//            		
//		}
//	}
//	
//	else if(diaoyong_cnt == 180)
//	{
//		miss_distance_X_float_end6 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end6 - miss_distance_X_float_start2;
//		miss_distance_X_float_sub6 = start_end_arry[start_end_cnt];		
//		start_end_cnt++;
//	}
//	else if(diaoyong_cnt == 190)
//	{
//		miss_distance_X_float_end7 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end7 - miss_distance_X_float_start2;
//		miss_distance_X_float_sub7 = start_end_arry[start_end_cnt];			
//		start_end_cnt++;
//	}
//	else if(diaoyong_cnt == 200)
//	{
//		miss_distance_X_float_end8 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end8 - miss_distance_X_float_start2;
//		miss_distance_X_float_sub8 = start_end_arry[start_end_cnt];			
//		start_end_cnt++;
//	}
//	else if(diaoyong_cnt == 210)
//	{
//		miss_distance_X_float_end9 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end9 - miss_distance_X_float_start2;
//		miss_distance_X_float_sub9 = start_end_arry[start_end_cnt];			
//		start_end_cnt++;
//	}
//	else if(diaoyong_cnt == 220)
//	{
//		miss_distance_X_float_end10 = miss_distance_X_float;
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end10 - miss_distance_X_float_start2;
//		miss_distance_X_float_sub10 = start_end_arry[start_end_cnt];		
//		start_end_cnt++;
//		
//		diaoyong_cnt = 0;

//	}
//	
//	if(start_end_cnt == 201)
//	{
//		start_end_cnt = 0;  //start_end_arry[]数组存200个数，每5个存一组
//	}
////稳定拍照时的脱靶量误差
	X_slave_1ms_distance_sum1_2 = X_slave_1ms_distance_sum * 0.5;
	
	FW_step_chafen_speed = getFW_step_chafen_speed(global_FW_step_piror,Z_slave_1ms_distance_sum,1000);
    FY_step_chafen_speed = getFY_step_chafen_speed(global_FY_step_piror,X_slave_1ms_distance_sum1_2,1000);
	
	FW_wending_qiankui_speed_lowpass = FsmLeadLag1(FW_step_chafen_speed,FSM_X,2);     //稳定速度前馈滤波
	FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_step_chafen_speed,FSM_Y,2);
	//FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_chafen_speed,FSM_Y,2);        //稳定速度前馈滤波
	//FY_chafen_speed = getFY_chafen_speed(global_FY_position_piror,X_slave_1ms_distance_sum,1000);
	//DA12_out(FW_location_set,FW_location_set);
	
	
	//前置滤波
//	FW_pre_lowpss_filter = FsmLeadLag1(FW_simulated_target_motion,FSM_X,0);
//	FY_pre_lowpss_filter = FsmLeadLag1(FY_simulated_target_motion,FSM_Y,0);
//	
//	FW_location_set = FW_pre_lowpss_filter;
//	FY_location_set = FY_pre_lowpss_filter;
}