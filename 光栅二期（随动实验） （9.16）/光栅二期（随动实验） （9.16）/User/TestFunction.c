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
extern float FW_zero_degree_zheng;     //0.02 ��- 201.2021
extern float FY_zero_degree; 
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float FW_location_set;    //��λ�趨ֵ
extern float FY_location_set;    //�������ջ��趨ֵ
extern unsigned int diaoyong_cnt;
extern float FW_location_kp;
extern float FW_location_loop_error ;    //�趨ֵ�������ֵ���
extern float FW_location_loop_actual_error;    //�趨ֵ�������ֵ���
extern float FW_step_chafen_speed;
extern float FY_step_chafen_speed;
extern float FW_wending_qiankui_speed_lowpass;  //��λǰ���ٶȵ�ͨ�˲�
extern float FY_wending_qiankui_speed_lowpass;   //�ȶ�ǰ���ٶ��˲�
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
	Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;  //Z��λ
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
	
	FW_location_set = FW_zero_degree_zheng - (FW_buchang_degree + buchang_offset_ly); //FW_buchang_degree�ں���Ծ
	FY_location_set = FY_zero_degree - FY_buchang_degree;
//��Kp,�ǳ�ʼ�Ѱ������ۼ���ƽ�����	
	if(diaoyong_cnt == 1)
	{
//		GPIO_SetBits(GPIOE,GPIO_Pin_2);//��ʱ��ƽ����
		FW_location_kp = FW_location_kp_step;    //��Ծ�׶�Kp=130
		//�ǽ�Ծ��ʼ�Ѱ���1
		//miss_distance_X_float_start1 = miss_distance_X_float;
	}
	else if(diaoyong_cnt == open_camera_value)                
	{
//		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//��ʱ��ƽ����
	}
	else if(diaoyong_cnt > open_camera_value && diaoyong_cnt< 110)  //�ȶ�����ʱ��
	{
		FW_rms_cnt++;
		FW_rms[FW_rms_cnt] = FW_location_loop_error;                //����������������
		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;               //����ۼ�
		
		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;         //ǰ���˲�ǰ����������������
		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum; //ǰ���˲�ǰ����������ۼ�
	}
	
	else if(diaoyong_cnt == 110)                         //��ʱ��ƽ����       
	{
//		GPIO_SetBits(GPIOE,GPIO_Pin_2);
		FW_rms_ave = FW_rms_sum * 0.025;//40����                //������������ֵ
		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;  //ǰ���˲�ǰ������������ֵ
		//FW_rms_ave = FW_rms[1];
		//FW_actual_rms_ave = FW_actual_rms[1];
		
		FW_rms_sum = 0; 
		FW_actual_rms_sum = 0;
		FW_rms_cnt = 0;
		FW_location_kp = FW_location_kp_step;   //��Ծ�׶�Kp=130
		//�ǽ�Ծ��ʼ�Ѱ���2
		//miss_distance_X_float_start2 = miss_distance_X_float;
	}
	else if(diaoyong_cnt == (110 + open_camera_value))   
	{
//		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//��ʱ��ƽ���� 
	}
	else if(diaoyong_cnt > (110 + open_camera_value) && diaoyong_cnt< 220)   //�ȶ�����ʱ��
	{
		FW_rms_cnt++;
		FW_rms[FW_rms_cnt] = FW_location_loop_error;     //����������������
		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;    //����ۼ�
		
		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;
		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum;
	}
	else if(diaoyong_cnt == 220)                      //����
	{
		diaoyong_cnt = 220;                 //��Ҫ����
	    FW_rms_ave = FW_rms_sum * 0.025;
		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;
		//FW_rms_ave = FW_rms[1];
		//FW_actual_rms_ave = FW_actual_rms[1];	
		FW_rms_sum = 0;
	    FW_actual_rms_sum = 0;
		FW_rms_cnt = 0;
	}
	
//�ȶ����ս׶��л��߿�������
	if(diaoyong_cnt > change_kp_value && diaoyong_cnt< 110)
	{
		FW_location_kp = FW_location_kp_stable; 
	}
    else if(diaoyong_cnt > (110 + change_kp_value) && diaoyong_cnt< 220)
	{
		FW_location_kp = FW_location_kp_stable; 
	}
	
////�ȶ�����ʱ���Ѱ������
//	if(diaoyong_cnt == 70)
//    {
//		miss_distance_X_float_end1 = miss_distance_X_float;  //endx������ȶ���
//		//start_end_arry[]���������Ѱ���������
//		start_end_arry[start_end_cnt] = miss_distance_X_float_end1 - miss_distance_X_float_start1;
//		//���Ѱ���������ת�棬�ɽ��в���
//		miss_distance_X_float_sub1 = start_end_arry[start_end_cnt];
//		start_end_cnt++;   //start_end_arry[]�����200����
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
//		//һ�ν�Ծ����5���㣬�ۼ��������ƽ��

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
//		start_end_cnt = 0;  //start_end_arry[]�����200������ÿ5����һ��
//	}
////�ȶ�����ʱ���Ѱ������
	X_slave_1ms_distance_sum1_2 = X_slave_1ms_distance_sum * 0.5;
	
	FW_step_chafen_speed = getFW_step_chafen_speed(global_FW_step_piror,Z_slave_1ms_distance_sum,1000);
    FY_step_chafen_speed = getFY_step_chafen_speed(global_FY_step_piror,X_slave_1ms_distance_sum1_2,1000);
	
	FW_wending_qiankui_speed_lowpass = FsmLeadLag1(FW_step_chafen_speed,FSM_X,2);     //�ȶ��ٶ�ǰ���˲�
	FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_step_chafen_speed,FSM_Y,2);
	//FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_chafen_speed,FSM_Y,2);        //�ȶ��ٶ�ǰ���˲�
	//FY_chafen_speed = getFY_chafen_speed(global_FY_position_piror,X_slave_1ms_distance_sum,1000);
	//DA12_out(FW_location_set,FW_location_set);
	
	
	//ǰ���˲�
//	FW_pre_lowpss_filter = FsmLeadLag1(FW_simulated_target_motion,FSM_X,0);
//	FY_pre_lowpss_filter = FsmLeadLag1(FY_simulated_target_motion,FSM_Y,0);
//	
//	FW_location_set = FW_pre_lowpss_filter;
//	FY_location_set = FY_pre_lowpss_filter;
}