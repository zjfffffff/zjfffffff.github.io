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
u8 track_axis_select = 0;      //������ѡ��0������λ���� 1;����λ 2�������� 
u8 miss_cnt_10ms = 0;
u8 track_type_select = 1;        //��������   1���Ѱ���   2������
float FW_miss_distance_error_lowpass = 0;      //��λ�Ѱ���80Hz��ͨ�˲�
float FY_miss_distance_error_lowpass = 0;      //�����Ѱ���80Hz��ͨ�˲�
float FW_moni_miss = 0;
float FY_moni_miss = 0;
float FW_light_loop_k = 1;       //����2000
float FW_light_loop_revise = 0;                 //��λУ�����
float FW_light_loop_k_1 = 100;
float FW_miss_distance_error = 0;              //��λ�Ѱ���ͳһ��λ+K
float FW_light_loop_k_2 = 4000;  //4   //Ĭ�϶���У��    90    45
float FW_pi_tao = 0;
float FW_pi_w = 0;
float FW_pi_hz = 1;
float FW_miss_distance_error_lowpass_sum = 0;      //��λ�Ѱ���80Hz��ͨ�˲�
float FW_pi_K = 2;
u8 miss_distance_X_flag = 1;
float FY_pi_d1 = 0;
float FY_pi_i1 = 200;
float FY_pi_K1 = 100;
float FW_pi_i1 = 200;
float FW_pi_K1 = 100;
float FY_light_loop_k1 = -50;
float FY_light_loop_k = 200;       //��ջ���������K��   -0.1,-7000
float FY_light_loop_k_1 = 10;
float FY_light_loop_k_2 = 1;      //Ĭ�϶���У����   -50   -25
float FY_light_loop_revise = 0;                //����У�����
float FY_miss_distance_error = 0;              //�����Ѱ���ͳһ��λ+K
float FY_pi_tao = 0;
float FY_pi_w = 0;
float FY_pi_hz = 1;
float FY_miss_distance_error_lowpass_sum = 0;      //�����Ѱ���80Hz��ͨ�˲�
float FY_miss_distance_error_lowpass_last = 0;
float FY_miss_distance_light_diff = 0;
float FY_pi_K = 1;
u8 FW_light_para_set = 1;      //��ջ�У������ѡ�� 1��K  2:һ��У��  3������У��   4��PIУ��
u8 FY_light_para_set = 1;

extern float FW_Miss_distance;//��λ�Ѱ���
extern float FY_Miss_distance;//�����Ѱ���
extern u8 output_open_flag;
extern u8 FY_location_loop_open_flag;
extern u8 FW_location_loop_open_flag;
extern u8 FW_light_loop_open_flag;
extern u8 FY_light_loop_open_flag;
extern float FW_zero_degree_zheng;     //0.02 ��- 201.2021
extern float FY_zero_degree;         //0.06 ��- 323.02
extern float FY_location_set;    //�������ջ��趨ֵ
extern float FW_location_set;    //��λ�趨ֵ
extern float velocity_limit_value_set;
extern u8 FW_light_para;
extern u8 FY_light_para;
extern float FW_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float sin_value;          //sin����ֵ
extern u32 sin_cnt;              //sin����
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
extern float FY_revise_out;               //У�������K
extern float FY_Target_Position;


//
float miss_offset_X=0;//���õ�ƫ�÷�λ�Ѱ����ı���
float miss_offset_Y=0;//���õ�ƫ�÷�λ�Ѱ����ı���
extern float z_axis_velocity_float;
//ϵͳģʽ
extern u8 system_mode;

void ServoFunction(void)
{

	//X���Ѱ���
//	miss_distance_X_float = FW_Miss_distance; //�����ص㣩
	if(system_mode == 7)	//�Ѱ�������
	{
		miss_distance_X_float_to_angle = (miss_distance_X_float + miss_offset_X) * 29.5 * 0.000057295; //���Ѱ���ת��Ϊ�ȣ�
		miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
	}
	if(system_mode == 8)   //�ߵ�����
	{
		miss_distance_X_float_to_angle = (z_axis_velocity_float + miss_offset_X); //���Ѱ���ת��Ϊ�ȣ�
		miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
	}
//	miss_distance_X_float_to_angle = (z_axis_velocity_float + miss_offset_X) * 29.5 * 0.000057295; //���Ѱ���ת��Ϊ�ȣ�
	
	//          miss_distance_X_float_to_angle = miss_distance_X_float_to_angle * 0.5;
	//Y���Ѱ���
//	miss_distance_Y_float = FY_Miss_distance;
//	miss_distance_Y_float_to_angle = (miss_distance_Y_float + miss_offset_Y) * 77 * 0.000057295;;		//29.5 * 0.000057295
//	miss_distance_Y_float_to_angle = miss_distance_Y_float_to_angle * 0.5;	

	output_open_flag = 1;
	if(track_axis_select == 0)    //������ѡ��0������λ���� 1;����λ 2�������� 
																//track_axis_select = 0  ����λ���� 
	{
		FY_location_loop_open_flag = 0;
		FW_location_loop_open_flag = 0;
		FW_light_loop_open_flag = 1;
		FY_light_loop_open_flag = 1;
	}
	else if(track_axis_select == 1)       //����λ����������
	{
		FY_location_loop_open_flag = 1;
		FW_location_loop_open_flag = 0;
		FW_light_loop_open_flag = 1;
		FY_light_loop_open_flag = 0;
		
		//FW_location_set = FW_zero_degree_zheng;			
		FY_location_set = FY_zero_degree - 30;	
	}
	else if(track_axis_select == 2)   //����������λ����
	{
			FY_location_loop_open_flag = 0;
			FW_location_loop_open_flag = 1;
	    FW_light_loop_open_flag = 0;
	    FY_light_loop_open_flag = 1;

			FW_location_set = FW_zero_degree_zheng ;			
//			FY_location_set = FY_zero_degree; 				
	}

	velocity_limit_value_set = 90;             //60
	FW_light_para = FW_light_para_set;		//��ջ�У������ѡ�� 1��K  2:һ��У��  3������У��   4��PIУ��
	FY_light_para = FY_light_para_set;    //��ջ�У������ѡ�� 1��K  2:һ��У��  3������У��   4��PIУ��

	miss_distance_track();   //�ں�����У�� + �����ٶ��˲�
}

float Amp_sin = 3;
float FW_moni_offset = 294.9;
float FW_moni_mrad = 0;
float FY_moni_mrad = 0;
float FW_RMS_mrad = 0;
float FY_RMS_mrad = 0;
void miss_distance_track(void)
{
			//����ģ���Ѱ�����1ms����һ��ǰ���ٶȣ�10ms��һ��λ�����
		sin_value = Amp_sin*sin(sin_cnt / 2000.0 * 2 * 3.1415926);
		sin_cnt++;
		if(sin_cnt == 2000)
		{
			sin_cnt = 0;
		}
		
//		miss_cnt_10ms++;	
//		if(miss_cnt_10ms == 10)  //ÿ10ms���Ѱ�������һ�Σ���Ϊ�Ѱ������Ϊ100Hz
		{
					miss_cnt_10ms = 0;
			//		if(read_usart5[2] == 0x10) //�о����������ӳ�
					if(track_type_select == 1)   //�����Ѱ���   
					{
						//80Hz��ͨ�˲�
						//FW_miss_distance_error_lowpass = FsmLeadLag1_servo(FW_moni_miss,FSM_X,1);
						//FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle * cos(FW_encoder_degrees - FW_zero_degree_zheng) + miss_distance_Y_float_to_angle * sin(FW_encoder_degrees - FW_zero_degree_zheng);
//						FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle + cos(FY_encoder_degrees)*tan(FW_encoder_degrees);
						FW_miss_distance_error_lowpass = miss_distance_X_float_to_angle;
						
						//80Hz��ͨ�˲�
//						FY_miss_distance_error_lowpass = FsmLeadLag1_servo(miss_distance_Y_float_to_angle,FSM_Y,1);   
						//FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle * cos(FW_encoder_degrees - FW_zero_degree_zheng) - miss_distance_X_float_to_angle * sin(FW_encoder_degrees - FW_zero_degree_zheng); 
						//FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle + cos(FW_encoder_degrees)*tan(FY_encoder_degrees);
						FY_miss_distance_error_lowpass = miss_distance_Y_float_to_angle;
						
//						FW_RMS_mrad = CalculateRMS1(miss_distance_X_float);
//						FW_RMS_mrad = FW_RMS_mrad * 29.5;
//						FY_RMS_mrad = CalculateRMS2(miss_distance_Y_float);
//						FY_RMS_mrad = FY_RMS_mrad * 29.5;
					}
					else if(track_type_select == 2)  //��������
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
					//��λ�Ѱ������ټ���
					//��ջ�У������ѡ�� 1��K  
					if(FW_light_para == 1)	
					{
							FW_light_loop_revise = FW_light_loop_k * FW_miss_distance_error_lowpass / 10.0;   
					}
					//��ջ�У������ѡ�� 2��һ�׳�ǰ�ͺ�У�� 
					else if(FW_light_para == 2)		
					{
							FW_miss_distance_error = FW_light_loop_k_1 * FW_miss_distance_error_lowpass / 10.0;   
							FW_light_loop_revise = FsmLeadLag2_servo(FW_miss_distance_error,FSM_X,2);
					}
					//��ջ�У������ѡ�� 3������У��
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
					else if(FW_light_para == 4)		//��ջ�У������ѡ�� 4��PD У�� 
					{
						FW_pi_w =  FW_pi_hz * 2.0 * 3.1415926;
						FW_pi_tao = 1.0 / FW_pi_w;
						FW_miss_distance_error_lowpass_sum = FW_miss_distance_error_lowpass_sum + FW_miss_distance_error_lowpass;
						FW_light_loop_revise = FW_pi_K * (FW_pi_tao * FW_miss_distance_error_lowpass + 0.01 * FW_miss_distance_error_lowpass_sum);      //0.01�ǲ���ʱ��
					}
					else if(FW_light_para == 5)		//��ջ�У������ѡ�� 4��PD У�� 
					{
						if((miss_distance_X_float > 5 || miss_distance_X_float < -5) && (miss_distance_X_flag == 1))
						{
							FW_light_loop_revise = FW_light_loop_k * FW_miss_distance_error_lowpass / 10.0;  
						}
						else
						{
//							miss_distance_X_flag = 0;
							FW_miss_distance_error_lowpass_sum = FW_miss_distance_error_lowpass_sum + FW_miss_distance_error_lowpass * 0.01;	//0.01�ǲ���ʱ��
							FW_light_loop_revise = FW_pi_K1 * FW_miss_distance_error_lowpass + FW_pi_i1 * FW_miss_distance_error_lowpass_sum;      
						}
					}
		
	//�����Ѱ������ټ���
					if(FY_light_para == 1)
					{
							
							FY_light_loop_revise = FY_light_loop_k * FY_miss_distance_error_lowpass / 10.0; 
					}
					else if(FY_light_para == 2)
					{
							FY_miss_distance_error = FY_light_loop_k_1 * FY_miss_distance_error_lowpass / 10.0;     //�Ѱ�����λ�Ƕȣ�DA�����1V=10��/�룬ͳһ��λ
							FY_light_loop_revise = FsmLeadLag1_servo(FY_miss_distance_error,FSM_Y,2);	  //����У�� (T0.2��1)
					}
					else if(FY_light_para == 3)
					{
							FY_miss_distance_error = FY_light_loop_k_2 * FY_miss_distance_error_lowpass / 10.0;     //�Ѱ�����λ�Ƕȣ�DA�����1V=10��/�룬ͳһ��λ
							if(Song == 1)
							{
									FY_light_loop_revise = FsmLeadLag2_servo(FY_miss_distance_error,FSM_Y,7);	  //����У�� (T0.2��1)
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
							FY_light_loop_revise = FY_pi_K * (FY_pi_tao * FY_miss_distance_error_lowpass + 0.01 * FY_miss_distance_error_lowpass_sum);      //0.01�ǲ���ʱ��
					}
					else if(FY_light_para == 5)		//��ջ�У������ѡ�� 4��PD У�� 
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
					else if(FY_light_para == 6)		//��ջ�У������ѡ�� 4��PD У�� 
					{
						if((miss_distance_Y_float > 10 || miss_distance_Y_float < -10) && (miss_distance_X_flag == 1))
						{
							FY_light_loop_revise = FY_light_loop_k1 * FY_miss_distance_error_lowpass / 10.0; 
						}
						else
						{
							FY_miss_distance_error_lowpass_sum = FY_miss_distance_error_lowpass_sum + FY_miss_distance_error_lowpass * 0.001;	//0.01�ǲ���ʱ��,����
							FY_miss_distance_light_diff = (FY_miss_distance_error_lowpass - FY_miss_distance_error_lowpass_last)*1000.f;
							
							FY_light_loop_revise = FY_pi_K1 * FW_miss_distance_error_lowpass  + FY_pi_i1 * FY_miss_distance_error_lowpass_sum + FY_pi_d1 * FY_miss_distance_light_diff; 
							FY_miss_distance_error_lowpass_last = FY_miss_distance_error_lowpass;
						}
						
					}
		}	
//		else  //���Ѱ���ʱ�����ݲ���
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
	////ǰ���ٶȼ���
	//	if(track_miss_flag == 1)
	//	{
	//		//�����ٶ��˲�
	//		//�����ٶ�ǰ������������                             
	//		X_axis_palstance_float = X_axis_palstance_float * 1 * -0.5;
	//		FY_track_qiankui_speed_lowpass = FsmLeadLag1(X_axis_palstance_float,FSM_Y,4);     //�����ٶ�ǰ���˲�
	//		
	//		//�����ٶ�ǰ������λ����
	//		Z_axis_palstance_float = Z_axis_palstance_float * 1;
	//		FW_track_qiankui_speed_lowpass = FsmLeadLag1(Z_axis_palstance_float,FSM_X,4);     //�����ٶ�ǰ���˲�	
	//	}
	//	else if(track_miss_flag == 2)
	//	{
	//		//�ȶ��ٶ��˲�
	//		//IMU������΢�ֳ��ٶȣ�ÿ���ߵ�λ����ȡ��������
	//		X_slave_1ms_distance_sub = sin_value - X_slave_1ms_distance_sum_last;
	//		FY_qiankui_speed = X_slave_1ms_distance_sub * 1000 * 1;         //ͳһ��λ
	//		X_slave_1ms_distance_sum_last = sin_value;
	//		FY_track_qiankui_speed_lowpass = FsmLeadLag1(FY_qiankui_speed,FSM_Y,2);     //�ȶ��ٶ�ǰ���˲�
	//		
	//		//IMU������΢�ֳ��ٶȣ�ÿ���ߵ�λ����ȡ������λ
	//		Z_slave_1ms_distance_sub = sin_value - Z_slave_1ms_distance_sum_last;
	//		FW_qiankui_speed = Z_slave_1ms_distance_sub * 1000 * 1;
	//		Z_slave_1ms_distance_sum_last = sin_value;
	//		FW_track_qiankui_speed_lowpass = FsmLeadLag1(FW_qiankui_speed,FSM_X,2);     //�ȶ��ٶ�ǰ���˲� 
	//	}
*/
}