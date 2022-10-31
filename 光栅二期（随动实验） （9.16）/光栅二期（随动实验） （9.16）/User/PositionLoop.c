/*�ļ�������λ�͸���λ�ñջ�������
����ȫ�ֱ�����FW_location_set��FY_location_set����λ����
              FY_encoder_degrees_lowpass��FW_encoder_degrees_lowpass���������˲����ֵ ��λ����
���ȫ�ֱ�����FY_revise_out��FW_revise_out      ��λ����/��
2020.03.01*/
#include "stm32f4xx.h"
#include "FsmCompensator.h"
#include "PositionLoop.h"

float FX_set_prepass_xiuzheng = 0;	//�������
float FY_set_prepass_xiuzheng = 0;	//�������
///////////////////////////////////////////////
//����
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
float W_FW = 0.01;	//��ģ�����˲�ʱ�����
float W_FY = 0.01;	//��ģ�����˲�ʱ�����
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

extern float FY_location_set ;             //�������ջ��趨ֵ
extern float FW_location_set;    //��λ�趨ֵ
extern float FY_encoder_degrees_lowpass ;  //����������ת��Ϊ�Ƕ�
extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern u8 FY_location_para;
extern u8 system_mode;
float FW_revise_out = 0;               //��λУ��ֵ���ٶȣ�
float FY_revise_out = 0;               //У�������K
float FY_location_loop_k = 150;           //λ�ñջ����п�������K��
float FY_location_loop_k_1 = 30;          //λ�ñջ�һ��У����������K
float FY_location_loop_k_2 = 50;     	  //λ�ñջ�����У����������K	
float FY_location_low_pass = 0;           //80Hz��ͨ�˲�ֵ
float FY_leadlag1_out = 0;                //һ��У�����δ��K
float FY_leadlag2_out = 0;                //����У�����δ��K
float FY_location_loop_error = 0;         //�趨ֵ - ������ֵ    




extern float FW_encoder_degrees_lowpass;  //����������ת��Ϊ�Ƕ�
extern float FW_location_actual_set;    //��λ�趨ֵ
extern u8 FW_location_para;
float FW_location_loop_error = 0;    //�趨ֵ�������ֵ���
float FW_location_loop_actual_error = 0;    //�趨ֵ�������ֵ���
float FW_location_low_pass = 0;      //80Hz�˲������ٶȻ�г���
float FW_location_loop_k = 150;       //λ�ñջ����п�������K
float FW_location_loop_k_1 = 30;     //λ�ñջ�һ��У����������K
float FW_location_loop_k_2 = 1000;   //λ�ñջ�����У����������K
float FW_leadlag1_out = 0;           //��λһ���ͺ�ǰУ�����       
float FW_leadlag2_out = 0;           //��λ�����ͺ�ǰУ�����  
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

extern float FW_encoder_degrees;  //����������ת��Ϊ�Ƕ�

extern float X10_fsmTao1;		//w2���ڿ���������

extern float Y10_fsmTao1;		//w2���ڿ���������

u8 count_dazhi_FY = 0;
u8 count_dazhi_FW = 0;

extern float kp_guangshan_Y;
extern float kp_guangshan_tuob_Y; 
extern float kp_guangshan;
extern float kp_guangshan_tuob_X; 
void fy_close_loop(void)
{
		//�������ջ��趨ֵ 	FY_location_set = 322.8715	
		//����������ת��Ϊ�Ƕ� 	FY_encoder_degrees_lowpass = 0;  
		//�趨ֵ ������ֵ	=	�������ջ��趨ֵ + �������ֵ	-	 ����������ת��Ϊ�Ƕ�
		FY_location_loop_error = (FY_location_set + FY_set_prepass_xiuzheng) - FY_encoder_degrees; 
		if(FY_location_loop_error>40||FY_location_loop_error<-40)
		{
			count_dazhi_FY++;
		}
	
//		FY_Watch_shuyin_error	 = FY_location_set - FY_encoder_degrees_lowpass;
//		Watch_location_loop 	 = FY_location_set - FY_encoder_degrees;
//	
		FY_location_low_pass 	 = FY_location_loop_error;       //ʡ���˲����� 
		if(FY_location_para == 1)       //ֻ��K
		{
				if(system_mode == 10 || system_mode == 4|| system_mode == 3)
				{
						FY_location_loop_k = 100;	//λ�ñջ����п�������K��
				}
				else
				{
						FY_location_loop_k = FY_k_1;//FY_k_1 = 150
				}
				//У�������K	=	80Hz��ͨ�˲�ֵ	*	��������K��
				FY_revise_out = FY_location_low_pass * FY_location_loop_k;
		}
		else if(FY_location_para == 2)  //һ��У��
		{
	//		FY_location_loop_k_1 = Position_k;	
				//һ��У�����δ��K
				FY_leadlag1_out = FsmLeadLag1(FY_location_low_pass,FSM_Y,5); //һ���ͺ�ǰУ����FSM_Y=1
				//У�������K	=	λ�ñջ�һ��У����������K	*	һ��У�����δ��K	= 30 * 
				FY_revise_out = FY_location_loop_k_1 * FY_leadlag1_out;
		}
		else if(FY_location_para == 3)  //����У�� 
		{
				//����У�����δ��K	
				FY_leadlag2_out = FsmLeadLag2(FY_location_low_pass,FSM_Y,3); //�����ͺ�ǰУ��
				//У�������K	=	λ�ñջ�����У����������K	*	����У�����δ��K	= 50 *
				FY_revise_out = FY_location_loop_k_2 * FY_leadlag2_out;
		}
		else if(FY_location_para == 4)  //PIDУ��
		{
			  
				//80Hz�˲������ٶȻ�г���
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_loop_error;			//����
				//�趨ֵ ������ֵ FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //��
				FY_location_loop_error_last = FY_location_loop_error;
				

				//	= 0.1	*	
				jifen_fy = FY_location_ki * FY_location_loop_error_sum;
				//У�������K	=	115	*	������ֵ + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * FY_location_low_pass + jifen_fy+FY_location_kd * FY_location_loop_error_diff;
		}
		else if(FY_location_para == 5)  //PIDУ��+�۲���
		{
				//80Hz�˲������ٶȻ�г���
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//����
				//�趨ֵ ������ֵ FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //��
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
				//У�������K	=	115	*	������ֵ + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * FY_location_low_pass + jifen_fy+FY_location_kd * FY_location_loop_error_diff;
				FY_revise_out_2 = FsmLeadLag3(FY_revise_out,FSM_Y,8);
				FY_revise_out_1 = FsmLeadLag3(FY_revise_out,FSM_Y,9);
				FY_revise_out_3 = FY_revise_out_1 - FY_revise_out_2;
				FY_revise_out = FY_revise_out - Gain*FY_revise_out_3;
		}
		else if(FY_location_para == 6)  //��ģ����+�۲���
		{
				//80Hz�˲������ٶȻ�г���
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//����
				//�趨ֵ ������ֵ FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //��
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
				//У�������K	=	115	*	������ֵ + jifen_fy + kd * diff
				FY_revise_out = FY_location_kp * (FY_location_low_pass + jifen_fy + FY_location_kd * FY_location_loop_error_diff);
				FY_revise_out_2 = FsmLeadLag3(FY_revise_out,FSM_Y,8);
				FY_revise_out_1 = FsmLeadLag3(FY_revise_out,FSM_Y,9);
				FY_revise_out_3 = FY_revise_out_1 - FY_revise_out_2;
				FY_revise_out = FY_revise_out - Gain*FY_revise_out_3;
		}
		else if(FY_location_para == 7)  //2���ɶ���ģ����+�۲���
		{
				//80Hz�˲������ٶȻ�г���
				FY_location_loop_error_sum = FY_location_loop_error_sum + FY_location_low_pass;			//����
				//�趨ֵ ������ֵ FY_location_loop_error 
				FY_location_loop_error_diff = FY_location_loop_error - FY_location_loop_error_last; //��
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
				//У�������K	=	115	*	������ֵ + jifen_fy + kd * diff
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
		//�趨ֵ�������ֵ��� = ��λ�趨ֵ + �������ֵ -	����������ת��Ϊ�Ƕ� = 203.6674 - 0
		FW_location_loop_error = (FW_location_set + FX_set_prepass_xiuzheng) - FW_encoder_degrees;
		
		if(FW_location_loop_error>40||FW_location_loop_error<-40)
		{
			count_dazhi_FW++;
		}
		//�趨ֵ�������ֵ��� = ��λ�趨ֵ - ����������ת��Ϊ�Ƕ�
		FW_location_loop_actual_error = FW_location_actual_set - FW_encoder_degrees_lowpass;
		//FW_location_low_pass = FsmLeadLag1(FW_location_loop_error,FSM_X,1);      //һ���ͺ�У��20Hz�˲�
		FW_location_low_pass = FW_location_loop_error;//�趨ֵ�������ֵ���
	
		if(FW_location_para == 1)    //ֻ��K
		{
				//��λУ��ֵ���ٶȣ� = 80Hz�˲������ٶȻ�г��� * λ�ñջ����п�������K
				FW_revise_out = FW_location_low_pass * FW_location_loop_k;//FW_location_loop_k=150
		}
		else if(FW_location_para == 2)  //һ��У��
		{
				//��λһ���ͺ�ǰУ����� 
				FW_leadlag1_out = FsmLeadLag1(FW_location_low_pass,FSM_X,5);//һ���ͺ�ǰУ��
				//��λУ��ֵ���ٶȣ� = λ�ñջ�һ��У����������K * ��λһ���ͺ�ǰУ�����
				FW_revise_out = FW_location_loop_k_1 * FW_leadlag1_out;
		}
		else if(FW_location_para == 3)  //����У�� 
		{
				//��λ�����ͺ�ǰУ�����  
				FW_leadlag2_out = FsmLeadLag2(FW_location_low_pass,FSM_X,7);//�����ͺ�ǰУ��
				//��λУ��ֵ���ٶȣ� = λ�ñջ�����У����������K * ��λ�����ͺ�ǰУ�����  
				FW_revise_out = FW_location_loop_k_2 * FW_leadlag2_out;
		}
		else if(FW_location_para == 4)  //PIDУ��
		{
				//80Hz�˲������ٶȻ�г���FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//�趨ֵ�������ֵ��� FW_location_loop_error
				FW_location_loop_error_last = FW_location_loop_error;
				
				//jifen = 0.1 * sum
				jifen = FW_location_ki * FW_location_loop_error_sum;

				//��λУ��ֵ���ٶȣ� = 30 * 80Hz�˲������ٶȻ�г��� + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
		}
		else if(FW_location_para == 5)  //PIDУ��+�۲���
		{
				//80Hz�˲������ٶȻ�г���FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//�趨ֵ�������ֵ��� FW_location_loop_error
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

				//��λУ��ֵ���ٶȣ� = 30 * 80Hz�˲������ٶȻ�г��� + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;
		}
		else if(FW_location_para == 6)  //��ģ����+�۲���
		{
				//80Hz�˲������ٶȻ�г���FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//�趨ֵ�������ֵ��� FW_location_loop_error
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

				//��λУ��ֵ���ٶȣ� = 30 * 80Hz�˲������ٶȻ�г��� + jifen + kd * diff
				FW_revise_out = FW_location_kp * (FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff);
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;
		}
		else if(FW_location_para == 7)			//2���ɶ���ģ����+�۲���
		{
				//80Hz�˲������ٶȻ�г���FW_location_low_pass
				FW_location_loop_error_sum = FW_location_loop_error_sum + FW_location_low_pass;
				FW_location_loop_error_diff = FW_location_loop_error - FW_location_loop_error_last;
				//�趨ֵ�������ֵ��� FW_location_loop_error
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

				//��λУ��ֵ���ٶȣ� = 30 * 80Hz�˲������ٶȻ�г��� + jifen + kd * diff
				FW_revise_out = FW_location_kp * FW_location_low_pass + jifen +  FW_location_kd * FW_location_loop_error_diff;
				FW_revise_out_2 = FsmLeadLag3(FW_revise_out,FSM_X,8);   //Q  		out
				FW_revise_out_1 = FsmLeadLag3(FW_revise_out,FSM_X,9);		//Q/Gn  out
				FW_revise_out_3 = FW_revise_out_1 - FW_revise_out_2;		//Q/Gn - Q
				FW_revise_out = FW_revise_out - Gain*FW_revise_out_3;		//		
		}
}

//��λ����У������
void fw_tuoluo_close_loop(void)
{
//		FW_tuoluo_loop_error = FW_Tuoluo_last - FW_Tuoluo_bit_float;
		FW_tuoluo_loop_error = (FW_location_set + FX_set_prepass_xiuzheng) - FW_encoder_degrees;
		
		
		if(FW_tuoluo_para == 1)    //ֻ��K
		{
				FW_tuoluo_out = FW_tuoluo_kp * FW_Tuoluo_bit_float;
		}
		else if(FW_tuoluo_para == 2)  //һ��У��
		{
				
		}
		else if(FW_tuoluo_para == 3)  //����У�� 
		{
			
		}
		else if(FW_tuoluo_para == 4)  //PIDУ��
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

//��������У������		
void fy_tuoluo_close_loop(void)
{
		FY_tuoluo_loop_error = (FY_location_set + FY_set_prepass_xiuzheng) - FY_encoder_degrees; 
//		FY_tuoluo_loop_error = FY_limit_acc_value - FY_Tuoluo_bit_float;
		
		if(FY_tuoluo_para == 1)    //ֻ��K
		{
				FY_tuoluo_out = FY_tuoluo_kp * FY_Tuoluo_bit_float;
		}
		else if(FY_tuoluo_para == 2)  //һ��У��
		{
				
		}
		else if(FY_tuoluo_para == 3)  //����У�� 
		{
				
		}
		else if(FY_tuoluo_para == 4)  //PIDУ��
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
		else if(FY_tuoluo_para == 5)  //PIDУ��
		{
				
		}
}