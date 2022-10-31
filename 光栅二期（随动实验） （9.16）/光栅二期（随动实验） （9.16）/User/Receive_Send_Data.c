#include "Receive_Send_Data.h"
#include "stm32f4xx.h"
#include "elmo.h"
#include "math.h"
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"
extern double jiao_1,jiao_2;
extern float yaw_attitude_float_ly ;
extern float pitch_attitude_float_ly ;
extern float roll_attitude_float_ly ;
extern float FW_tuoluo_loop_error;
extern float FY_tuoluo_loop_error;
float GS_Suidong_Flag=0;
extern float GS_FW_DEG,GS_FY_DEG,GS_HG_DEG;;
float GS_Suidong_offset_FW=0;
float GS_Suidong_offset_FY=0;
int Suidong_jishu=0;
float FY_Degree_GS_suidong,FW_Degree_GS_suidong;
/////////////////////////////////
//���ŷ����ջ����ܿذ������
u16 baochang = 0;
u16 jieshoushijian = 0;
int zhenjishu = 0;
extern float Tuoluo_bit_float_k_2;
extern float FW_Tuoluo_bit_float ;		//��λ�������ݽ��ٶ�
extern float FY_Tuoluo_bit_float ;		//�����������ݽ��ٶ�
extern u8 GOY_K;
extern float k_11 ;
extern float GOY_K2 ;
extern float GOY_K1;


extern float FW_location_set ;    //��λ�趨ֵ
extern float FY_location_set ;    //�������ջ��趨ֵ

int genzon_biaozhi = 0;
float Four_Quadrant_bit_X = 0;					//������X
float Four_Quadrant_bit_Y = 0;					//������Y

float Shangweiji_Pitch_Angle_Value_prepass = 0;
float Tuoluo_bit_float12 = 0;
float Tuoluo_bit_float11 = 0;
float Tuoluo_bit_float_k = 1.82;
float Tuoluo_bit_float_k1 = - 0.019;
float Tuoluo_bit_float1 = 0;
float Tuoluo_bit_float_Y1 = 0;	
float Tuoluo_bit_float = 0;							//�������ݴ��ؽ��ٶ�
float Tuoluo_bit_float_Y = 0;						//�������ݴ��ؽ��ٶ�
extern float touluo_biaoduyingsu;				//���ݱ������
s32 Tuoluo_bit_u32_X1;								//����X
s32 Tuoluo_bit_u32_Y1;								//����Y

u8 XunHuan_Ma = 0;
u8 Receive_XunHuan_Ma_low = 0;	//����ѭ����
u8 Receive_XunHuan_Ma_high = 0;	
u8 Send_XunHuan_Ma_TX = 0;				//������ܿط���ѭ����

extern float miss_offset_X;				//���õ�ƫ�÷�λ�Ѱ����ı���
extern float miss_offset_Y;				//���õ�ƫ�÷�λ�Ѱ����ı���

//extern float Point_FY_set_prepass;
u8 Receive_Data[Receive_RX_Len];
u8 Receive_TXData[Receive_TX_Len];
u8 Receive_DBJ_Data[Receive_Suidong_GS_Len];
//extern u8 SF_mode;					//�������� 1:ָ�� 2:���� 3:���� 4:ֹͣ   5:�������
//extern float FW_pointing_degree_LY;
//extern float FY_pointing_degree_LY;
s32 	Receive_Direction_Angle_Value;	//ָ��λλ��
s32 	Receive_Pitch_Angle_Value;			//ָ����λ��
u8 Receive_IP;											//�����ŷ�IP��
u8 Receive_suidong_IP; //danbaijingzhuangtai
typedef union
{
	struct
	{
		u8 Low_byte;
		u8 MLow_byte;
		u8 MHigh_byte;
		u8 High_byte;
	}Receive_Data_byte;
	float Receive_Data_float;
	u32 Data_u32;	  //�޷�������
	s32 Data_s32;     //�з�������
}Receive_Data_value;
Receive_Data_value Temp_32_Receive;
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//���ŷ����������ܿذ������
//typedef union
//{
//	struct
//	{
//		u8 Low_byte;
//		u8 MLow_byte;
//		u8 MHigh_byte;
//		u8 High_byte;
//	}Send_Data_byte;
//	float Send_Data_float;
//	u32 Send_Data_u32;	  //�޷�������
//	s32 Send_Data_s32;     //�з�������
//}Send_Data_value;
//Send_Data_value Temp_32_Send;

extern float x_axis_velocity_float ;
extern float y_axis_velocity_float ;
extern float z_axis_velocity_float ;

extern float FW_encoder_degrees;  //��λ������ת��Ϊ�Ƕ�
extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�

extern float Speed_encode_FY;
extern float Speed_encode_FW;

int cugenzongmoshi = 0;
int cugenzongzhiling = 0;

extern float miss_distance_X_float;
extern float miss_distance_Y_float;

float Send_Direction_Angle_AnswerValue;	//ָ��λλ��
float Send_Pitch_Angle_AnswerValue;		//ָ����λ��

float FW_Jixia_tuoba_degrees = 0;		//�Ѱ���X
float FY_Jixia_tuoba_degrees = 0;		//�Ѱ���Y

float FX_JixiaTuoluo_bit_float = 0; //����X
float FY_JixiaTuoluo_bit_float = 0;	//����Y

float Send_Four_Quadrant_bit_X = 0; //������X
float Send_Four_Quadrant_bit_Y = 0; //������Y

float Send_com_En = 0x02;				//�ָ���ʹ��:0x01:ж�أ�0x02��ʹ��

extern u8 system_mode; 					//
u8 Send_gai_Status = 0;					//�ָ���ģʽ
u8 Send_mode;										//�ָ���ָ�� 1:ָ�� 2:������� 3:������ 4:������5:���ŷ�ʹ��  6:ֹͣ 7:ͼ������л�
u8 Send_Status = 0;					      	//�ָ���ָ��ִ��״̬ 0x00:���� 0x01:ִ���� 0x02:ִ���� 0x03:δִ��
u8 FY_Xianwei;									//������λ
u8 Send_Err1;										//���ŷ����ϴ���1
u8 Send_Err2;										//���ŷ����ϴ���2
u8 Send_CAMERA_Err = 0;					//�ߵ�ͨ��״̬ 0x00:ͨ������ 0x01:ͨ�Ź���

float POS_SP_SWITCH = 0;        //�����ٶȻ������λ��

///////////////////////////////////////////////////////////////////////////////////////////
///���ŷ����ջ���λ��������
u8 Shangweiji_Data[Shangweiji_RX_Len];	
u8 Shangweiji_Data_len = 0;             //���ݰ�����
u8 Shangweiji_Command;									//ָ��ִ��״̬ 
float 	Shangweiji_Direction_Angle_Value;				//ָ��λλ��
float 	Shangweiji_Pitch_Angle_Value;						//ָ����λ��
float GS_Suidong_FW=0.01;
float GS_Suidong_chafen_FW;
float GS_Suidong_chafen_FY;
float GS_Suidong_FW_before;
float GS_Suidong_FY_before;
float GS_Suidong_FY=0.01;

float Shangweiji_Direction_Angle_prepass = 0;

extern float FW_pointing_degree_LY;
//extern float FY_pointing_degree_LY;
extern float Point_FY_set_prepass;
float Shangweiji_Send_time = 0;						//��λ���������ݷ���ʱ��
u8 BaoJishu_Ma = 0;												//��������
char Shineng_biaozhi = 0;										//ʹ�ܱ�־
char lici_biaozhi = 0;	                    //���ű�־
extern float FX_set_prepass_xiuzheng;
extern float FY_set_prepass_xiuzheng;
//////////////////////////////////////////////////////////////////////////////////////////

unsigned int touluo_Cnt1 = 0;	//�����жϵ��ܴ���
unsigned int touluo_Cnt2 = 0;	//֡ͷ����
unsigned int touluo_Cnt3 = 0;	//֡δ����
unsigned int touluo_Cnt4 = 0;	//У��ʹ���
unsigned int touluo_True = 0;	//������ȷ
extern double jiao1,jiao2;//di 1 zu jie
extern double jiao3,jiao4;//di 2 zu jie

extern float FY_zero_degree;              //0.06 ��- 323.02  2 

extern double FW_Degree_GS;
extern double FY_Degree_GS;

//float MissKx=0.72,MissKy=0.72;
float MissKx=1,MissKy=1;
//���ŷ����ջ����ܿذ������
void Receive_DATA_Parse()
{
		u8 i=0,Sum=0;
		touluo_Cnt1 ++;			        //�����жϵ��ܴ���
		if((Receive_Data[0]==0xEB) & (Receive_Data[1]==0x90))//֡ͷ
		{
				if(0==0)//֡β
				{
						Sum=0;
				for(i=0;i<(Receive_RX_Len-1);i++)
					 {
						Sum+=Receive_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_Data[Receive_RX_Len-1]==(Sum))//���У��
			
						{	
								touluo_True ++;	//������ȷ
							  baochang  = Receive_Data[3];
								Receive_IP = Receive_Data[4];								
							if(Receive_IP == 6)/////////////////�ָ����ŷ�ָ��/////
							{
									  system_mode = 17;
										
										//�ָ����ŷ���λ
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[5];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[6];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[7];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[8];
										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Data_s32; 
//							    	Shangweiji_Direction_Angle_prepass = FsmLeadLag1(Shangweiji_Direction_Angle_Value,FSM_X,0);
								
										FW_Degree_GS = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
										//�ָ����ŷ�����
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[9];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[10];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[11];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[12];
										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Data_s32; 
//								    Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(Shangweiji_Pitch_Angle_Value,FSM_Y,0);
								
								
										FY_Degree_GS = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
							}
							else if(Receive_IP == 7)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
//							system_mode = 26;	
							}
							else if(Receive_IP == 8)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
//							system_mode = 26;	
							}
							else if(Receive_IP == 9)/////////////�ָ����ŷ���������/////�����Խ׶���Ϊ����ģʽ��
							{
//								    system_mode = 25;
										
//										//�ָ����ŷ���λ
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[6];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[7];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[8];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Direction_Angle_Value = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//										//�ָ����ŷ�����
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[10];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[11];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[12];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Pitch_Angle_Value = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//									
//										miss_offset_X = Shangweiji_Direction_Angle_Value;  //�������õ�ƫ�÷�λ�Ѱ���
//										miss_offset_Y = Shangweiji_Pitch_Angle_Value;      //�������õ�ƫ�ø����Ѱ���
							}
							else if(Receive_IP == 18)///////////////�ָ����ŷ�����
							{
								    system_mode = 3;
							}
							else if(Receive_IP == 19)/////////////�ָ����ŷ�ʹ��
							{
						      	Shineng_biaozhi = Receive_Data[5];
										if(Shineng_biaozhi == 0)	 //ʧ��
										{
												system_mode = 2;
										}
										if(Shineng_biaozhi == 1)   //ʹ��
										{
												system_mode = 4;
										}
							}
							else if (Receive_IP == 20)/////////////�ָ����ŷ�ֹͣ
							{
								    system_mode = 1;
							}
							else if (Receive_IP == 21)///////////////�ָ����ŷ��ջ�
							{
										lici_biaozhi = Receive_Data[5];
										if (lici_biaozhi == 2)///////////////�ɼ���ջ�
										{
//											system_mode = 7;
										}
										else if (lici_biaozhi == 1)/////////////////����ջ�
										{
										}    
							}
							else if (Receive_IP == 25)////////////////////����/////////////
							{
							}
							else if (Receive_IP == 26)///////////////////�ղ�////////////////
							{
							}
							else if (Receive_IP == 27)/////////////////չ��///////////////
							{
							}
							else if (Receive_IP == 30)////////////////////�ָ��������л�/////////////
							{	
							}
							else if (Receive_IP == 33)////////////////�־��ŷ�״̬��ѯ//////////////////////
							{	
							}
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[24];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[25];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[26];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[27];
										jieshoushijian =Temp_32_Receive.Receive_Data_float;
							
							 
						}
						else
						{
							touluo_Cnt4 ++;	//У��ʹ���
						}
				}
				else
				{
					touluo_Cnt3 ++;	//֡δ����
				}
		}
		else
		{
			touluo_Cnt2 ++;			//֡ͷ����
		}
		Receive_XunHuan_Ma_low = Receive_Data[31];					//ѭ����
		Receive_XunHuan_Ma_high = Receive_Data[32];
	 zhenjishu = Receive_XunHuan_Ma_high + Receive_XunHuan_Ma_low>>8;
}

//���ŷ����ջ���λ��������
void Receive_Shangweiji_DATA_Parse()
{
//		u16 i=0,Sum=0;
//		if((Shangweiji_Data[0]==0xEB) & (Shangweiji_Data[1]==0x90))//֡ͷ
//		{
//				Shangweiji_Data_len = Shangweiji_Data[2];			//���ݰ�����43
//				if(Shangweiji_Data[Shangweiji_RX_Len-1]==0xFE)//֡β
//				{
//						for(i=2;i<(Shangweiji_RX_Len-2);i++)
//						{
//								Sum += Shangweiji_Data[i];
//						}
//						if(Shangweiji_Data[Shangweiji_RX_Len-2]==Sum)//���У��
//						{
//								Shangweiji_Command = Shangweiji_Data[3];	//ָ��״̬ 1:ָ�� 2:������� 3:�������� 4:���� 5:ʹ�� 6:ֹͣ
//								
//								if(Shangweiji_Command == 1)    //ָ��
//								{
//										system_mode = 17;
//										
//										//�ָ����ŷ���λ
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FW_pointing_degree_LY = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//										//�ָ����ŷ�����
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Point_FY_set_prepass = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//								}
//								if(Shangweiji_Command == 2)				//�������
//								{
//										system_mode = 3;
//									
//										//�ָ����ŷ���λ
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FX_set_prepass_xiuzheng = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//										//�ָ����ŷ�����
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FY_set_prepass_xiuzheng = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//								}
//								if(Shangweiji_Command == 3)    //��������
//								{
//										system_mode = 7;
//										
//										//�ָ����ŷ���λ
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Direction_Angle_Value = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//										//�ָ����ŷ�����
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Pitch_Angle_Value = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//uradת��;
//									
//										miss_offset_X = Shangweiji_Direction_Angle_Value;  //�������õ�ƫ�÷�λ�Ѱ���
//										miss_offset_Y = Shangweiji_Pitch_Angle_Value;      //�������õ�ƫ�ø����Ѱ���
//								}
//								if(Shangweiji_Command == 4)    //����
//								{
//										system_mode = 3;
//								}
//								if(Shangweiji_Command == 5)    //ʹ�ܱ�־
//								{	
//										Shineng_biaozhi = Shangweiji_Data[5];
//										if(Shineng_biaozhi == 1)	 //ʧ��
//										{
////												wMO_FW(0x37F,0);
////												wMO_FY(0x37F,0);
//												system_mode = 2;
//										}
//										if(Shineng_biaozhi == 2)   //ʹ��
//										{
////												wMO_FW(0x37F,1);
////												wMO_FY(0x37F,1);
//												system_mode = 4;
//										}
//								}
//								if(Shangweiji_Command == 6)    //ֹͣ
//								{
//										system_mode = 1;
//								}
//								
////								//�������ݷ���ʱ�䣨ms��
////								Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[36];
////								Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[37];
////								Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[38];
////								Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[39];
////								Shangweiji_Send_time = Temp_32_Receive.Receive_Data_float; 
//								
//								//��������
////								BaoJishu_Ma =	Shangweiji_Data[40];
//						}
//				}
//		}
}

extern float FY_miss_distance_light_diff;

u8 tt = 0;
extern float FY_encoder_quzhi;
extern float FY_output_value_da;
extern float x1_axis_velocity_float;
extern float x_axis_velocity_1msdistance_sum ; 
extern float x_axis_velocity_1msdistance_sum1;
extern float FY_location_loop_error;
extern float FW_location_loop_error;
extern float z_axis_velocity_1msdistance_sum;
extern double *Deg_2_1;
extern double *Deg_1_1;
void Receive_DATA_Parse_2th()
{
	u16 i=0;
	u8 Sum=0;
	touluo_Cnt1 ++;			        //�����жϵ��ܴ���
	if((Receive_Data[0]==0xEB) & (Receive_Data[1]==0x90))//֡ͷ
	{
		Sum=0;
		for(i=0;i<(Receive_RX_Len-1);i++)
		{
			Sum+=Receive_Data[i];
		}
		if(Receive_Data[Receive_RX_Len-1]==Sum && Receive_Data[2]==0x1E)//У��ֵ�ͳ���
		{
			touluo_True ++;	//������ȷ
			baochang  = Receive_Data[2];
			Receive_IP = Receive_Data[3];
			
     if(Receive_IP == 2)/////////////////�ָ����ŷ�ָ��/////
							{
									  system_mode = 17;	
								//��դ�Ƕ�1
				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
				Shangweiji_Direction_Angle_Value = Temp_32_Receive.Data_s32; 
				FW_Degree_GS = Shangweiji_Direction_Angle_Value/10000; 
//				FW_Degree_GS = Temp_32_Receive.Receive_Data_float;
				//��դ�Ƕ�2
				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
				Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Data_s32; 
				FY_Degree_GS = Shangweiji_Pitch_Angle_Value/10000; 							//*360/2/3.14/
//				FY_Degree_GS = Temp_32_Receive.Receive_Data_float; 							//*360/2/3.14/1000/1000;//uradת��;
							}
							else if(Receive_IP == 3)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
							system_mode = 25;	
							}
							else if(Receive_IP == 4)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
							system_mode = 3;	
							}
							else if(Receive_IP == 5)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
							system_mode = 1;	
							}
			        else if(Receive_IP == 0x40)///////////////�ָ����ŷ�ɨ��/////(���Խ׶���Ϊ�ȶ�ģʽ)
							{
							if(Receive_Data[4]==1)
							{system_mode = 2;}
              if(Receive_Data[4]==2)
							{system_mode = 4;}							
							}
//			switch (Receive_IP)
//			{
//			case 2:
//				system_mode = 17;
//				//��դ�Ƕ�1
//				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
//				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
//				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
//				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
//				FW_Degree_GS = Temp_32_Receive.Receive_Data_float;
//				//��դ�Ƕ�2
//				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
//				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
//				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
//				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
//				FY_Degree_GS = Temp_32_Receive.Receive_Data_float;
//				break;
//			case 3:
////				//˫��դ�ŷ���λ�Ƕ�
////				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
////				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
////				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
////				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
////				Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float;
////				FW_Degree_GS = Shangweiji_Direction_Angle_Value;
////				//˫��դ�ŷ������Ƕ�
////				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
////				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
////				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
////				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
////				Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float;
////				FY_Degree_GS = Shangweiji_Pitch_Angle_Value;
//			system_mode = 25;
//				break;
//			case 4:    //˫��դ����
//				system_mode = 3;
//				break;
////			case 4:    //˫��դ����
////				system_mode = 3;
////				break;
//			case 5:    //˫��դֹͣ
//				system_mode = 1;
//				break;
//			default:
//				break;
//			}
		}
		else
		{
			touluo_Cnt4 ++;	//У��ʹ���
		}
	}
	else
	{
		touluo_Cnt2 ++;			//֡ͷ����
	}

	//�������ݷ���ʱ��(ms)
	Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[24];
	Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[25];
	Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[26];
	Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[27];
	Shangweiji_Send_time = Temp_32_Receive.Receive_Data_float;

	//��������
	BaoJishu_Ma = Receive_Data[28];
}
//���ŷ����������ܿذ������
void Send_Data_TX(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//֡ͷ
	Receive_TXData[2] = 49;
	Temp_32_Receive.Data_s32 = (FW_encoder_degrees)*10000;
//	Temp_32_Receive.Data_s32 = (GS_Suidong_chafen_FW)*10000;
//	Temp_32_Receive.Data_s32 = (jiao_1)*10000;
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	Temp_32_Receive.Data_s32 = (FY_encoder_degrees )*10000;
//	Temp_32_Receive.Data_s32 = (GS_Suidong_chafen_FY )*10000;
//	Temp_32_Receive.Data_s32 = (jiao_2)*10000;
	Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	if(POS_SP_SWITCH == 1)
	{
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FW * 1000.0f);
		Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FY * 1000.0f);
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	}
	if(system_mode == 4)
	{
		Temp_Uchar = 1;
	}
	else if(system_mode == 2 )
	{
		Temp_Uchar = 0;
	}
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{

		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{

		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// �ߵ�///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
//	Temp_32_Receive.Data_s32 = z_axis_velocity_float*10000;		//��λ�ٶ�
//	Temp_32_Receive.Data_s32 = GS_FW_DEG*10000;		//��λ�ٶ�
	Temp_32_Receive.Data_s32 = yaw_attitude_float_ly*10000;		//��λ�ٶ�9.16
//	Temp_32_Receive.Data_s32 = FW_location_loop_error*10000;		//��λ�ٶ�9.16
	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
//	Temp_32_Receive.Data_s32 = x_axis_velocity_float*10000; 	// �����ٶ�
//	Temp_32_Receive.Data_s32 = GS_FY_DEG*10000;		//��λ�ٶ�
	Temp_32_Receive.Data_s32 = pitch_attitude_float_ly*10000; 	// �����ٶ�9.16
//	Temp_32_Receive.Data_s32 = FY_location_loop_error*10000; 	// �����ٶ�9.16
	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
//	Temp_32_Receive.Data_s32 = y_axis_velocity_float*10000;		//����ٶ�
//	Temp_32_Receive.Data_s32 = GS_HG_DEG*10000;		//��λ�ٶ�
	Temp_32_Receive.Data_s32 = roll_attitude_float_ly*10000;		//�����̬
	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//  Receive_TXData[31] = 0x00;
//  Receive_TXData[32] = 0x00;
//	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[31] = 0;
	//		//�ָ����ŷ��Ѱ���X
//		FW_Jixia_tuoba_degrees = FW_location_loop_error*100;
		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float*100*15.7;
		Temp_32_Receive.Data_s32 = FW_Jixia_tuoba_degrees;
		Receive_TXData[32] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//�ָ����ŷ��Ѱ���Y
//		FY_Jixia_tuoba_degrees = FY_location_loop_error*100;
		FY_Jixia_tuoba_degrees = MissKy*miss_distance_Y_float*100*15.7;
		Temp_32_Receive.Data_s32 = FY_Jixia_tuoba_degrees;
		Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[37] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[38] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[39] = Temp_32_Receive.Receive_Data_byte.High_byte;
	  Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
	  for(i = 2; i < (Receive_TX_Len - 2); i++)
			{
				Sum += Receive_TXData[i];
			}
		Receive_TXData[Receive_TX_Len - 2] = (Sum & 0x00FF);//���У��
		Receive_TXData[Receive_TX_Len - 1] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}
///////////////////////625//////////////////
//		Receive_TXData[36] = Send_gai_Status;			//�ָ���ģʽ
//		Receive_TXData[37] = Shangweiji_Command;  //�ָ���ָ��
//		Receive_TXData[38] = Send_Status;					//�ָ���ָ��ִ��״̬
//		Receive_TXData[39] = FY_Xianwei;					//������λ
//		Receive_TXData[40] = Send_Err1;						//���ŷ����ϴ���1
//		Receive_TXData[41] = Send_Err2;						//���ŷ����ϴ���2
//		
	
//		Receive_TXData[78] = Send_CAMERA_Err; //�ߵ�ͨ��״̬	
		Send_JiXiaData_Write();
}




//���ŷ�������λ��������  2022.8.16
void Send_SW_Data_TX(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//֡ͷ
	
//	Temp_32_Receive.Receive_Data_float = (FW_encoder_degrees);
	Temp_32_Receive.Receive_Data_float = miss_distance_X_float * 15.7;
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[2] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Temp_32_Receive.Receive_Data_float = (FY_encoder_degrees );
	Temp_32_Receive.Receive_Data_float = miss_distance_Y_float * 15.7;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.Low_byte;

/*	
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{
//		case :                          ////////////ָ��
//			cugenzongmoshi = 15;
//			break;
//		case :                           ////////////ɨ��
//			cugenzongmoshi = 16;
//			break;
//		case :                       ///////////////��������
//			cugenzongmoshi = 17;
//			break;
//		case :                       ///////////////����
//			cugenzongmoshi = 18;
//			break;
//		case :                 //////////////ʹ��
//			cugenzongmoshi = 19;
//			break;
//		case :                 //////////////ֹͣ
//			cugenzongmoshi = 20;
//			break;
//		case :                     /////////////�ջ�
//			cugenzongmoshi = 21;
//			break;
//		case :                  ////////////////Ŀ��������
//			cugenzongmoshi = 25;
//		  break;
//		case :             ///////////////////�ղ�
//			cugenzongmoshi = 26;
//			break;
//		case :          //////////////////չ�� 
//			cugenzongmoshi = 27;
//			break;
//		case :                     ///////////�ָ��������л�
//			cugenzongmoshi = 30;
//			break;
//		case :                  //////////////�־��ŷ�״̬��ѯ
//			cugenzongmoshi = 33;
//		  break;
		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{
//		case 15:                          ////////////ָ��
//			cugenzongzhiling = 15;
//			break;
//		case 16:                           ////////////ɨ��
//			cugenzongzhiling = 16;
//			break;
//		case 17:                       ///////////////��������
//			cugenzongzhiling = 17;
//			break;
//		case 18:                       ///////////////����
//			cugenzongzhiling = 18;
//			break;
//		case 19:                 //////////////ʹ��
//			cugenzongzhiling = 19;
//			break;
//		case 20:                 //////////////ֹͣ
//			cugenzongzhiling = 20;
//			break;
//		case 21:                     /////////////�ջ�
//			cugenzongzhiling = 21;
//			break;
//		case 25:                  ////////////////Ŀ��������
//			cugenzongzhiling = 25;
//		  break;
//		case 26:             ///////////////////�ղ�
//			cugenzongzhiling = 26;
//			break;
//		case 27:          //////////////////չ�� 
//			cugenzongzhiling = 27;
//			break;
//		case 30:                     ///////////�ָ��������л�
//			cugenzongzhiling = 30;
//			break;
//		case 33:                  //////////////�־��ŷ�״̬��ѯ
//			cugenzongzhiling = 33;
//		  break;
		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// �ߵ�///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
*/

//	Temp_32_Receive.Receive_Data_float = z_axis_velocity_float;		//��λ�ٶ�
//	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	
////	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
//	Temp_32_Receive.Receive_Data_float = x_axis_velocity_float; 	// �����ٶ�
//	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	
////	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
//	Temp_32_Receive.Receive_Data_float = y_axis_velocity_float;		//����ٶ�
//	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
////  Receive_TXData[31] = 0x00;
////  Receive_TXData[32] = 0x00;
////	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
////	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
////	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
////	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[31] = 0;
	//		//�ָ����ŷ��Ѱ���X
//		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float;
		    FW_Jixia_tuoba_degrees = FW_encoder_degrees;
		Temp_32_Receive.Receive_Data_float = FW_Jixia_tuoba_degrees;
		Receive_TXData[13] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[12] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[11] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//�ָ����ŷ��Ѱ���Y
//		FY_Jixia_tuoba_degrees = x_axis_velocity_1msdistance_sum;
		FY_Jixia_tuoba_degrees = FY_encoder_degrees;
		Temp_32_Receive.Receive_Data_float = FY_Jixia_tuoba_degrees;
		Receive_TXData[17] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[16] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[15] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[14] = Temp_32_Receive.Receive_Data_byte.High_byte;
		
//	  for(int i=2;i<33;i++)
//		{
//				Receive_TXData[i] = 0;
//		}
			Receive_TXData[36]++;
//		Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
			Sum = 0;
			for(i = 2; i < 37; i++)
			{
				Sum += Receive_TXData[i];
			}
//		Receive_TXData[34] = (Sum & 0x00FF);//���У��
			Receive_TXData[37] = Sum;
		Receive_TXData[38] = 0xB5;
		Receive_TXData[39] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}	
	
//		Receive_TXData[78] = Send_CAMERA_Err; //�ߵ�ͨ��״̬	
		Send_SW_Data_Write();
}




//���ھ����͸������ܿ�����
void Send_Data_TX_SPM(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//֡ͷ
	Receive_TXData[2] = 35;
	Temp_32_Receive.Receive_Data_float = (FW_encoder_degrees);
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	Temp_32_Receive.Receive_Data_float = (FY_encoder_degrees );
//	Temp_32_Receive.Receive_Data_float = FY_location_set;
	Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	if(POS_SP_SWITCH == 1)
	{
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FW * 1000.0f);
		Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FY * 1000.0f);
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	}
	if(system_mode == 4)
	{
		Temp_Uchar = 1;
	}
	else if(system_mode == 2 )
	{
		Temp_Uchar = 0;
	}
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{
//		case :                          ////////////ָ��
//			cugenzongmoshi = 15;
//			break;
//		case :                           ////////////ɨ��
//			cugenzongmoshi = 16;
//			break;
//		case :                       ///////////////��������
//			cugenzongmoshi = 17;
//			break;
//		case :                       ///////////////����
//			cugenzongmoshi = 18;
//			break;
//		case :                 //////////////ʹ��
//			cugenzongmoshi = 19;
//			break;
//		case :                 //////////////ֹͣ
//			cugenzongmoshi = 20;
//			break;
//		case :                     /////////////�ջ�
//			cugenzongmoshi = 21;
//			break;
//		case :                  ////////////////Ŀ��������
//			cugenzongmoshi = 25;
//		  break;
//		case :             ///////////////////�ղ�
//			cugenzongmoshi = 26;
//			break;
//		case :          //////////////////չ�� 
//			cugenzongmoshi = 27;
//			break;
//		case :                     ///////////�ָ��������л�
//			cugenzongmoshi = 30;
//			break;
//		case :                  //////////////�־��ŷ�״̬��ѯ
//			cugenzongmoshi = 33;
//		  break;
		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{
//		case 15:                          ////////////ָ��
//			cugenzongzhiling = 15;
//			break;
//		case 16:                           ////////////ɨ��
//			cugenzongzhiling = 16;
//			break;
//		case 17:                       ///////////////��������
//			cugenzongzhiling = 17;
//			break;
//		case 18:                       ///////////////����
//			cugenzongzhiling = 18;
//			break;
//		case 19:                 //////////////ʹ��
//			cugenzongzhiling = 19;
//			break;
//		case 20:                 //////////////ֹͣ
//			cugenzongzhiling = 20;
//			break;
//		case 21:                     /////////////�ջ�
//			cugenzongzhiling = 21;
//			break;
//		case 25:                  ////////////////Ŀ��������
//			cugenzongzhiling = 25;
//		  break;
//		case 26:             ///////////////////�ղ�
//			cugenzongzhiling = 26;
//			break;
//		case 27:          //////////////////չ�� 
//			cugenzongzhiling = 27;
//			break;
//		case 30:                     ///////////�ָ��������л�
//			cugenzongzhiling = 30;
//			break;
//		case 33:                  //////////////�־��ŷ�״̬��ѯ
//			cugenzongzhiling = 33;
//		  break;
		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// �ߵ�///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
	Temp_32_Receive.Receive_Data_float = z_axis_velocity_float;		//��λ�ٶ�
	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
	Temp_32_Receive.Receive_Data_float = x_axis_velocity_float; 	// �����ٶ�
	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
	Temp_32_Receive.Receive_Data_float = y_axis_velocity_float;		//����ٶ�
	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//  Receive_TXData[31] = 0x00;
//  Receive_TXData[32] = 0x00;
//	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[31] = 0;
	//		//�ָ����ŷ��Ѱ���X
		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float*15.7;
//		FW_Jixia_tuoba_degrees = jiao1;
		Temp_32_Receive.Receive_Data_float = FW_Jixia_tuoba_degrees;
		Receive_TXData[32] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//�ָ����ŷ��Ѱ���Y
		FY_Jixia_tuoba_degrees = MissKy*miss_distance_Y_float*15.7;
//				FY_Jixia_tuoba_degrees = jiao3;
		Temp_32_Receive.Receive_Data_float = FY_Jixia_tuoba_degrees;
		Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[37] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[38] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[39] = Temp_32_Receive.Receive_Data_byte.High_byte;
	  Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
	  for(i = 2; i < (Receive_TX_Len - 2); i++)
			{
				Sum += Receive_TXData[i];
			}
		Receive_TXData[Receive_TX_Len - 2] = (Sum & 0x00FF);//���У��
		Receive_TXData[Receive_TX_Len - 1] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}
///////////////////////625//////////////////
//		Receive_TXData[36] = Send_gai_Status;			//�ָ���ģʽ
//		Receive_TXData[37] = Shangweiji_Command;  //�ָ���ָ��
//		Receive_TXData[38] = Send_Status;					//�ָ���ָ��ִ��״̬
//		Receive_TXData[39] = FY_Xianwei;					//������λ
//		Receive_TXData[40] = Send_Err1;						//���ŷ����ϴ���1
//		Receive_TXData[41] = Send_Err2;						//���ŷ����ϴ���2
//		
	
//		Receive_TXData[78] = Send_CAMERA_Err; //�ߵ�ͨ��״̬	
		Send_JiXiaData_Write();
}


//���ŷ����ջ����ܿذ�����ݣ����ŷ���������ܿذ�ͨ�ţ�����2
void Receive_JiShangData_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR1;
	for(Receive_i=0; Receive_i < Receive_RX_Len; Receive_i++)
	{
		Receive_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	Receive_DATA_Parse_2th();
}


//���ŷ����������ܿذ�����ݣ����ŷ���������ܿذ�ͨ�ţ�
void Send_JiXiaData_Write()
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR1;     //0800x
	for(Receive_i=0; Receive_i < Receive_TX_Len; Receive_i++)
	{
		*(uint32_t*)(Receive_ADDR_tmp)=Receive_TXData[Receive_i];
	}
	*(uint32_t*)(Receive_Com_En)= 1;
}

//���ŷ�����ջ���λ�������ݣ����ŷ�������λ��ͨ�ţ�
void Receive_ShangweijiData_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR3;
	for(Receive_i=0; Receive_i < Shangweiji_RX_Len; Receive_i++)
	{
		Shangweiji_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	Receive_Shangweiji_DATA_Parse();
}

//���ŷ�������λ��������   2022.8.16
void Send_SW_Data_Write()
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
//	Receive_ADDR_tmp = Receive_ADDR1;     //0800x
	for(Receive_i=0; Receive_i < 40; Receive_i++)
	{
		*(uint32_t*)(0x64000800)=Receive_TXData[Receive_i];
	}
	*(uint32_t*)(0x64000C00)= 1;
}
void Receive_Suidong_GS()
{
	
u8 i=0,Sum=0;	
if((Receive_DBJ_Data[0]==0xEB) & (Receive_DBJ_Data[1]==0x90))//֡ͷ
		{
			if(system_mode == 25)
	    {GS_Suidong_Flag=0;}
			if(system_mode == 27)
	    {GS_Suidong_Flag=1;}
				if((Receive_DBJ_Data[12]==0xFF)&(GS_Suidong_Flag==1))//֡β
				{
						Sum=0;
				for(i=0;i<(13-2);i++)
					 {
						Sum+=Receive_DBJ_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_DBJ_Data[13-2]==(Sum))//���У��
			
						{	
//								touluo_True ++;	//������ȷ
							  Suidong_jishu++;
								Receive_suidong_IP = Receive_DBJ_Data[10];
						}
//						else
//						{
//			     Suidong_jishu++;			
//						}
						
           if(Receive_suidong_IP == 14||Receive_suidong_IP == 25)/////////////////�ָ����ŷ�ָ��/////
							{
									  system_mode = 17;
										
										//�ָ����ŷ���λ
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[2];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[3];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[4];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[5];
										GS_Suidong_FY = Temp_32_Receive.Receive_Data_float; 
										Shangweiji_Direction_Angle_prepass = FsmLeadLag1(GS_Suidong_FY,FSM_X,0);
								
										FY_Degree_GS = -Shangweiji_Direction_Angle_prepass+GS_Suidong_offset_FY; 							//*360/2/3.14/1000/1000;//uradת��;
								    
										//�ָ����ŷ�����
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[6];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[7];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[8];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[9];
										GS_Suidong_FW = Temp_32_Receive.Receive_Data_float; 
							    	Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(GS_Suidong_FW,FSM_Y,0);
								
										FW_Degree_GS = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW; 							//*360/2/3.14/1000/1000;//uradת��;
								    GS_Suidong_chafen_FW=GS_Suidong_FW_before-Shangweiji_Pitch_Angle_Value_prepass;
								    GS_Suidong_chafen_FY=GS_Suidong_FY_before-Shangweiji_Direction_Angle_prepass;

								GS_Suidong_FW_before=Shangweiji_Pitch_Angle_Value_prepass;
								GS_Suidong_FY_before=Shangweiji_Direction_Angle_prepass;
							}
							
}

				}

			
			}

void Receive_Deadscale_Suidong_GS()
{
	
u8 i=0,Sum=0;	
if((Receive_DBJ_Data[0]==0xEB) & (Receive_DBJ_Data[1]==0x90))//֡ͷ
		{
//			if(system_mode == 25)
//	    {GS_Suidong_Flag=0;}
//			if(system_mode == 27)
//	    {GS_Suidong_Flag=1;}
				if((Receive_DBJ_Data[12]==0xFF))//֡β
				{
						Sum=0;
				for(i=0;i<(13-2);i++)
					 {
						Sum+=Receive_DBJ_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_DBJ_Data[13-2]==(Sum))//���У��
			
						{	
//								touluo_True ++;	//������ȷ
							  Suidong_jishu++;
								Receive_suidong_IP = Receive_DBJ_Data[10];
						}
//						else
//						{
//			     Suidong_jishu++;			
//						}
						
           if(Receive_suidong_IP == 14||Receive_suidong_IP == 25||Receive_suidong_IP == 17)/////////////////�ָ����ŷ�ָ��/////
							{
//									  system_mode = 17;
										
										//�ָ����ŷ���λ
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[2];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[3];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[4];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[5];
										GS_Suidong_FY = Temp_32_Receive.Receive_Data_float; 
										Shangweiji_Direction_Angle_prepass = FsmLeadLag1(GS_Suidong_FY,FSM_X,0);
								
										FY_Degree_GS_suidong = -Shangweiji_Direction_Angle_prepass+GS_Suidong_offset_FY; 							//*360/2/3.14/1000/1000;//uradת��;
								    
										//�ָ����ŷ�����
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[6];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[7];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[8];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[9];
										GS_Suidong_FW = Temp_32_Receive.Receive_Data_float; 
							    	Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(GS_Suidong_FW,FSM_Y,0);
                    FW_Degree_GS_suidong = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW;								
//										FW_Degree_GS_suidong = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW; 							//*360/2/3.14/1000/1000;//uradת��;
//								    GS_Suidong_chafen_FW=GS_Suidong_FW_before-Shangweiji_Pitch_Angle_Value_prepass;
//								    GS_Suidong_chafen_FY=GS_Suidong_FY_before-Shangweiji_Direction_Angle_prepass;

//								GS_Suidong_FW_before=Shangweiji_Pitch_Angle_Value_prepass;
//								GS_Suidong_FY_before=Shangweiji_Direction_Angle_prepass;
							}
							
}

				}
			}
void Receive_DBJsuidong_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = (u32)(0x64003000);
	for(Receive_i=0; Receive_i < Receive_Suidong_GS_Len; Receive_i++)
	{
		Receive_DBJ_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	if(system_mode == 25)
	{
	Receive_Deadscale_Suidong_GS();
	}
	else
	{
	Receive_Suidong_GS();
	}
}			