#include "PC_Communication.h"
#include "stm32f4xx.h"
#include "Camera_Config.h"
#include "AHRS_100A.h"

//
extern float miss_offset_X;	//���õ�ƫ�÷�λ�Ѱ����ı���
extern float miss_offset_Y;	//���õ�ƫ�÷�λ�Ѱ����ı���
	


extern float FW_pointing_degree_LY;
extern float Point_FY_set_prepass;
extern u8 system_mode; 

typedef union
{
	struct
	{
		u8 Low_byte;
		u8 MLow_byte;
		u8 MHigh_byte;
		u8 High_byte;
	}Data_byte;
	float Data_float;
	u32 Data_u32;
	s32 Data_s32;
}Data_value;
Data_value Temp_32;

//������������Ѱ�������
float FW_tuoba_degrees = 0;
float FY_tuoba_degrees = 0;



u8    PC_gai_Status;					//�ָ���״̬��

u8  PC_TXData[PC_TX_Len],PC_Data[PC_RX_Len];
u8	PC_Err=0;		//�����־��

/***************************��λ��Э��������***************************/
u8 		SF_IP=1;						//�����ŷ�IP��
u8 		SF_Command;					//����ָ�� 0x55:���� 0xAA:��ѯ
u8 		SF_mode;					//�������� 1:ָ�� 2:���� 3:���� 4:ֹͣ   5:�������
float 	SF_Direction_Angle_Value;	//ָ��λλ��
float 	SF_Pitch_Angle_Value;		//ָ����λ��
u8 		INERTIAL_NAVI_IP=4;			//�ߵ�IP��
u8		LASER_IP=3;					//����̽����IP��
u8 		CAMERA_IP=2;				//�������IP��
u8 		CAMERA_Command;				//���ָ�� 0x55:���� 0xAA:��ѯ 0x66:����ָ��������� 0x67�����������
u8		CAMERA_Set_Flag=0;			//������ñ�־
u16 	CAMERA_Threshold_Value;		//�����ֵ
u8 		CAMERA_Algorithm;			//�����㷨 0:�����㷨 1:�����㷨
u8 		CAMERA_Windows_Switch;		//�����л� 0:���� 1:С���� 2:�󴰿�
u32 	CAMERA_Exposure_Time_Value;	//�ع�ʱ��
u16 	CAMERA_X_Length_Value;		//����X�᳤��
u16 	CAMERA_Y_Length_Value;		//����Y�᳤��
u16 	CAMERA_X_Migration_Value;	//����X��ƫ��
u16 	CAMERA_Y_Migration_Value;	//����Y��ƫ��
/************************************************************************/
/***************************����λ�����صı���***************************/
//��������ڵ���PC_Data_TX()����ǰ��Ҫȫ����ֵ��
//ע��PC_frame����Ҫ��ֵ��



float 	SF_Direction_Angle_AnswerValue;	//ָ��λλ��
float 	SF_Pitch_Angle_AnswerValue;		//ָ����λ��
float 	CAMERA_X_Target_Missing_Value;	//X���Ѱ���
float 	CAMERA_Y_Target_Missing_Value;	//Y���Ѱ���
u16		CAMERA_Frame_Rate;				//֡Ƶ
u8		CAMERA_Gray_Value;				//�Ҷ�ֵ
u8		CAMERA_Err;						//���ͨ��״̬ 0x00:ͨ������ 0x01:ͨ�Ź���
float 	CAMERA_Power;					//������
u8 		CAMERA_Status;					//ͼ����ٱ�־ 0x10:����Ŀ�� 0x20:Ŀ�궪ʧ
u8 		SF_Status;						//�ŷ�����״̬�� 0x00:δ���� 0x01:���� 0x02:Ŀ�궪ʧ
u8 		SF_mode_TX;						//�ŷ����صĹ���ģʽ 0x00:����(ֹͣ)ģʽ 0x01:����ģʽ 0x02:ָ��ģʽ 0x03:����ģʽ
u8 		SF_Err;							//�ŷ�ͨ��״̬ 0x00:ͨ������ 0x01:ͨ�Ź���
u8		PC_frame=0;						//֡����
float	INERTIAL_NAVI_YAWING;			//�ߵ�����
float	INERTIAL_NAVI_PITCHING;			//�ߵ�����
float	INERTIAL_NAVI_ROLLING;			//�ߵ����
/************************************************************************/

extern u16 Threshold;//��ֵ
extern u8 Windows_Switch;
extern u16 X_Length_Setting,Y_Length_Setting;
extern u32 Exposure_Time;//�ع�ʱ��
extern float X_Migration;
extern float Y_Migration;
extern float FW_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float miss_distance_X_float;
extern float miss_distance_Y_float;
extern AHRS_100A_Data Gyro;

void PC_DATA_Parse()
{
		u16 i=0,Sum=0;
		if((PC_Data[0]==0xEB) & (PC_Data[1]==0x90))//֡ͷ
		{
				if(PC_Data[PC_RX_Len-1]==0xFE)//֡β
				{
						for(i=2;i<(PC_RX_Len-2);i++)
						{
							Sum+=PC_Data[i];
						}
						if(PC_Data[PC_RX_Len-2]==(Sum&0x00ff))//���У��
						{
								SF_IP=PC_Data[3];///�ָ���
								SF_Command=PC_Data[4];//����ָ�� 0x55:���� 0xAA:��ѯ
								if(PC_Data[4] == 0x55)
								{
										SF_mode=PC_Data[5];	//�������� 1:ָ�� 4:���� 3:���� 4:ֹͣ   5:�������
										if(SF_mode == 1)    //ָ��
										{
												system_mode = 17;
										}
										if(SF_mode == 4)    //����
										{
												system_mode = 3;
										}
										if(SF_mode == 3)    //����
										{
												system_mode = 7;
										}
										if(SF_mode == 5)    //ֹͣ
										{
												system_mode = 1;
										}
				//					if(SF_mode == 5)    //�������
				//					{
				//							system_mode = 20;
				//					}
										Temp_32.Data_byte.High_byte=PC_Data[6];
										Temp_32.Data_byte.MHigh_byte=PC_Data[7];
										Temp_32.Data_byte.MLow_byte=PC_Data[8];
										Temp_32.Data_byte.Low_byte=PC_Data[9];
										SF_Direction_Angle_Value = Temp_32.Data_float;
										FW_pointing_degree_LY = SF_Direction_Angle_Value; //*360/2/3.14/1000/1000;//uradת��;

										Temp_32.Data_byte.High_byte=PC_Data[10];
										Temp_32.Data_byte.MHigh_byte=PC_Data[11];
										Temp_32.Data_byte.MLow_byte=PC_Data[12];
										Temp_32.Data_byte.Low_byte=PC_Data[13];

										SF_Pitch_Angle_Value=Temp_32.Data_float;
										Point_FY_set_prepass = SF_Pitch_Angle_Value;	// *360/2/3.14/1000/1000;;
										
										if(SF_mode==2)
										{
												miss_offset_X=SF_Direction_Angle_Value;  //�������õ�ƫ�÷�λ�Ѱ���
												miss_offset_Y=SF_Pitch_Angle_Value;      //�������õ�ƫ�ø����Ѱ���
										}
								}
								//////////// 
								CAMERA_IP=PC_Data[18];
								CAMERA_Command=PC_Data[19];
								if(CAMERA_Command==0x66)		
										Restore_Factory_Settings();		//����ָ���������
								if(CAMERA_Command==0x67)		
										Save_Configuration_Settings();	//����������
								if(CAMERA_Command==0x55)
								{
										CAMERA_Set_Flag=1;
										CAMERA_Threshold_Value=((u16)PC_Data[20])<<8 | PC_Data[21];
										Threshold =CAMERA_Threshold_Value;
										switch(PC_Data[22]&0x01)
										{
												case 0x01:
												{
													CAMERA_Algorithm=Barycentre;
													break;
												}
												case 0x00:
												{
													CAMERA_Algorithm=Centroid;
													break;
												}
										}
										switch(PC_Data[22]&0x0C)
										{
												case 0x04:
												{
													CAMERA_Windows_Switch=Window_S;
													Windows_Switch = 1;
													break;
												}
												case 0x08:
												{
													CAMERA_Windows_Switch=Window_L;
													Windows_Switch = 0;
													break;
												}
										}			
										Temp_32.Data_byte.High_byte=PC_Data[23];
										Temp_32.Data_byte.MHigh_byte=PC_Data[24];
										Temp_32.Data_byte.MLow_byte=PC_Data[25];
										Temp_32.Data_byte.Low_byte=PC_Data[26];
										CAMERA_Exposure_Time_Value=Temp_32.Data_u32;
										Exposure_Time = CAMERA_Exposure_Time_Value;
										
										CAMERA_X_Length_Value=((u16)PC_Data[27])<<8 | PC_Data[28];
										CAMERA_Y_Length_Value=((u16)PC_Data[29])<<8 | PC_Data[30];
										X_Length_Setting = CAMERA_X_Length_Value;
										Y_Length_Setting = CAMERA_Y_Length_Value;
										
										CAMERA_X_Migration_Value=((u16)PC_Data[31])<<8 | PC_Data[32];
										CAMERA_Y_Migration_Value=((u16)PC_Data[33])<<8 | PC_Data[34];
										X_Migration = CAMERA_X_Migration_Value;
										Y_Migration = CAMERA_Y_Migration_Value;
								}			
								LASER_IP=PC_Data[37];
						}
						else
						{
								PC_Err=1;
						}
				}
				else
				{
						PC_Err=2;
				}	
		}
		else
		{
				PC_Err=3;
		}
}
void PC_Data_TX(void)
{
		u16 i=0,Sum=0;
		PC_TXData[0]=0xEB;//֡ͷ
		PC_TXData[1]=0x90;
		PC_TXData[2]=PC_TX_Len-4;//֡����
		PC_TXData[3]=SF_IP;//01
		PC_TXData[4]=SF_Command;
		if(system_mode==1)
		{
				PC_gai_Status=0x01;
		}
		if(system_mode==3)
		{
				PC_gai_Status=0x02;
		}
		if(system_mode==17)
		{
				PC_gai_Status=0x03;
		}
		if(system_mode==7)
		{
				PC_gai_Status=0x04;
		}
		PC_TXData[5]=PC_gai_Status;
		
		//�ָ����ŷ���λ
		SF_Direction_Angle_AnswerValue = FW_encoder_degrees;
		Temp_32.Data_float=SF_Direction_Angle_AnswerValue;
		PC_TXData[6]=Temp_32.Data_byte.High_byte;
		PC_TXData[7]=Temp_32.Data_byte.MHigh_byte;
		PC_TXData[8]=Temp_32.Data_byte.MLow_byte;
		PC_TXData[9]=Temp_32.Data_byte.Low_byte;
		
		SF_Pitch_Angle_AnswerValue = FY_encoder_degrees;
		Temp_32.Data_float=SF_Pitch_Angle_AnswerValue;
		PC_TXData[10]=Temp_32.Data_byte.High_byte;
		PC_TXData[11]=Temp_32.Data_byte.MHigh_byte;
		PC_TXData[12]=Temp_32.Data_byte.MLow_byte;
		PC_TXData[13]=Temp_32.Data_byte.Low_byte;
		
		//�Ѱ���
		FW_tuoba_degrees=miss_distance_X_float;
		Temp_32.Data_float=FW_tuoba_degrees;
		PC_TXData[24]=Temp_32.Data_byte.High_byte;
		PC_TXData[25]=Temp_32.Data_byte.MHigh_byte;
		PC_TXData[26]=Temp_32.Data_byte.MLow_byte;
		PC_TXData[27]=Temp_32.Data_byte.Low_byte;
		
		FY_tuoba_degrees=miss_distance_Y_float;
		Temp_32.Data_float=FY_tuoba_degrees;
		PC_TXData[28]=Temp_32.Data_byte.High_byte;
		PC_TXData[29]=Temp_32.Data_byte.MHigh_byte;
		PC_TXData[30]=Temp_32.Data_byte.MLow_byte;
		PC_TXData[31]=Temp_32.Data_byte.Low_byte;
		
		for(i=2;i<(PC_TX_Len-2);i++)
		{
			Sum+=PC_TXData[i];
		}
		PC_TXData[32]=Sum;
		PC_TXData[PC_TX_Len-1]=0xFE;//֡β
		
	
	//	PC_TXData[13]=SF_Status=3;
	//	PC_TXData[14]=SF_mode;
	//	
	//	PC_TXData[15]=SF_Err;
	//	
	//	PC_TXData[16]=CAMERA_IP;
	//	PC_TXData[17]=CAMERA_Command;
	//	PC_TXData[18]=CAMERA_Status;
	//	
	//	CAMERA_X_Target_Missing_Value = miss_distance_X_float;
	//	Temp_32.Data_float=CAMERA_X_Target_Missing_Value;
	//	PC_TXData[19]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[20]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[21]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[22]=Temp_32.Data_byte.Low_byte;
	//	
	//	CAMERA_Y_Target_Missing_Value = miss_distance_Y_float;
	//	Temp_32.Data_float=CAMERA_Y_Target_Missing_Value;
	//	PC_TXData[23]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[24]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[25]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[26]=Temp_32.Data_byte.Low_byte;

	//	PC_TXData[27]=CAMERA_Frame_Rate>>8;
	//	PC_TXData[28]=CAMERA_Frame_Rate;
	//	
	//	PC_TXData[31]=CAMERA_Gray_Value;
	//	
	//	PC_TXData[32]=CAMERA_Err=0;
	//	
	//	PC_TXData[33]=LASER_IP;
	//	Temp_32.Data_float=CAMERA_Power;
	//	PC_TXData[35]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[36]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[37]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[38]=Temp_32.Data_byte.Low_byte;
	//	
	//	PC_TXData[43]=INERTIAL_NAVI_IP;
	//	PC_TXData[44]=0xAA;
	//	
	//	INERTIAL_NAVI_YAWING = Gyro.Yaw;
	//	Temp_32.Data_float=INERTIAL_NAVI_YAWING;
	//	PC_TXData[45]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[46]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[47]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[48]=Temp_32.Data_byte.Low_byte;
	//	
	//	INERTIAL_NAVI_PITCHING = Gyro.Pitch;
	//	Temp_32.Data_float=INERTIAL_NAVI_PITCHING;
	//	PC_TXData[49]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[50]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[51]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[52]=Temp_32.Data_byte.Low_byte;
	//	
	//	INERTIAL_NAVI_ROLLING = Gyro.Roll;
	//	Temp_32.Data_float=INERTIAL_NAVI_ROLLING;
	//	PC_TXData[53]=Temp_32.Data_byte.High_byte;
	//	PC_TXData[54]=Temp_32.Data_byte.MHigh_byte;
	//	PC_TXData[55]=Temp_32.Data_byte.MLow_byte;
	//	PC_TXData[56]=Temp_32.Data_byte.Low_byte;
	//	
	//	
	//	PC_TXData[57]=PC_frame;//֡����
	//	
	//	for(i=2;i<(PC_TX_Len-3);i++)
	//	{
	//		Sum+=PC_TXData[i];
	//	}
	//	PC_TXData[PC_TX_Len-3]=Sum>>8;//���У��
	//	PC_TXData[PC_TX_Len-2]=Sum;//���У��
	//	PC_TXData[PC_TX_Len-1]=0xFE;//֡β
	//	PC_frame++;
		PC_Com_Write();
}

void PC_Com_Read()	
{
	u8 PC_i=0;
	u32 PC_ADDR_tmp;
	PC_ADDR_tmp=PC_ADDR;
	for(PC_i=0; PC_i<PC_RX_Len; PC_i++)
	{
		PC_Data[PC_i]=*(uint32_t*)(PC_ADDR_tmp);
		PC_ADDR_tmp = PC_ADDR_tmp+2;
	}
	PC_DATA_Parse();
}

void PC_Com_Write()
{
	u8 PC_i=0;
	u32 PC_ADDR_tmp;
	for(PC_i=0; PC_i<PC_TX_Len; PC_i++)
	{
		*(uint32_t*)(PC_ADDR)= PC_TXData[PC_i];
	}
	*(uint32_t*)(PC_Com_En)= 1;
}
