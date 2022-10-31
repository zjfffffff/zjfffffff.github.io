#include "tongxin.h"




void TX_Data_TX(void)
{
	u16 i=0,Sum=0;
	PC_TXData[0]=0xEB;//帧头
	PC_TXData[1]=0x90;
	PC_TXData[2]=PC_TX_Len-4;//帧长度
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
	
	//粗跟踪伺服方位
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
	
	//脱靶量
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
	PC_TXData[PC_TX_Len-1]=0xFE;//帧尾