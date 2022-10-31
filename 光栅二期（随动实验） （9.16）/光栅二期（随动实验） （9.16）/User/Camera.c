/**
  ******************************************************************************
  * @file    Camera.c
  * @author  wjy
  * @version V1.0.0
  * @date    03-December-2018
  * @brief   ���������
  *           
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
	
/* Includes ����ͷ�ļ�----------------------------------------------------------------------------------*/
#include "Camera.h"
/* Private typedef �Զ���ͬ��ؼ���---------------------------------------------------------------------*/
/* Private define  �Զ��������-------------------------------------------------------------------------*/ 
/* Private macro   �Զ��庯����-------------------------------------------------------------------------*/
/* Private variables �Զ������-------------------------------------------------------------------------*/
u32 Exposure_Time=0;//�ع�ʱ��
u16 X_Migration_Setting=0;//����Xƫ��
u16 Y_Migration_Setting=0;//����Yƫ��
u16 X_Length_Setting=0;//����X����
u16 Y_Length_Setting=0;//����Y����
u8 Windows_Switch=0;//�л�����
u16 Threshold=0;//��ֵ
u8 Tracing_Algorithm=0;//�����㷨
/////////////////////////
u16 X_Migration_Setting1=0;//����Xƫ��
u32 Exposure_Time1=0;//�ع�ʱ��
u16 X_Length_Setting1=0;//����X����
u16 Y_Length_Setting1=0;//����Y����
u16 Y_Migration_Setting1=0;//����Yƫ��
u16 Threshold1=0;//��ֵ
u8 Tracing_Algorithm1=0;//�����㷨
u8 Windows_Switch1=0;//�л�����





///////////////////////
u16 Image_Threshold=0;//����ͼ����ֵ
u8 Image_Gray_Value=0;//ͼ��ƽ���Ҷ�ֵ
u8 Background_Gray_Value=0;//����ƽ���Ҷ�ֵ
u8 Spot_Gray_Value=0;//���ƽ���Ҷ�ֵ
u8 Camera_Window=0;//�������״̬
u8 Coarse_Image_Symbol=0;//ͼ��ָ��ٱ�־
u8 Precise_Image_Symbol=0;//ͼ�񾫸��ٱ�־
u8 Image_Algorithm=0;//ͼ���㷨
u32 Integral_Time=0;//����ʱ��
u32 Integral_Time_High=0;
u32 Integral_Time_Low=0;

u16 X_Migration_Now=0;//Xƫ��
u16 Y_Migration_Now=0;//Yƫ��
u16 X_Length_Now=0;//X����
u16 Y_Length_Now=0;//Y����
u8 windows_OK=0;

u8 Camera_Error=0,Camera_ADD_Error=0;//�������
//////////////////////
u16 Image_Threshold1=0;//����ͼ����ֵ
u8 Image_Gray_Value1=0;//ͼ��ƽ���Ҷ�ֵ
u8 Background_Gray_Value1=0;//����ƽ���Ҷ�ֵ
u8 Spot_Gray_Value1=0;//���ƽ���Ҷ�ֵ
u8 Camera_Window1=0;//�������״̬
u8 Coarse_Image_Symbol1=0;//ͼ��ָ��ٱ�־
u8 Precise_Image_Symbol1=0;//ͼ�񾫸��ٱ�־
u8 Image_Algorithm1=0;//ͼ���㷨
u32 Integral_Time1=0;//����ʱ��
u32 Integral_Time_High1=0;
u32 Integral_Time_Low1=0;

u16 X_Migration_Now1=0;//Xƫ��
u16 Y_Migration_Now1=0;//Yƫ��
u16 X_Length_Now1=0;//X����
u16 Y_Length_Now1=0;//Y����
u8 windows_OK1=0;

u8 Camera_Error1=0,Camera_ADD_Error1=0;//�������

////////////////////


u16 wait_cam_tim=50000;
u32 wait_cam_tim1=50000;
/* Private function prototypes �Զ��庯������-----------------------------------------------------------*/
/* Private functions           �Զ��庯�� --------------------------------------------------------------*/
//////////////////////////////////////////
////////////////
/**
  * @brief	�������������
	* @data		
  */
void Camera_Param_Set1(u16 Param_Addr,u16 Param_Data,u8 Data[])
{
	u8 Check=0;
	Data[22]=0x55;
	Data[23]=0x7E;//֡ͷ
	/*֡����*/
	Data[24]=0x30;
	Data[25]=0x45;
	/*�豸��ַ*/
	Data[26]=0x36;
	Data[27]=0x30;
	/*�豸������ַ*/ 
	Data[31]=Char_To_Hex((u8)(Param_Addr&0x000F));
	Data[30]=Char_To_Hex((u8)((Param_Addr&0x00F0)>>4));
	Data[29]=Char_To_Hex((u8)((Param_Addr&0x0F00)>>8));
	Data[28]=Char_To_Hex((u8)((Param_Addr&0xF000)>>12));
	/*���ò�������*/
	Data[35]=Char_To_Hex((u8)(Param_Data&0x000F));
	Data[34]=Char_To_Hex((u8)((Param_Data&0x00F0)>>4));
	Data[33]=Char_To_Hex((u8)((Param_Data&0x0F00)>>8));
	Data[32]=Char_To_Hex((u8)((Param_Data&0xF000)>>12));
#ifdef CAMERA_SETTING_CHECK
	/*У�� = �豸��ַ+���ò�����ַ+���ò������� = 60H + 00H + 00H +00H + 00H = 60H*/
	Check=0x60+	\
				((u8)(Param_Addr&0x00FF))+((u8)((Param_Addr&0xFF00)>>8))+	\
				((u8)(Param_Data&0x00FF))+((u8)((Param_Data&0xFF00)>>8));	
	Data[36]=Char_To_Hex((u8)((Check&0x00F0)>>4));
	Data[37]=Char_To_Hex((u8)(Check&0x000F));
#endif
	Data[38]=0x3E;//֡β
}

/**
  * @brief	���������
	* @data		
  */
//�ָ���������
void Restore_Factory_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	Camera_Param_Set1(Reset_Addr,0, Data);
	//Comm1_Data_Pack();//���ŷ�
	//Comm_Write(ADDR,Data,n,Enable_ADDR);
}
//���浱ǰ����
void Save_Configuration_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	u32 i=0;
	Camera_Param_Set1(Save_Configuration_Addr,0, Data);
	//Comm1_Data_Pack();//���ŷ�
	//Comm_Write(ADDR,Data,n,Enable_ADDR);
//	while(Comm4Flag==0)
//		{
//			i++;
//			Check_Comm_Status();
//			if(i>wait_cam_tim)
//			{
//				i=0;Camera_Error=1;break;
//			}
//		}
}
/**
  * @brief	�������
	* @data		
  */
u8 Camera_Step,Camera_TX_falg;
void Camera_Set_chu(u32 ADDR,u8 Data[])
{
	//u32 i=0;
	Camera_Step++;
	Temp_16_32.Data_u32=Exposure_Time1;//�ع�ʱ��
	switch(Camera_Step)
	{
		case 1:
		{
					
		Camera_Param_Set1(Exposure_Time_Addr2,Temp_16_32.Data_byte.High_byte,Data);//�ع�ʱ��
		//Comm_Write(ADDR,Data,n,Enable_ADDR);
	//Delay_us(30);	
			break;
		}
		case 2:
		{
					
		Camera_Param_Set1(Exposure_Time_Addr1,Temp_16_32.Data_byte.Low_byte,Data);//X����λ��
		//Comm_Write(ADDR,Data,n,Enable_ADDR);
	//Delay_us(30);	
			break;
		}
//		/
//		case 2:
//		{
//					
//		Camera_Param_Set(0X1271,X_Center_Position,Data);//Y����λ��
//		Comm_Write(ADDR,Data,n,Enable_ADDR);
//	//Delay_us(30);	
//			break;
//		}

		case 3:
		{
		Camera_Param_Set1(Threshold_Addr,Threshold1,Data);//��ֵ
		//Comm_Write(ADDR,Data,n,Enable_ADDR);
			break;
		//Delay_us(30);
		}

      case 4:
		   {
				Camera_Param_Set1(X_Migration_Addr,X_Migration_Setting1,Data);//Xƫ��
				//Comm_Write(ADDR,Data,n,Enable_ADDR);
				//Delay_us(30);
				 break;
			 }

			
		case 5:
		{
				Camera_Param_Set1(Y_Migration_Addr,Y_Migration_Setting1,Data);//Yƫ��
				//Comm_Write(ADDR,Data,n,Enable_ADDR);
			break;
		}

			case 6:
		{
				Camera_Param_Set1(X_Length,X_Length_Setting1,Data);//X����
				//Comm_Write(ADDR,Data,n,Enable_ADDR);
			break;
		}
				//X_Length_Now=X_Length_Setting;

			case 7:
		   {
				Camera_Param_Set1(Y_Length,Y_Length_Setting1,Data);//Y����
				//Comm_Write(ADDR,Data,n,Enable_ADDR);
				 break;
		   }

			
			case 8:
		   {
		     Camera_Param_Set1(Small_Window_Switch,Windows_Switch1,Data);//�����л�
		     //Comm_Write(ADDR,Data,n,Enable_ADDR);
				 break;
			 }

	
    case 9:
		{
		   Camera_Param_Set1(Tracking_Algorithm_Addr,Tracing_Algorithm1,Data);//�����㷨
		   //Comm_Write(ADDR,Data,n,Enable_ADDR);
			Camera_TX_falg=0;
			Camera_Step=0;
			break;
		//Delay_us(30);
		}
/*	
  		case 1:
		{
					
		Camera_Param_Set(0X1270,X_Center_Position,Data);//X����λ��
		Comm_Write(ADDR,Data,n,Enable_ADDR);
	//Delay_us(30);	
			break;
		}
		case 2:
		{
					
		Camera_Param_Set(0X1271,X_Center_Position,Data);//Y����λ��
		Comm_Write(ADDR,Data,n,Enable_ADDR);
	//Delay_us(30);	
			break;
		}
		case 10:
		{
		   Camera_Param_Set(0x1310,Image_Output_Enable,Data);//ͼ�����ʹ��
		   Comm_Write(ADDR,Data,n,Enable_ADDR);
//			Camera_TX_falg=0;
//			Camera_Step=0;
			break;
		//Delay_us(30);
		}
		case 11:
		{
		   Camera_Param_Set(0x1218,ADC_Gain_Setting,Data);//ADC����
		   Comm_Write(ADDR,Data,n,Enable_ADDR);
			
			break;
		//Delay_us(30);
		}
		case 12:
		{
		   Camera_Param_Set(0x1220,PGA_Gain_Setting,Data);//PAG����
		   Comm_Write(ADDR,Data,n,Enable_ADDR);
			
			break;
		//Delay_us(30);
		}
		case 13:
		{
		   Camera_Param_Set(0x1308,Sun_Pixel,Data);//̫����С
		   Comm_Write(ADDR,Data,n,Enable_ADDR);
			Camera_TX_falg=0;
			Camera_Step=0;
			break;
		//Delay_us(30);
		}*/

	}

}
///////////////
void Camera_Set1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	u32 i=0;
	Temp_16_32.Data_u32=Exposure_Time1;//�ع�ʱ��
	if(Temp_16_32.Data_byte.High_byte!=Integral_Time_High1)
	{
		Camera_Param_Set1(Exposure_Time_Addr2,Temp_16_32.Data_byte.High_byte,Data);//��λ
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		
		while(Comm3Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
		if(Comm4Flag==1)//�����Ӧ��
		{
			Camera_Answer1(Comm3_Addr,Comm1_RXD);
			Comm4Flag=0;
		}
	}
	if(Temp_16_32.Data_byte.Low_byte!=Integral_Time_Low1)
	{
		Camera_Param_Set1(Exposure_Time_Addr1,Temp_16_32.Data_byte.Low_byte,Data);//��λ
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm4Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
		if(Comm4Flag==1)//�����Ӧ��
		{
			Camera_Answer1(Comm3_Addr,Comm1_RXD);
			Comm4Flag=0;
		}
	}		
	if(Threshold1!=Image_Threshold1)
	{
		Camera_Param_Set1(Threshold_Addr,Threshold1,Data);//��ֵ
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm4Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
		if(Comm4Flag==1)//�����Ӧ��
		{
			Camera_Answer1(Comm3_Addr,Comm1_RXD);
			Comm4Flag=0;
		}
	}
	if((Windows_Switch1!=Camera_Window1)||(X_Migration_Setting1!=X_Migration_Now1)||(Y_Migration_Setting1!=Y_Migration_Now1)||(X_Length_Setting1!=X_Length_Now1)||(Y_Length_Setting1!=Y_Length_Now1))//����
	{
		if(Windows_Switch1==Window_S)//С����
		{
			if(X_Migration_Setting1!=X_Migration_Now1)
			{
				Camera_Param_Set1(X_Migration_Addr,X_Migration_Setting1,Data);//Xƫ��
				Comm1_Data_Pack();
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				X_Migration_Now1=X_Migration_Setting1;
				while(Comm4Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim1)
					{
						i=0;Camera_Error1=1;break;
					}
				}
				if(Comm4Flag==1)//���Ӧ��
				{
					Camera_Answer1(Comm3_Addr,Comm1_RXD);
					Comm4Flag=0;
				}
			}
		 delay_us(100);
			if(Y_Migration_Setting1!=Y_Migration_Now1)
			{
				Camera_Param_Set1(Y_Migration_Addr,Y_Migration_Setting1,Data);//Yƫ��
				Comm1_Data_Pack();
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				Y_Migration_Now1=Y_Migration_Setting1;
				while(Comm4Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim1)
					{
						i=0;Camera_Error1=1;break;
					}
				}
				if(Comm4Flag==1)//���Ӧ��
				{
					Camera_Answer1(Comm3_Addr,Comm1_RXD);
					Comm4Flag=0;
				}
			}
			if(X_Length_Setting1!=X_Length_Now1)
			{
				if(X_Length_Setting1==0)
				{
					X_Length_Setting1=1;//����Ϊ��
				}
				Camera_Param_Set1(X_Length,X_Length_Setting1,Data);//X����
				Comm1_Data_Pack();
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				X_Length_Now1=X_Length_Setting1;
				while(Comm4Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim1)
					{
						i=0;Camera_Error1=1;break;
					}
				}
				if(Comm4Flag==1)//�����Ӧ��
				{
					Camera_Answer1(Comm3_Addr,Comm1_RXD);
					Comm4Flag=0;
				}
			}
				delay_us(1000);
			if(Y_Length_Setting1!=Y_Length_Now1)
			{
				if(Y_Length_Setting1==0)
				{
					Y_Length_Setting1=1;//����Ϊ��
				}
				Camera_Param_Set1(Y_Length,Y_Length_Setting1,Data);//Y����
				Comm1_Data_Pack();
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				Y_Length_Now1=Y_Length_Setting1;
				while(Comm4Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim)
					{
						i=0;Camera_Error1=1;break;
					}
				}
				if(Comm4Flag==1)//���Ӧ��
				{
					Camera_Answer1(Comm3_Addr,Comm1_RXD);
					Comm4Flag=0;
				}
			}
		}	
		Camera_Param_Set1(Small_Window_Switch,Windows_Switch1,Data);//�����л�
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm4Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
		if(Comm4Flag==1)//�����Ӧ��
		{
			Camera_Answer1(Comm3_Addr,Comm1_RXD);
			Comm4Flag=0;
		}
	}
	if(Tracing_Algorithm1!=Image_Algorithm1)
	{
		Camera_Param_Set1(Tracking_Algorithm_Addr,Tracing_Algorithm1,Data);//�����㷨
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm4Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
		if(Comm4Flag==1)//�����Ӧ��
		{
			Camera_Answer1(Comm3_Addr,Comm1_RXD);
			Comm4Flag=0;
		}
	}
	if(	(Exposure_Time1==Integral_Time1)&&(Threshold1==Image_Threshold1)&&(Windows_Switch1==Camera_Window1)&&(Tracing_Algorithm1==Image_Algorithm1)	)
	{
		Camera_Error1=0;
	}
	
	Camera_Param_Set1(Small_Window_Switch,Windows_Switch1,Data);//�����л�
		Comm1_Data_Pack();
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm4Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim1)
			{
				i=0;Camera_Error1=1;break;
			}
		}
}
////////////////////////////////
/**
  * @brief	�����Ӧ��
	* @data		
  */
u16 Camera_Param_Addr1=0,Camera_Param_Data1=0;
void Camera_Answer1(u32 ADDR,u8 Data[])
{
	u8 i;
	u8 Check=0,C1=0,C2=0;
	u16 Temp_Addr=0,Temp_Data=0;
//	u16 Camera_Param_Addr=0,Camera_Param_Data=0;
	u32 readADDRTemp=0;
	readADDRTemp=ADDR;	
	for(i=0;i<111;i++)
	{
		Data[i]=*(uint32_t*)(readADDRTemp);/////////////////////////////////////
		readADDRTemp+=2;
	}
	Comm1_Data_Unpack();
	if(Data[92]==0x7C)//֡ͷ
	{
		if(Data[107]==0x3C)//֡β
		{		
			Temp_Addr=Hex_To_Char(Data[97]);	Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[98]);	Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[99]);	Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[100]);
			
			Temp_Data=Hex_To_Char(Data[101]);		Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[102]);	Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[103]);	Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[104]);
#ifdef CAMERA_SETTING_CHECK
			/*У�� = �豸��ַ+���ò�����ַ+���ò������� = 60H + 00H + 00H +00H + 00H = 60H*/
			Check=0x60+	\
						((u8)(Temp_Addr&0x00FF))+((u8)((Temp_Addr&0xFF00)>>8))+	\
						((u8)(Temp_Data&0x00FF))+((u8)((Temp_Data&0xFF00)>>8));	
			C1=Hex_To_Char(Check&0xF0);
			C2=Hex_To_Char(Check&0x0F);
			if((Data[105]==0x7C)&(Data[106]==0x7C))//У��
			{
#endif
				Camera_Param_Addr1=Temp_Addr;
				Camera_Param_Data1=Temp_Data;
#ifdef CAMERA_SETTING_CHECK
			}
			else Camera_ADD_Error1=1;
#endif
			Camera_Answer_Param1(Camera_Param_Addr1,Camera_Param_Data1);
		}
		else Camera_ADD_Error1=2;
	}
	else Camera_ADD_Error1=3;
}
/**
  * @brief	�����Ӧ�����
	* @data		
  */
void Camera_Answer_Param1(u16 Param_Addr,u16 Param_Data)
{
	u32 Temp=0;
	switch(Param_Addr)
	{
		case Exposure_Time_Addr2:
		{
			Integral_Time_High1=Param_Data;	
			break;
		}
		case Exposure_Time_Addr1:
		{
			Integral_Time_Low1=Param_Data;
			break;
		}
		case Tracking_Algorithm_Addr:
		{
			Image_Algorithm1=Param_Data;
			break;
		}	
		case Small_Window_Switch:
		{
			Camera_Window1=Param_Data;
			break;
		}
		case Threshold_Addr:
		{	
			Image_Threshold1=Param_Data;
			break;
		}
	}
	Temp=Integral_Time_High1;	Temp<<=16;
	Temp+=Integral_Time_Low1;
	Integral_Time1=Temp;
}




///////////////////////////////////////////
////////////////////////////
/**
  * @brief	�����������
	* @data		
  */
void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[])
{
	u8 Check=0;
	Data[0]=0x7E;//֡ͷ
	/*֡����*/
	Data[1]=0x30;
	Data[2]=0x45;
	/*�豸��ַ*/
	Data[3]=0x36;
	Data[4]=0x30;
	/*�豸������ַ*/ 
	Data[8]=Char_To_Hex((u8)(Param_Addr&0x000F));
	Data[7]=Char_To_Hex((u8)((Param_Addr&0x00F0)>>4));
	Data[6]=Char_To_Hex((u8)((Param_Addr&0x0F00)>>8));
	Data[5]=Char_To_Hex((u8)((Param_Addr&0xF000)>>12));
	/*���ò�������*/
	Data[12]=Char_To_Hex((u8)(Param_Data&0x000F));
	Data[11]=Char_To_Hex((u8)((Param_Data&0x00F0)>>4));
	Data[10]=Char_To_Hex((u8)((Param_Data&0x0F00)>>8));
	Data[9]=Char_To_Hex((u8)((Param_Data&0xF000)>>12));
#ifdef CAMERA_SETTING_CHECK
	/*У�� = �豸��ַ+���ò�����ַ+���ò������� = 60H + 00H + 00H +00H + 00H = 60H*/
	Check=0x60+	\
				((u8)(Param_Addr&0x00FF))+((u8)((Param_Addr&0xFF00)>>8))+	\
				((u8)(Param_Data&0x00FF))+((u8)((Param_Data&0xFF00)>>8));	
	Data[13]=Char_To_Hex((u8)((Check&0x00F0)>>4));
	Data[14]=Char_To_Hex((u8)(Check&0x000F));
#endif
	Data[15]=0x3E;//֡β
}
//////////////////�ָ���������
void Restore_Factory_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	u32 i=0;
	Camera_Param_Set(Reset_Addr,0, Data);
	Comm_Write(ADDR,Data,n,Enable_ADDR);
	while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
}

//���浱ǰ����
void Save_Configuration_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	u32 i=0;
	Camera_Param_Set(Save_Configuration_Addr,0, Data);
	Comm_Write(ADDR,Data,n,Enable_ADDR);
	while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
}
/**
  * @brief	�������
	* @data		
  */
void Camera_Set(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR)
{
	u32 i=0;
	Temp_16_32.Data_u32=Exposure_Time;//�ع�ʱ��
	//*
	if(Temp_16_32.Data_byte.High_byte!=Integral_Time_High)
	{
		Camera_Param_Set(Exposure_Time_Addr2,Temp_16_32.Data_byte.High_byte,Data);//��λ
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		
		while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
		if(Comm1Flag==1)//���Ӧ��
		{
			Camera_Answer(Comm0_Addr,Comm4_RXD);
			Comm1Flag=0;
		}
	}//*/
	//*
	if(Temp_16_32.Data_byte.Low_byte!=Integral_Time_Low)
	{
		Camera_Param_Set(Exposure_Time_Addr1,Temp_16_32.Data_byte.Low_byte,Data);//��λ
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
		if(Comm1Flag==1)//���Ӧ��
		{
			Camera_Answer(Comm0_Addr,Comm4_RXD);
			Comm1Flag=0;
		}
	}	
//*/	
///*
	if(Threshold!=Image_Threshold)
	{
		Camera_Param_Set(Threshold_Addr,Threshold,Data);//��ֵ
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
		if(Comm1Flag==1)//���Ӧ��
		{
			Camera_Answer(Comm0_Addr,Comm4_RXD);
			Comm1Flag=0;
		}
	}
	//*/
//	
	if((Windows_Switch!=Camera_Window)||(X_Migration_Setting!=X_Migration_Now)||(Y_Migration_Setting!=Y_Migration_Now)||(X_Length_Setting!=X_Length_Now)||(Y_Length_Setting!=Y_Length_Now))//����
	{
		if(Windows_Switch==Window_S)//С����
		{
			if(X_Migration_Setting!=X_Migration_Now)
			{
				Camera_Param_Set(X_Migration_Addr,X_Migration_Setting,Data);//Xƫ��
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				X_Migration_Now=X_Migration_Setting;
				while(Comm1Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim)
					{
						i=0;Camera_Error=1;break;
					}
				}
				if(Comm1Flag==1)//���Ӧ��
				{
					Camera_Answer(Comm0_Addr,Comm4_RXD);
					Comm1Flag=0;
				}
			}
			
			if(Y_Migration_Setting!=Y_Migration_Now)
			{
				Camera_Param_Set(Y_Migration_Addr,Y_Migration_Setting,Data);//Yƫ��
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				Y_Migration_Now=Y_Migration_Setting;
				while(Comm1Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim)
					{
						i=0;Camera_Error=1;break;
					}
				}
				if(Comm1Flag==1)//���Ӧ��
				{
					Camera_Answer(Comm0_Addr,Comm4_RXD);
					Comm0Flag=0;
				}
			}
			delay_us(10);
			if(X_Length_Setting!=X_Length_Now)
			{
				if(X_Length_Setting==0)
				{
					X_Length_Setting=1;//����Ϊ��
				}
				Camera_Param_Set(X_Length,X_Length_Setting,Data);//X����
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				X_Length_Now=X_Length_Setting;
				while(Comm1Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim)
					{
						i=0;Camera_Error=1;break;
					}
				}
				if(Comm1Flag==1)//���Ӧ��
				{
					Camera_Answer(Comm0_Addr,Comm4_RXD);
					Comm1Flag=0;
				}
			}
			delay_us(1000);
			if(Y_Length_Setting!=Y_Length_Now)
			{
				if(Y_Length_Setting==0)
				{
					Y_Length_Setting=1;//����Ϊ��
				}
				Camera_Param_Set(Y_Length,Y_Length_Setting,Data);//Y����
				Comm_Write(ADDR,Data,n,Enable_ADDR);
				//Y_Length_Now=Y_Length_Setting;
				while(Comm1Flag==0)
				{
					i++;
					Check_Comm_Status();
					if(i>wait_cam_tim)
					{
						i=0;Camera_Error=1;break;
					}
				}
				if(Comm1Flag==1)//���Ӧ��
				{
					Camera_Answer(Comm0_Addr,Comm4_RXD);
					Comm1Flag=0;
				}
			}
		}	
		Camera_Param_Set(Small_Window_Switch,Windows_Switch,Data);//�����л�
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
		if(Comm1Flag==1)//���Ӧ��
		{
			Camera_Answer(Comm0_Addr,Comm4_RXD);
			Comm1Flag=0;
		}
	}
	if(Tracing_Algorithm!=Image_Algorithm)
	{
		Camera_Param_Set(Tracking_Algorithm_Addr,Tracing_Algorithm,Data);//�����㷨
		Comm_Write(ADDR,Data,n,Enable_ADDR);
		while(Comm1Flag==0)
		{
			i++;
			Check_Comm_Status();
			if(i>wait_cam_tim)
			{
				i=0;Camera_Error=1;break;
			}
		}
		if(Comm1Flag==1)//���Ӧ��
		{
			Camera_Answer(Comm0_Addr,Comm4_RXD);
			Comm1Flag=0;
		}
	}
	//*/
//	if(	(Exposure_Time==Integral_Time)&&(Threshold==Image_Threshold)&&(Tracing_Algorithm==Image_Algorithm)	)
		if(	(Exposure_Time==Integral_Time)&&(Threshold==Image_Threshold)&&(Windows_Switch==Camera_Window)&&(Tracing_Algorithm==Image_Algorithm)	)
	{
		Camera_Error=0;
	}
}

/**
  * @brief	���Ӧ��
	* @data		
  */
u16 Camera_Param_Addr=0,Camera_Param_Data=0;
void Camera_Answer(u32 ADDR,u8 Data[])
{
	u8 i;
	u8 Check=0,C1=0,C2=0;
	u16 Temp_Addr=0,Temp_Data=0;
//	u16 Camera_Param_Addr=0,Camera_Param_Data=0;
	u32 readADDRTemp=0;
	readADDRTemp=ADDR;	
	for(i=0;i<16;i++)
	{
		Data[i]=*(uint32_t*)(readADDRTemp);
		readADDRTemp+=2;
	}		
	if(Data[0]==0x7C)//֡ͷ
	{
		if(Data[15]==0x3C)//֡β
		{		
			Temp_Addr=Hex_To_Char(Data[5]);		Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[6]);	Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[7]);	Temp_Addr<<=4;
			Temp_Addr+=Hex_To_Char(Data[8]);
			
			Temp_Data=Hex_To_Char(Data[9]);		Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[10]);	Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[11]);	Temp_Data<<=4;
			Temp_Data+=Hex_To_Char(Data[12]);
#ifdef CAMERA_SETTING_CHECK
			/*У�� = �豸��ַ+���ò�����ַ+���ò������� = 60H + 00H + 00H +00H + 00H = 60H*/
			Check=0x60+	\
						((u8)(Temp_Addr&0x00FF))+((u8)((Temp_Addr&0xFF00)>>8))+	\
						((u8)(Temp_Data&0x00FF))+((u8)((Temp_Data&0xFF00)>>8));	
			C1=Hex_To_Char(Check&0xF0);
			C2=Hex_To_Char(Check&0x0F);
			if((Data[13]==0x7C)&(Data[14]==0x7C))//У��
			{
#endif
				Camera_Param_Addr=Temp_Addr;
				Camera_Param_Data=Temp_Data;
#ifdef CAMERA_SETTING_CHECK
			}
			//else Camera_ADD_Error=1;
#endif
			Camera_Answer_Param(Camera_Param_Addr,Camera_Param_Data);
		}
		//else Camera_ADD_Error=2;
	}
	//else Camera_ADD_Error=3;
}

/**
  * @brief	���Ӧ�����
	* @data		
  */
void Camera_Answer_Param(u16 Param_Addr,u16 Param_Data)
{
	u32 Temp=0;
	switch(Param_Addr)
	{
		case Exposure_Time_Addr2:
		{
			Integral_Time_High=Param_Data;	
			break;
		}
		case Exposure_Time_Addr1:
		{
			Integral_Time_Low=Param_Data;
			break;
		}
		case Tracking_Algorithm_Addr:
		{
			Image_Algorithm=Param_Data;
			break;
		}	
		case Small_Window_Switch:
		{
			Camera_Window=Param_Data;
			break;
		}
		case Threshold_Addr:
		{	
			Image_Threshold=Param_Data;
			break;
		}
	}
	Temp=Integral_Time_High;	Temp<<=16;
	Temp+=Integral_Time_Low;
	Integral_Time=Temp;
}

///////////////////////////////////////////
///////////////////////////////

/////////////////////////////////////////�ڶ������
/**
  * @brief	�����������
	* @data		
  */
void Camera_Param_Set2(u16 Param_Addr,u16 Param_Data,u8 Data[])
{
	u8 Check=0;
	Data[0]=0x7E;//֡ͷ
	/*֡����*/
	Data[1]=0x30;
	Data[2]=0x45;
	/*�豸��ַ*/
	Data[3]=0x36;
	Data[4]=0x30;
	/*�豸������ַ*/ 
	Data[8]=Char_To_Hex((u8)(Param_Addr&0x000F));
	Data[7]=Char_To_Hex((u8)((Param_Addr&0x00F0)>>4));
	Data[6]=Char_To_Hex((u8)((Param_Addr&0x0F00)>>8));
	Data[5]=Char_To_Hex((u8)((Param_Addr&0xF000)>>12));
	/*���ò�������*/
	Data[12]=Char_To_Hex((u8)(Param_Data&0x000F));
	Data[11]=Char_To_Hex((u8)((Param_Data&0x00F0)>>4));
	Data[10]=Char_To_Hex((u8)((Param_Data&0x0F00)>>8));
	Data[9]=Char_To_Hex((u8)((Param_Data&0xF000)>>12));
#ifdef CAMERA_SETTING_CHECK
	/*У�� = �豸��ַ+���ò�����ַ+���ò������� = 60H + 00H + 00H +00H + 00H = 60H*/
	Check=0x60+	\
				((u8)(Param_Addr&0x00FF))+((u8)((Param_Addr&0xFF00)>>8))+	\
				((u8)(Param_Data&0x00FF))+((u8)((Param_Data&0xFF00)>>8));	
	Data[13]=Char_To_Hex((u8)((Check&0x00F0)>>4));
	Data[14]=Char_To_Hex((u8)(Check&0x000F));
#endif
	Data[15]=0x3E;//֡β
}

/**
  * @brief	������õڶ������
	* @data		
  */
