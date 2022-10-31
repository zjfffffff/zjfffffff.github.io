#include "elmo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "can.h"

extern int32_t vel_FY;   //wSP���ٶ�    
extern int32_t acc_FY;   //wAC�ļ��ٶ�
extern int32_t pos_FY;   //wPA,wPX,wPR
extern int32_t spe_FY;   //wJV,wSP
extern int32_t sdac_FY;
extern int32_t dcc_FY;
extern int32_t sf_FY;

extern int32_t vel_FW;   //wSP���ٶ�  
extern int32_t acc_FW;   //wAC�ļ��ٶ�
extern int32_t pos_FW;   //wPA,wPX,wPR
extern int32_t spe_FW;   //wJV,wSP
extern int32_t sdac_FW;
extern int32_t dcc_FW;
extern int32_t sf_FW;
extern float   TI1_fw;
#define SRAM_ADDR  	((uint32_t)0x20000000)
uint32_t *pBuf;
float *pBuf_float;

void wRS_FY(void)   //д������
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x00;          //��׼ID  11λ    0x00 -- 0x7FF
	TxMessage.RTR = CAN_RTR_DATA;    // CAN_RTR_DATA = 0x00000000  ����֡��Զ��֡
	TxMessage.IDE = CAN_ID_STD;      //CAN_Id_Standard = 0x00000000  ��׼����չ
	TxMessage.DLC = 8;               //���ݳ���0-8   
	TxMessage.Data[0] = 0x01;    
	TxMessage.Data[1] = 0x00;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = 0x00;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wRS_FW(void)   //д������
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x00;          //��׼ID  11λ    0x00 -- 0x7FF
	TxMessage.RTR = CAN_RTR_DATA;    // CAN_RTR_DATA = 0x00000000  ����֡��Զ��֡
	TxMessage.IDE = CAN_ID_STD;      //CAN_Id_Standard = 0x00000000  ��׼����չ
	TxMessage.DLC = 8;               //���ݳ���0-8   
	TxMessage.Data[0] = 0x01;    
	TxMessage.Data[1] = 0x00;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = 0x00;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}


/****************************************************
Name:Unit Mode
Type:Integer,Read/Write
Range:1 to 6
Default:3
Remarks: 
1-Torque Control loop
2-Speed Control loop
3-Stepper
4-Reserved
5-Position Loop
6-Stepper open loop 
****************************************************/ 
void wUM(uint16_t CAN_ID,uint8_t CAN_DATA4)     //д����ģʽ
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x55;    
	TxMessage.Data[1] = 0x4d;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN1,&TxMessage); 			//��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);		
}
void rUM(uint16_t CAN_ID)  //������ģʽ
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 			//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA; 	//����֡
	TxMessage.IDE = CAN_ID_STD;  		//��׼֡
	TxMessage.DLC = 4;            	//���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x55;    
	TxMessage.Data[1] = 0x4d;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage);  //��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);		
}
/****************************************************
Name:Jog Velocity
Type:Integer,Read/Write
Range: (-2^31) to (2^31-1)
Default:0 
****************************************************/
/*void rPA(void)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x37f; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}*/
void wJV(uint16_t CAN_ID,int32_t speed)    //д�����˶��ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(speed >= 0)
	{
		spe_FY = speed; 
	}
	else
	{
		spe_FY = speed + 0xFFFFFFFF +1;
	} 

	CAN_DATA4 = (spe_FY & 0x000000FF);	
	CAN_DATA5 = (spe_FY & 0x0000FF00)>>8;	
	CAN_DATA6 = (spe_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (spe_FY & 0xFF000000)>>24;	
    
	TxMessage.StdId = CAN_ID;            //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;        //����֡
	TxMessage.IDE = CAN_ID_STD;          //��׼֡
	TxMessage.DLC = 8;            			 //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x4A;    
	TxMessage.Data[1] = 0x56;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 			 //��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
/****************************************************
Name:Position Absolute
Type:Integer,Read/Write
Range: (-2^31) to (2^31-1)
Default:0 
****************************************************/
void rPA(uint16_t CAN_ID)   //������λ��
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 4;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;           
	TxMessage.Data[1] = 0x41;           
	TxMessage.Data[2] = 0x00;           
	TxMessage.Data[3] = 0x40;           
	CAN_Transmit(CAN2,&TxMessage);      //��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}                                     
void wPA_FY(uint16_t CAN_ID,int32_t position)    //д����λ��
{                                     
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position >= 0)
	{
		pos_FY = position; 
	}
	else
	{
		pos_FY = position + 0xFFFFFFFF +1;
	} 

	CAN_DATA4 = (pos_FY & 0x000000FF);	
	CAN_DATA5 = (pos_FY & 0x0000FF00)>>8;	
	CAN_DATA6 = (pos_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (pos_FY & 0xFF000000)>>24;	
    
	TxMessage.StdId = CAN_ID; 				  //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;            			//���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 			//��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wPA_FW(uint16_t CAN_ID,int32_t position)    //д����λ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position >= 0)
	{
		pos_FW = position; 
	}
	else
	{
		pos_FW = position + 0xFFFFFFFF +1;
	} 

	CAN_DATA4 = (pos_FW & 0x000000FF);	
	CAN_DATA5 = (pos_FW & 0x0000FF00)>>8;	
	CAN_DATA6 = (pos_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (pos_FW & 0xFF000000)>>24;	
    
	TxMessage.StdId = CAN_ID; 					 //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;        //����֡
	TxMessage.IDE = CAN_ID_STD;          //��׼֡
	TxMessage.DLC = 8;            			 //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 			//��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

 void wPR_FY(uint16_t CAN_ID,int32_t position)     //д���λ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position >= 0)
	{
		pos_FY = position; 
	}
	else
	{
		pos_FY = position + 0xFFFFFFFF +1;
	} 

	CAN_DATA4 = (pos_FY & 0x000000FF);	
	CAN_DATA5 = (pos_FY & 0x0000FF00)>>8;	
	CAN_DATA6 = (pos_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (pos_FY & 0xFF000000)>>24;	
    
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 				//��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void wPR_FW(uint16_t CAN_ID,int32_t position)     //д���λ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position >= 0)
	{
		pos_FW = position; 
	}
	else
	{
		pos_FW = position + 0xFFFFFFFF +1;
	} 

	CAN_DATA4 = (pos_FW & 0x000000FF);	
	CAN_DATA5 = (pos_FW & 0x0000FF00)>>8;	
	CAN_DATA6 = (pos_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (pos_FW & 0xFF000000)>>24;	
    
	TxMessage.StdId = CAN_ID; 						 //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;          //����֡
	TxMessage.IDE = CAN_ID_STD;            //��׼֡
	TxMessage.DLC = 8;                     //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 				//��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void wPX_FY(uint16_t CAN_ID,int32_t position)  //д��λ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position > 0)
	{
		pos_FY = position; 
	}
	else 
	{
		pos_FY = position + 0xFFFFFFFF +1;
	} 
   if(position == 0)
	 {
			CAN_DATA4 = 0;	
			CAN_DATA5 = 0;	
			CAN_DATA6 = 0;
			CAN_DATA7 = 0;	
	 }
	 else
	 {
			CAN_DATA4 = (pos_FY & 0x000000FF);	
			CAN_DATA5 = (pos_FY & 0x0000FF00)>>8;	
			CAN_DATA6 = (pos_FY & 0x00FF0000)>>16;
			CAN_DATA7 = (pos_FY & 0xFF000000)>>24;
			
	 }
		
    
	TxMessage.StdId = CAN_ID; 								//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;             //����֡
	TxMessage.IDE = CAN_ID_STD;               //��׼֡
	TxMessage.DLC = 8;                        //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 						//��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wPX_FW(uint16_t CAN_ID,int32_t position)  //д��λ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 
	if(position > 0)
	{
		pos_FW = position; 
	}
	else 
	{
		pos_FW = position + 0xFFFFFFFF +1;
	} 
   if(position == 0)
	 {
			CAN_DATA4 = 0;	
			CAN_DATA5 = 0;	
			CAN_DATA6 = 0;
			CAN_DATA7 = 0;	
	 }
	 else
	 {
			CAN_DATA4 = (pos_FW & 0x000000FF);	
			CAN_DATA5 = (pos_FW & 0x0000FF00)>>8;	
			CAN_DATA6 = (pos_FW & 0x00FF0000)>>16;
			CAN_DATA7 = (pos_FW & 0xFF000000)>>24;
			
	 }
		
    
	TxMessage.StdId = CAN_ID; 								//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;             //����֡
	TxMessage.IDE = CAN_ID_STD;               //��׼֡
	TxMessage.DLC = 8;                        //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 						//��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Torque Command
Type:Float,Read/Write
Range: -PL[1] to PL[1]
Default:0 
****************************************************/
void rTC(uint16_t CAN_ID)    //��ת��
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 						 //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;          //����֡
	TxMessage.IDE = CAN_ID_STD;            //��׼֡
	TxMessage.DLC = 4;                     //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 				 //��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}/*
void wTC(uint16_t CAN_ID,float current)
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	 


	CAN_DATA4 = (pos & 0x000000FF);	
	CAN_DATA5 = (pos & 0x0000FF00)>>8;	
	CAN_DATA6 = (pos & 0x00FF0000)>>16;
	CAN_DATA7 = (pos & 0xFF000000)>>24;	
			
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x80; //������    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}*/

/****************************************************
Name:Motor On
Type:Integer,Read/Write
Range: 0,1
Default:0 
****************************************************/
void rMO(void)   //������
{
	CanTxMsg TxMessage;								
	TxMessage.StdId = 0x37f; 							//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 4;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x4f;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage); 				//��ȡ��������
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}


void wMO_FY(uint16_t CAN_ID,uint8_t CAN_DATA4)    //д����
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 							//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;           //����֡
	TxMessage.IDE = CAN_ID_STD;             //��׼֡
	TxMessage.DLC = 8;                      //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x4f;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN2,&TxMessage); 				//��ȡ��������
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
		
}
void wMO_FW(uint16_t CAN_ID,uint8_t CAN_DATA4)    //д����
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x4f;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
		
}

/****************************************************
Name:PTP Profiler Speed
Type:Integer,Read/Write
Range: 1 to 2e9
Default:100000
****************************************************/
void rSP(void)   //PTPģʽ�ٶȣ���Ե㣩  ����   ����   ����
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x37f; 							//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 4;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x50;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wSP_FY(uint16_t CAN_ID,int32_t speed)    //PTPģʽ�ٶȣ���Ե㣩  ����   ����   ����
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	vel_FY = speed; 

	CAN_DATA4 = (vel_FY & 0x000000FF);
	CAN_DATA5 = (vel_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (vel_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (vel_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 					 //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;        //����֡
	TxMessage.IDE = CAN_ID_STD;          //��׼֡
	TxMessage.DLC = 8;                   //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x50;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wSP_FW(uint16_t CAN_ID,int32_t speed)    //PTPģʽ�ٶȣ���Ե㣩  ����   ����   ����
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	vel_FW = speed; 

	CAN_DATA4 = (vel_FW & 0x000000FF);
	CAN_DATA5 = (vel_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (vel_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (vel_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA; 	      //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x50;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
void wAC_FY(uint16_t CAN_ID,int32_t acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	acc_FY = acceleration; 

	CAN_DATA4 = (acc_FY & 0x000000FF);
	CAN_DATA5 = (acc_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (acc_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (acc_FY & 0xFF000000)>>24;

			
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wAC_FW(uint16_t CAN_ID,int32_t acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	acc_FW = acceleration; 

	CAN_DATA4 = (acc_FW & 0x000000FF);
	CAN_DATA5 = (acc_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (acc_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (acc_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
void wDC_FY(uint16_t CAN_ID,int32_t acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	dcc_FY = acceleration; 

	CAN_DATA4 = (dcc_FY & 0x000000FF);
	CAN_DATA5 = (dcc_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (dcc_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (dcc_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x44;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wDC_FW(uint16_t CAN_ID,int32_t acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	dcc_FW = acceleration; 

	CAN_DATA4 = (dcc_FW & 0x000000FF);
	CAN_DATA5 = (dcc_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (dcc_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (dcc_FW & 0xFF000000)>>24;

	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x44;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
void wSD_FY(uint16_t CAN_ID,int32_t sd_acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sdac_FY = sd_acceleration; 

	CAN_DATA4 = (sdac_FY & 0x000000FF);
	CAN_DATA5 = (sdac_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (sdac_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (sdac_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 							//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;           //����֡
	TxMessage.IDE = CAN_ID_STD;             //��׼֡
	TxMessage.DLC = 8;                      //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;  //S  
	TxMessage.Data[1] = 0x44;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wSD_FW(uint16_t CAN_ID,int32_t sd_acceleration)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sdac_FW = sd_acceleration; 

	CAN_DATA4 = (sdac_FW & 0x000000FF);
	CAN_DATA5 = (sdac_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (sdac_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (sdac_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 							//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;           //����֡
	TxMessage.IDE = CAN_ID_STD;             //��׼֡
	TxMessage.DLC = 8;                      //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;  //S  
	TxMessage.Data[1] = 0x44;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
void wSF_FY(uint16_t CAN_ID,int32_t sf_value)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sf_FY = sf_value; 

	CAN_DATA4 = (sf_FY & 0x000000FF);
	CAN_DATA5 = (sf_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (sf_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (sf_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;         //����֡
	TxMessage.IDE = CAN_ID_STD;           //��׼֡
	TxMessage.DLC = 8;                    //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;  //S  
	TxMessage.Data[1] = 0x46;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wSF_FW(uint16_t CAN_ID,int32_t sf_value)   //���ٶ�
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sf_FW = sf_value; 

	CAN_DATA4 = (sf_FW & 0x000000FF);
	CAN_DATA5 = (sf_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (sf_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (sf_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						 //��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;          //����֡
	TxMessage.IDE = CAN_ID_STD;            //��׼֡
	TxMessage.DLC = 8;                     //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;  //S  
	TxMessage.Data[1] = 0x46;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Main Homing
Type:Integer,Read/Write
Index Range: 1 ro 13
****************************************************/
void rHM(uint16_t CAN_ID,uint8_t INDEX)   //����
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 4;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x48;    
	TxMessage.Data[1] = 0x4D;    
	TxMessage.Data[2] = INDEX;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wHM(uint16_t CAN_ID,uint8_t INDEX,uint8_t CAN_DATA4)  //
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x48;    
	TxMessage.Data[1] = 0x4D;    
	TxMessage.Data[2] = INDEX;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wTC(uint16_t CAN_ID,float cur)//-0.2  ת������
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	uint32_t CUR=0;
	CUR = f_to_i(cur);
	
	
	CAN_DATA4 = (CUR & 0x000000FF);
	CAN_DATA5 = (CUR & 0x0000FF00)>>8;
	CAN_DATA6 = (CUR & 0x00FF0000)>>16;
	CAN_DATA7 = (CUR & 0xFF000000)>>24;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x04;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x80;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
	
}
void wCL(uint16_t CAN_ID,uint8_t INDEX,float cur)     //����������Ƽ�������ͱ�������
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	uint32_t CUR=0;
	CUR = f_to_i(cur);

	CAN_DATA4 = (CUR & 0x000000FF);
	CAN_DATA5 = (CUR & 0x0000FF00)>>8;
	CAN_DATA6 = (CUR & 0x00FF0000)>>16;
	CAN_DATA7 = (CUR & 0xFF000000)>>24;
	
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 8;            
	TxMessage.Data[0] = 0x43;    
	TxMessage.Data[1] = 0x4C;    
	TxMessage.Data[2] = INDEX;    
	TxMessage.Data[3] = 0x80;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;    
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void wPL(uint16_t CAN_ID,uint8_t INDEX,float cur)    //�����ֵ�����ͷ�ֵ����ʱ��
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	uint32_t CUR=0;
	CUR = f_to_i(cur);

	CAN_DATA4 = (CUR & 0x000000FF);
	CAN_DATA5 = (CUR & 0x0000FF00)>>8;
	CAN_DATA6 = (CUR & 0x00FF0000)>>16;
	CAN_DATA7 = (CUR & 0xFF000000)>>24;
	
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 8;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x4C;    
	TxMessage.Data[2] = INDEX;    
	TxMessage.Data[3] = 0x80;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;    
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
/****************************************************
Name:Begin Motion
Type:Command
****************************************************/
void wBG_FY(uint16_t CAN_ID)    //��ʼ�˶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;     //����֡
	TxMessage.IDE = CAN_ID_STD;       //��׼֡
	TxMessage.DLC = 4;                //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x42;    
	TxMessage.Data[1] = 0x47;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wBG_FW(uint16_t CAN_ID)    //��ʼ�˶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;     //����֡
	TxMessage.IDE = CAN_ID_STD;       //��׼֡
	TxMessage.DLC = 4;                //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x42;    
	TxMessage.Data[1] = 0x47;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Stop Motion
Type:Command
****************************************************/
void wST_FY(uint16_t CAN_ID)    //ֹͣ�˶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x54;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wST_FW(uint16_t CAN_ID)    //ֹͣ�˶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;     //����֡
	TxMessage.IDE = CAN_ID_STD;       //��׼֡
	TxMessage.DLC = 4;                //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x54;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Main Position in Counts
Type:Integer,Read/Write
Range: -2^31 to 2^31-1
****************************************************/
void rPX_FY(uint16_t CAN_ID)    //��ȡ��ǰλ��
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40; 
	TxMessage.Data[4] = 0x00;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void rPX_FW(uint16_t CAN_ID)    //��ȡ��ǰλ��
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//��׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;       //����֡
	TxMessage.IDE = CAN_ID_STD;         //��׼֡
	TxMessage.DLC = 8;                  //���ݳ��ȣ����Ϊ8��
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40; 
	TxMessage.Data[4] = 0x00;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage); 
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Feedback Velocity
Type:Integer,Read
****************************************************/
void rPE(uint16_t CAN_ID)   //λ�����
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x45;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void rVX(uint16_t CAN_ID)      //��ȡ��ǰ�ٶ�  ��ֵ/��
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x56;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void rVE(uint16_t CAN_ID)   //�ٶ����  
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x56;    
	TxMessage.Data[1] = 0x45;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void rAC(uint16_t CAN_ID)   //���ٶ�  
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void rIQ(uint16_t CAN_ID)   //����  
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x49;    
	TxMessage.Data[1] = 0x51;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void rIQ_FW(uint16_t CAN_ID)   //����  
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x49;    
	TxMessage.Data[1] = 0x51;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

////////////////////////////////////////////2020.03.31���///////////////////////////////////////////////////
void EC_FWRead(uint16_t CAN_ID)   //ELMO�������   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x45;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}


void MF_FWRead(uint16_t CAN_ID)   //����������   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x46;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void SR_FWRead(uint16_t CAN_ID)   //״̬�Ĵ���   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void AN_1_FWRead(uint16_t CAN_ID)   //ʵʱ����A��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x03;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void AN_2_FWRead(uint16_t CAN_ID)   //ʵʱ����B��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x04;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void AN_3_FWRead(uint16_t CAN_ID)   //ʵʱ����C��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x05;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void AN_4_FWRead(uint16_t CAN_ID)   //ʵʱĸ�ߵ�ѹ   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x06;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void EE_1_FWRead(uint16_t CAN_ID)   //������״̬   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x45;    
	TxMessage.Data[1] = 0x45;    
	TxMessage.Data[2] = 0x01;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void TI_1_FWRead(uint16_t CAN_ID)   //�������¶�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x49;    
	TxMessage.Data[2] = 0x01;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void PX_FWRead(uint16_t CAN_ID)   //�Ƕ�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void VX_FWRead(uint16_t CAN_ID)   //�ٶ�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x56;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void SO_FWRead(uint16_t CAN_ID)   //���ʹ��״̬  CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x4F;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

///////////////////////////////////////////////////////////////////////////////////////////////

void PY_FWRead(uint16_t CAN_ID)   //�Ƕ�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x59;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void WD_FW_Read(uint16_t CAN_ID)   //�¶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x49;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN1,&TxMessage); 	
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void WD_FY_Read(uint16_t CAN_ID)   //�¶�
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x49;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}


uint32_t f_to_i(float x)   //
{
	uint32_t y;
	
	pBuf = (uint32_t *)SRAM_ADDR;
	pBuf_float = (float *)SRAM_ADDR;
	
	*pBuf_float = x;
	y=*pBuf;
	
	return y;
	
}



////////////////////////////////////////////2020.03.31���///////////////////////////////////////////////////
void EC_FYRead(uint16_t CAN_ID)   //ELMO�������   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x45;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}


void MF_FYRead(uint16_t CAN_ID)   //����������   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x46;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void SR_FYRead(uint16_t CAN_ID)   //״̬�Ĵ���   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void AN_1_FYRead(uint16_t CAN_ID)   //ʵʱ����A��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x03;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void AN_2_FYRead(uint16_t CAN_ID)   //ʵʱ����B��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x04;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void AN_3_FYRead(uint16_t CAN_ID)   //ʵʱ����C��   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x05;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void AN_4_FYRead(uint16_t CAN_ID)   //ʵʱĸ�ߵ�ѹ   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x41;    
	TxMessage.Data[1] = 0x4E;    
	TxMessage.Data[2] = 0x06;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void EE_1_FYRead(uint16_t CAN_ID)   //������״̬   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x45;    
	TxMessage.Data[1] = 0x45;    
	TxMessage.Data[2] = 0x01;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void TI_1_FYRead(uint16_t CAN_ID)   //�������¶�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x49;    
	TxMessage.Data[2] = 0x01;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void PX_FYRead(uint16_t CAN_ID)   //�Ƕ�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void VX_FYRead(uint16_t CAN_ID)   //�ٶ�   CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x56;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void SO_FYRead(uint16_t CAN_ID)   //���ʹ��״̬  CAN1
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 
	TxMessage.RTR = CAN_RTR_DATA; 
	TxMessage.IDE = CAN_ID_STD;  
	TxMessage.DLC = 4;            
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x4F;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 	
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void wAS_FY(uint16_t CAN_ID,float vol)    //дƫִ��ѹ CAN2
{
		CanTxMsg TxMessage;
		int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
		uint32_t CUR=0;
		CUR = f_to_i(vol);

  	   CAN_DATA4 = (CUR & 0x000000FF);
  	   CAN_DATA5 = (CUR & 0x0000FF00)>>8;
	   CAN_DATA6 = (CUR & 0x00FF0000)>>16;
	   CAN_DATA7 = (CUR & 0xFF000000)>>24;
	 
		TxMessage.StdId = CAN_ID; 
		TxMessage.RTR = CAN_RTR_DATA; 
		TxMessage.IDE = CAN_ID_STD;  
		TxMessage.DLC = 8;            
		TxMessage.Data[0] = 0x41;    
		TxMessage.Data[1] = 0x53;    
		TxMessage.Data[2] = 0x01;    
		TxMessage.Data[3] = 0x80;    
		TxMessage.Data[4] = CAN_DATA4;    
		TxMessage.Data[5] = CAN_DATA5;     
		TxMessage.Data[6] = CAN_DATA6;    
		TxMessage.Data[7] = CAN_DATA7;      
		CAN_Transmit(CAN2,&TxMessage); 
		while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

///////////////////////////////////////////////////////////////////////////////////////////////
