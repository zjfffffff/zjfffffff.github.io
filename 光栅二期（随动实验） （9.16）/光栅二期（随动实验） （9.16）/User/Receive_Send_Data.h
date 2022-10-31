#ifndef _Receive_Send_Data_h
#define _Receive_Send_Data_h



#include "stm32f4xx.h"	


#define Receive_ADDR1        ((u32)(0x64001000))		//���ŷ���������ܿذ�ͨ��
#define Receive_ADDR2        ((u32)(0x64000800))		//���ŷ���������ܿذ�ͨ��
#define Receive_ADDR3        ((u32)(0x64000800))		//���ŷ�������λ��ͨ��
#define Receive_Com_En       ((u32)(0x64001400))		//���ŷ���������ܿذ�ͨ��ʹ��


#define Receive_RX_Len 30							//���ŷ���������ܿذ�ͨ�ŵ��ֽ�
#define Receive_TX_Len 49							//���ŷ���������ܿذ�ͨ�ŵ��ֽ�
#define Shangweiji_RX_Len 43					//���ŷ�������λ��ͨ�ŵ��ֽ�
#define Receive_Suidong_GS_Len 13
void Receive_ShangweijiData_Read(void)	;
void Send_JiXiaData_Write(void);			//���ŷ���������ܿذ�ͨ�ź���
void Receive_JiShangData_Read(void);	//���ŷ���������ܿذ�ͨ�ź���
void Receive_DATA_Parse(void);				//���ŷ���������ܿذ�ͨ�ŵ����ݽ������
void Send_Data_TX(void);							//���ŷ���������ܿذ�ͨ�ŵ����ݽ��
void Receive_Shangweiji_DATA_Parse(void);//���ŷ�������λ��ͨ��
//void Send_Tongbuxinhao_Write_Low(void);//��դ����ͬ���źŵ͵�ƽ
//void Send_Tongbuxinhao_Write_High(void);//��դ����ͬ���źŸߵ�ƽ
void Receive_DBJsuidong_Read();
void Receive_Deadscale_Suidong_GS();
void Receive_Suidong_GS();
void Send_SW_Data_Write();   //������λ������ 2022.8.16
void Send_SW_Data_TX(void);	 //2022.8.16
#endif