#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1����RX0�ж�ʹ��
#define CAN2_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
void CAN1_Mode_Init(void);
void CAN2_Mode_Init(void);             //CAN��ʼ��

void CAN1_Mode_Init1(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��
void CAN2_Mode_Init2(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
void NVIC_config(void);
 
u8 CAN2_Send_Msg(u8* msg,u8 len);						//��������

u8 CAN2_Receive_Msg(u8 *buf);							//��������
#endif

