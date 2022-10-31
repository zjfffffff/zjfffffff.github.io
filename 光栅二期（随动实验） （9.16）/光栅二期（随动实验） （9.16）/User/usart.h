#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "stm32f4xx_it.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2011/6/14
//�汾��V1.4
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			4  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart4_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound_3);
void Delay_us(int num);



void Com_Read(u32 ADDR);
void Com_Read_1(u32 ADDR,u16 data[],u8 len);
void Com_Write(u32 ADDR,u32 Com_En,u16 TX_Len);
void SF_Com_Falg_Read(void);
//void SF_DATA_1K();

//typedef union
//{
//	struct
//	{
//			u8 Low_byte;
//			u8 MLow_byte;
//			u8 MHigh_byte;
//			u8 High_byte;
//	}Data_byte;
//  float Data_float;
//	u32 Data_u32;
//	s32 Data_s32;
//}Data_value;

//typedef struct
//{
//	float Gyro_X;//
//  float Gyro_Y;//
//	float Gyro_Z;//
//	
//	float Gyro_X_axis_plus_meter;
//  float Gyro_Y_axis_plus_meter;
//	float Gyro_Z_axis_plus_meter;
//	
//	float Gyro_Roll;          //�����
//  float Gyro_Pitch;         //������
//	float Gyro_Yaw;         //�����
//}Gyro_DATA;

#endif


