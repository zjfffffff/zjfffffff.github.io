#ifndef __IO_INIT_H
#define __IO_INIT_H
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//#define FW_zero_degree_zheng 201.2221     //0.02 ��- 201.2021
//#define FW_zero_degree_fu -158.22
//#define FW_zero_degree_fu 415        //������
//#define FY_zero_degree  323.08         //0.06 ��- 323.02

//LED�˿ڶ���
#define LED0 PBout(2)	// DS0 

void DA12_out(float da_in1,float da_in2);
void IO_Init(void);//��ʼ��	
void Delay_ms(int num);
void Delay_us(int num);
#endif
