#ifndef __IO_INIT_H
#define __IO_INIT_H
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//#define FW_zero_degree_zheng 201.2221     //0.02 《- 201.2021
//#define FW_zero_degree_fu -158.22
//#define FW_zero_degree_fu 415        //测试用
//#define FY_zero_degree  323.08         //0.06 《- 323.02

//LED端口定义
#define LED0 PBout(2)	// DS0 

void DA12_out(float da_in1,float da_in2);
void IO_Init(void);//初始化	
void Delay_ms(int num);
void Delay_us(int num);
#endif
