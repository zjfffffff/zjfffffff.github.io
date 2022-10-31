#ifndef _Receive_Send_Data_h
#define _Receive_Send_Data_h



#include "stm32f4xx.h"	


#define Receive_ADDR1        ((u32)(0x64001000))		//粗伺服板与机上总控板通信
#define Receive_ADDR2        ((u32)(0x64000800))		//粗伺服板与机下总控板通信
#define Receive_ADDR3        ((u32)(0x64000800))		//粗伺服板与上位机通信
#define Receive_Com_En       ((u32)(0x64001400))		//粗伺服板与机下总控板通信使能


#define Receive_RX_Len 30							//粗伺服板与机上总控板通信的字节
#define Receive_TX_Len 49							//粗伺服板与机下总控板通信的字节
#define Shangweiji_RX_Len 43					//粗伺服板与上位机通信的字节
#define Receive_Suidong_GS_Len 13
void Receive_ShangweijiData_Read(void)	;
void Send_JiXiaData_Write(void);			//粗伺服板与机下总控板通信函数
void Receive_JiShangData_Read(void);	//粗伺服板与机上总控板通信函数
void Receive_DATA_Parse(void);				//粗伺服板与机上总控板通信的数据解包函数
void Send_Data_TX(void);							//粗伺服板与机下总控板通信的数据解包
void Receive_Shangweiji_DATA_Parse(void);//粗伺服板与上位机通信
//void Send_Tongbuxinhao_Write_Low(void);//光栅二期同步信号低电平
//void Send_Tongbuxinhao_Write_High(void);//光栅二期同步信号高电平
void Receive_DBJsuidong_Read();
void Receive_Deadscale_Suidong_GS();
void Receive_Suidong_GS();
void Send_SW_Data_Write();   //发给上位机数据 2022.8.16
void Send_SW_Data_TX(void);	 //2022.8.16
#endif