#include "elmo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "can.h"

extern int32_t vel_FY;   //wSP的速度    
extern int32_t acc_FY;   //wAC的加速度
extern int32_t pos_FY;   //wPA,wPX,wPR
extern int32_t spe_FY;   //wJV,wSP
extern int32_t sdac_FY;
extern int32_t dcc_FY;
extern int32_t sf_FY;

extern int32_t vel_FW;   //wSP的速度  
extern int32_t acc_FW;   //wAC的加速度
extern int32_t pos_FW;   //wPA,wPX,wPR
extern int32_t spe_FW;   //wJV,wSP
extern int32_t sdac_FW;
extern int32_t dcc_FW;
extern int32_t sf_FW;
extern float   TI1_fw;
#define SRAM_ADDR  	((uint32_t)0x20000000)
uint32_t *pBuf;
float *pBuf_float;

void wRS_FY(void)   //写软重置
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x00;          //标准ID  11位    0x00 -- 0x7FF
	TxMessage.RTR = CAN_RTR_DATA;    // CAN_RTR_DATA = 0x00000000  数据帧和远程帧
	TxMessage.IDE = CAN_ID_STD;      //CAN_Id_Standard = 0x00000000  标准和扩展
	TxMessage.DLC = 8;               //数据长度0-8   
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
void wRS_FW(void)   //写软重置
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x00;          //标准ID  11位    0x00 -- 0x7FF
	TxMessage.RTR = CAN_RTR_DATA;    // CAN_RTR_DATA = 0x00000000  数据帧和远程帧
	TxMessage.IDE = CAN_ID_STD;      //CAN_Id_Standard = 0x00000000  标准和扩展
	TxMessage.DLC = 8;               //数据长度0-8   
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
void wUM(uint16_t CAN_ID,uint8_t CAN_DATA4)     //写运行模式
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;                  //数据长度：最大为8；
	TxMessage.Data[0] = 0x55;    
	TxMessage.Data[1] = 0x4d;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN1,&TxMessage); 			//获取发送邮箱
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);		
}
void rUM(uint16_t CAN_ID)  //读运行模式
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 			//标准标识符
	TxMessage.RTR = CAN_RTR_DATA; 	//数据帧
	TxMessage.IDE = CAN_ID_STD;  		//标准帧
	TxMessage.DLC = 4;            	//数据长度：最大为8；
	TxMessage.Data[0] = 0x55;    
	TxMessage.Data[1] = 0x4d;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage);  //获取发送邮箱
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
void wJV(uint16_t CAN_ID,int32_t speed)    //写持续运动速度
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
    
	TxMessage.StdId = CAN_ID;            //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;        //数据帧
	TxMessage.IDE = CAN_ID_STD;          //标准帧
	TxMessage.DLC = 8;            			 //数据长度：最大为8；
	TxMessage.Data[0] = 0x4A;    
	TxMessage.Data[1] = 0x56;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 			 //获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
/****************************************************
Name:Position Absolute
Type:Integer,Read/Write
Range: (-2^31) to (2^31-1)
Default:0 
****************************************************/
void rPA(uint16_t CAN_ID)   //读绝对位置
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 4;                  //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;           
	TxMessage.Data[1] = 0x41;           
	TxMessage.Data[2] = 0x00;           
	TxMessage.Data[3] = 0x40;           
	CAN_Transmit(CAN2,&TxMessage);      //获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}                                     
void wPA_FY(uint16_t CAN_ID,int32_t position)    //写绝对位置
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
    
	TxMessage.StdId = CAN_ID; 				  //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;            			//数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 			//获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wPA_FW(uint16_t CAN_ID,int32_t position)    //写绝对位置
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
    
	TxMessage.StdId = CAN_ID; 					 //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;        //数据帧
	TxMessage.IDE = CAN_ID_STD;          //标准帧
	TxMessage.DLC = 8;            			 //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 			//获取发送邮箱
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

 void wPR_FY(uint16_t CAN_ID,int32_t position)     //写相对位置
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
    
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 				//获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}

void wPR_FW(uint16_t CAN_ID,int32_t position)     //写相对位置
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
    
	TxMessage.StdId = CAN_ID; 						 //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;          //数据帧
	TxMessage.IDE = CAN_ID_STD;            //标准帧
	TxMessage.DLC = 8;                     //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x52;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 				//获取发送邮箱
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}

void wPX_FY(uint16_t CAN_ID,int32_t position)  //写主位置
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
		
    
	TxMessage.StdId = CAN_ID; 								//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;             //数据帧
	TxMessage.IDE = CAN_ID_STD;               //标准帧
	TxMessage.DLC = 8;                        //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN2,&TxMessage); 						//获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wPX_FW(uint16_t CAN_ID,int32_t position)  //写主位置
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
		
    
	TxMessage.StdId = CAN_ID; 								//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;             //数据帧
	TxMessage.IDE = CAN_ID_STD;               //标准帧
	TxMessage.DLC = 8;                        //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x58;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = CAN_DATA5;     
	TxMessage.Data[6] = CAN_DATA6;    
	TxMessage.Data[7] = CAN_DATA7;      
	CAN_Transmit(CAN1,&TxMessage); 						//获取发送邮箱
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}
/****************************************************
Name:Torque Command
Type:Float,Read/Write
Range: -PL[1] to PL[1]
Default:0 
****************************************************/
void rTC(uint16_t CAN_ID)    //读转矩
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 						 //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;          //数据帧
	TxMessage.IDE = CAN_ID_STD;            //标准帧
	TxMessage.DLC = 4;                     //数据长度：最大为8；
	TxMessage.Data[0] = 0x50;    
	TxMessage.Data[1] = 0x41;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x40;          
	CAN_Transmit(CAN2,&TxMessage); 				 //获取发送邮箱
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
			
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
	TxMessage.Data[0] = 0x54;    
	TxMessage.Data[1] = 0x43;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x80; //浮点数    
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
void rMO(void)   //读励磁
{
	CanTxMsg TxMessage;								
	TxMessage.StdId = 0x37f; 							//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 4;                    //数据长度：最大为8；
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x4f;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN1,&TxMessage); 				//获取发送邮箱
	while(CAN_TransmitStatus(CAN1, 0x00) != 0x01);
}


void wMO_FY(uint16_t CAN_ID,uint8_t CAN_DATA4)    //写励磁
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 							//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;           //数据帧
	TxMessage.IDE = CAN_ID_STD;             //标准帧
	TxMessage.DLC = 8;                      //数据长度：最大为8；
	TxMessage.Data[0] = 0x4d;    
	TxMessage.Data[1] = 0x4f;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;    
	TxMessage.Data[4] = CAN_DATA4;    
	TxMessage.Data[5] = 0x00;     
	TxMessage.Data[6] = 0x00;    
	TxMessage.Data[7] = 0x00;      
	CAN_Transmit(CAN2,&TxMessage); 				//获取发送邮箱
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
		
}
void wMO_FW(uint16_t CAN_ID,uint8_t CAN_DATA4)    //写励磁
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void rSP(void)   //PTP模式速度（点对点）  加速   匀速   减速
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x37f; 							//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 4;                    //数据长度：最大为8；
	TxMessage.Data[0] = 0x53;    
	TxMessage.Data[1] = 0x50;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wSP_FY(uint16_t CAN_ID,int32_t speed)    //PTP模式速度（点对点）  加速   匀速   减速
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	vel_FY = speed; 

	CAN_DATA4 = (vel_FY & 0x000000FF);
	CAN_DATA5 = (vel_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (vel_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (vel_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 					 //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;        //数据帧
	TxMessage.IDE = CAN_ID_STD;          //标准帧
	TxMessage.DLC = 8;                   //数据长度：最大为8；
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
void wSP_FW(uint16_t CAN_ID,int32_t speed)    //PTP模式速度（点对点）  加速   匀速   减速
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	vel_FW = speed; 

	CAN_DATA4 = (vel_FW & 0x000000FF);
	CAN_DATA5 = (vel_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (vel_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (vel_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA; 	      //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wAC_FY(uint16_t CAN_ID,int32_t acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	acc_FY = acceleration; 

	CAN_DATA4 = (acc_FY & 0x000000FF);
	CAN_DATA5 = (acc_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (acc_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (acc_FY & 0xFF000000)>>24;

			
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wAC_FW(uint16_t CAN_ID,int32_t acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	acc_FW = acceleration; 

	CAN_DATA4 = (acc_FW & 0x000000FF);
	CAN_DATA5 = (acc_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (acc_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (acc_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wDC_FY(uint16_t CAN_ID,int32_t acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	dcc_FY = acceleration; 

	CAN_DATA4 = (dcc_FY & 0x000000FF);
	CAN_DATA5 = (dcc_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (dcc_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (dcc_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wDC_FW(uint16_t CAN_ID,int32_t acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	dcc_FW = acceleration; 

	CAN_DATA4 = (dcc_FW & 0x000000FF);
	CAN_DATA5 = (dcc_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (dcc_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (dcc_FW & 0xFF000000)>>24;

	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wSD_FY(uint16_t CAN_ID,int32_t sd_acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sdac_FY = sd_acceleration; 

	CAN_DATA4 = (sdac_FY & 0x000000FF);
	CAN_DATA5 = (sdac_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (sdac_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (sdac_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 							//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;           //数据帧
	TxMessage.IDE = CAN_ID_STD;             //标准帧
	TxMessage.DLC = 8;                      //数据长度：最大为8；
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
void wSD_FW(uint16_t CAN_ID,int32_t sd_acceleration)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sdac_FW = sd_acceleration; 

	CAN_DATA4 = (sdac_FW & 0x000000FF);
	CAN_DATA5 = (sdac_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (sdac_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (sdac_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 							//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;           //数据帧
	TxMessage.IDE = CAN_ID_STD;             //标准帧
	TxMessage.DLC = 8;                      //数据长度：最大为8；
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
void wSF_FY(uint16_t CAN_ID,int32_t sf_value)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sf_FY = sf_value; 

	CAN_DATA4 = (sf_FY & 0x000000FF);
	CAN_DATA5 = (sf_FY & 0x0000FF00)>>8;
	CAN_DATA6 = (sf_FY & 0x00FF0000)>>16;
	CAN_DATA7 = (sf_FY & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;         //数据帧
	TxMessage.IDE = CAN_ID_STD;           //标准帧
	TxMessage.DLC = 8;                    //数据长度：最大为8；
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
void wSF_FW(uint16_t CAN_ID,int32_t sf_value)   //加速度
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	
	sf_FW = sf_value; 

	CAN_DATA4 = (sf_FW & 0x000000FF);
	CAN_DATA5 = (sf_FW & 0x0000FF00)>>8;
	CAN_DATA6 = (sf_FW & 0x00FF0000)>>16;
	CAN_DATA7 = (sf_FW & 0xFF000000)>>24;

    
	TxMessage.StdId = CAN_ID; 						 //标准标识符
	TxMessage.RTR = CAN_RTR_DATA;          //数据帧
	TxMessage.IDE = CAN_ID_STD;            //标准帧
	TxMessage.DLC = 8;                     //数据长度：最大为8；
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
void rHM(uint16_t CAN_ID,uint8_t INDEX)   //归零
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 4;                  //数据长度：最大为8；
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
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;                  //数据长度：最大为8；
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
void wTC(uint16_t CAN_ID,float cur)//-0.2  转矩命令
{
	CanTxMsg TxMessage;
	int8_t CAN_DATA4,CAN_DATA5,CAN_DATA6,CAN_DATA7;
	uint32_t CUR=0;
	CUR = f_to_i(cur);
	
	
	CAN_DATA4 = (CUR & 0x000000FF);
	CAN_DATA5 = (CUR & 0x0000FF00)>>8;
	CAN_DATA6 = (CUR & 0x00FF0000)>>16;
	CAN_DATA7 = (CUR & 0xFF000000)>>24;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;                  //数据长度：最大为8；
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
void wCL(uint16_t CAN_ID,uint8_t INDEX,float cur)     //电机持续限制及电机阻滞保护参数
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

void wPL(uint16_t CAN_ID,uint8_t INDEX,float cur)    //电机峰值电流和峰值延续时间
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
void wBG_FY(uint16_t CAN_ID)    //开始运动
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;     //数据帧
	TxMessage.IDE = CAN_ID_STD;       //标准帧
	TxMessage.DLC = 4;                //数据长度：最大为8；
	TxMessage.Data[0] = 0x42;    
	TxMessage.Data[1] = 0x47;    
	TxMessage.Data[2] = 0x00;    
	TxMessage.Data[3] = 0x00;          
	CAN_Transmit(CAN2,&TxMessage); 
	while(CAN_TransmitStatus(CAN2, 0x00) != 0x01);
}
void wBG_FW(uint16_t CAN_ID)    //开始运动
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;     //数据帧
	TxMessage.IDE = CAN_ID_STD;       //标准帧
	TxMessage.DLC = 4;                //数据长度：最大为8；
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
void wST_FY(uint16_t CAN_ID)    //停止运动
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
void wST_FW(uint16_t CAN_ID)    //停止运动
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 				//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;     //数据帧
	TxMessage.IDE = CAN_ID_STD;       //标准帧
	TxMessage.DLC = 4;                //数据长度：最大为8；
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
void rPX_FY(uint16_t CAN_ID)    //读取当前位置
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;                  //数据长度：最大为8；
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
void rPX_FW(uint16_t CAN_ID)    //读取当前位置
{
	CanTxMsg TxMessage;
	TxMessage.StdId = CAN_ID; 					//标准标识符
	TxMessage.RTR = CAN_RTR_DATA;       //数据帧
	TxMessage.IDE = CAN_ID_STD;         //标准帧
	TxMessage.DLC = 8;                  //数据长度：最大为8；
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
void rPE(uint16_t CAN_ID)   //位置误差
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
void rVX(uint16_t CAN_ID)      //读取当前速度  数值/秒
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
void rVE(uint16_t CAN_ID)   //速度误差  
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

void rAC(uint16_t CAN_ID)   //加速度  
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

void rIQ(uint16_t CAN_ID)   //电流  
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

void rIQ_FW(uint16_t CAN_ID)   //电流  
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

////////////////////////////////////////////2020.03.31添加///////////////////////////////////////////////////
void EC_FWRead(uint16_t CAN_ID)   //ELMO错误代码   CAN1
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


void MF_FWRead(uint16_t CAN_ID)   //电机错误代码   CAN1
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

void SR_FWRead(uint16_t CAN_ID)   //状态寄存器   CAN1
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

void AN_1_FWRead(uint16_t CAN_ID)   //实时电流A相   CAN1
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

void AN_2_FWRead(uint16_t CAN_ID)   //实时电流B相   CAN1
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

void AN_3_FWRead(uint16_t CAN_ID)   //实时电流C相   CAN1
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

void AN_4_FWRead(uint16_t CAN_ID)   //实时母线电压   CAN1
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

void EE_1_FWRead(uint16_t CAN_ID)   //编码器状态   CAN1
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

void TI_1_FWRead(uint16_t CAN_ID)   //驱动器温度   CAN1
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

void PX_FWRead(uint16_t CAN_ID)   //角度   CAN1
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

void VX_FWRead(uint16_t CAN_ID)   //速度   CAN1
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

void SO_FWRead(uint16_t CAN_ID)   //电机使能状态  CAN1
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

void PY_FWRead(uint16_t CAN_ID)   //角度   CAN1
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

void WD_FW_Read(uint16_t CAN_ID)   //温度
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

void WD_FY_Read(uint16_t CAN_ID)   //温度
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



////////////////////////////////////////////2020.03.31添加///////////////////////////////////////////////////
void EC_FYRead(uint16_t CAN_ID)   //ELMO错误代码   CAN1
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


void MF_FYRead(uint16_t CAN_ID)   //电机错误代码   CAN1
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

void SR_FYRead(uint16_t CAN_ID)   //状态寄存器   CAN1
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

void AN_1_FYRead(uint16_t CAN_ID)   //实时电流A相   CAN1
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

void AN_2_FYRead(uint16_t CAN_ID)   //实时电流B相   CAN1
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

void AN_3_FYRead(uint16_t CAN_ID)   //实时电流C相   CAN1
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

void AN_4_FYRead(uint16_t CAN_ID)   //实时母线电压   CAN1
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

void EE_1_FYRead(uint16_t CAN_ID)   //编码器状态   CAN1
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

void TI_1_FYRead(uint16_t CAN_ID)   //驱动器温度   CAN1
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

void PX_FYRead(uint16_t CAN_ID)   //角度   CAN1
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

void VX_FYRead(uint16_t CAN_ID)   //速度   CAN1
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

void SO_FYRead(uint16_t CAN_ID)   //电机使能状态  CAN1
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

void wAS_FY(uint16_t CAN_ID,float vol)    //写偏执电压 CAN2
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
