/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "FSMC.h"
#include "aaa.h"
#include "can.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Ke_FW 0.009			//方位电机反电动势常数
#define Ke_FY 0.005625	//俯仰电机反电动势常数
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float K_ADDA=1;//变输出增益，AD量程±4.096V，DA量程±10V，必要时进行调整
float K_Sweep_Freq=2.5;//扫频增益，与伺服板统一
float DianLiu_FW = 0;
float DianLiu_FY = 0;


float WD_FY = 0;
float WD_FW = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
//void EXTI15_10_IRQHandler(void)
//{
//	if(EXTI_GetITStatus(EXTI_Line11) != RESET) 
//	{
//		Check_Comm_Status();//串口状态检查
//		Comm_data_Read();//读
//		
//		Float=AD1_float;
//		DA2_float=FW_Speed*0.078;
////		Encoder_Resolving();//编码器解算
//		
////		DA1_float=FW_Speed*Ke_FW*K_ADDA/K_Sweep_Freq;//扫方位
////		DA2_float=FY_Speed*Ke_FY*K_ADDA/K_Sweep_Freq;//扫俯仰
//		
////		Comm_data_Write();//写
//	}
//	EXTI_ClearITPendingBit(EXTI_Line11);//清除LINE4上的中断标志位  
//}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
long             RxCAN[9]={0};
extern uint16_t  CAN_ID;

extern uint8_t 	 hmf;
extern uint8_t   flagHmf;
extern uint8_t   hme;
extern int32_t   prf;
extern int32_t   pre;
extern int32_t   spf_fw;  //速度
extern int32_t   spf_fy;
extern int32_t   spe_fw;  
extern int32_t   spe_fy;
extern int32_t   dcf_fw;   //减速度
extern int32_t   dcf_fy;

extern int32_t   pxf_FW;  //当前位置
extern int32_t   pxe;
extern int32_t   pef;     //位置误差
extern int32_t   pee;
extern int32_t   vxf;     //当前速度
extern int32_t   vxe;
extern int32_t   vef;     //速度误差
extern int32_t   vee;

extern int32_t   acf_fw;     //加速度
extern int32_t   acf_fy;     //加速度
extern int32_t   ace;
extern int32_t   sdf_fw;
extern int32_t   sdf_fy;
extern float iqf;
extern float iqe;

extern u8 iqf_usart_1 ;
extern u8 iqf_usart_2 ;
extern u8 iqf_usart_3 ;
extern u8 iqf_usart_4 ;

extern u8 hm_open_flag;

extern int32_t   SO_fw;
extern int32_t   EE1_fw;
extern int32_t   EC_fw;
extern int32_t   MF_fw;
extern int32_t   SR_fw;
extern  float   AN_1_fw;
extern  float   AN_2_fw;
extern  float   AN_3_fw;
extern  float   AN_4_fw;
extern  float TI1_fw;

extern float   PX_fw;
extern float   PY_fw;
extern int32_t   VX_fw;



extern int32_t   EC_fy;
extern  int32_t   MF_fy;
extern  int32_t   SR_fy;
extern  float   AN_1_fy;
extern  float   AN_2_fy;
extern  float   AN_3_fy;
extern  float   AN_4_fy;
extern  float   TI1_fy;
extern  float   PX_fy;
extern  float   PY_fy;
extern  int32_t   EE1_fy;
extern  int32_t   VX_fy;
extern  int32_t   SO_fy;

u32 PA_cnt = 0;
u32 BG_cnt = 0;


union float_to_char
{
    float val;
	#pragma anon_unions
	struct
	{
	     unsigned char lowlow;
		 unsigned char lowhi;
		 unsigned char hilow;
         unsigned char hihi;		
	};
};

union float_to_char tc;

//CAN中断	
void CAN2_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!=RESET)
	{
	CanRxMsg RxMessage;
	CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);
	CAN_ID=RxMessage.StdId;
	RxCAN[0]=CAN_ID;
	RxCAN[1]=RxMessage.Data[0];
	RxCAN[2]=RxMessage.Data[1];
	RxCAN[3]=RxMessage.Data[2];
	RxCAN[4]=RxMessage.Data[3];
	RxCAN[5]=RxMessage.Data[4];
	RxCAN[6]=RxMessage.Data[5];
	RxCAN[7]=RxMessage.Data[6];
	RxCAN[8]=RxMessage.Data[7];
	
	//AC加速度
	if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x43)			
	{
		acf_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
	}
	//DC减速度			
	if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x44 && RxCAN[2] == 0x43)  			
	{		
		dcf_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);;  
	}
    //SP设定位置环速度（点对点）
	if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x50)			
	{
		spf_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
	}	
  	//IQ电流
	if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x49 && RxCAN[2] == 0x51)			
	{
					    tc.lowlow = RxCAN[5];
				        tc.lowhi = RxCAN[6];
				        tc.hilow = RxCAN[7];
				        tc.hihi = RxCAN[8]; 
				        iqf = tc.val;
				   
				        iqf_usart_1 = RxCAN[8];
				        iqf_usart_2 = RxCAN[7];
				        iqf_usart_3 = RxCAN[6];
				        iqf_usart_4 = RxCAN[5];
				
				        DianLiu_FY = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);
		
							
	}
	//TI温度
	if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x54 && RxCAN[2] == 0x49)			
	{
					    tc.lowlow = RxCAN[5];
				        tc.lowhi = RxCAN[6];
				        tc.hilow = RxCAN[7];
				        tc.hihi = RxCAN[8]; 
				        iqf = tc.val;
				   
				        iqf_usart_1 = RxCAN[8];
				        iqf_usart_2 = RxCAN[7];
				        iqf_usart_3 = RxCAN[6];
				        iqf_usart_4 = RxCAN[5];
				
				        WD_FY = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
				
	}
	
////////////////////////////////////2020.03.31添加////////////////////////////////////////////////////////////////			
//EC  ELMO错误代码
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x45 && RxCAN[2] == 0x43)			
			{
				EC_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
//MF  电机错误代码
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x4d && RxCAN[2] == 0x46)			
			{
				MF_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}			
//SR  状态寄存器
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x52)			
			{
				SR_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}				

//AN[3]  实时电流A相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x03)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_1_fy = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
//				AN_1_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}

//AN[4]  实时电流B相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x04)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_2_fy = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}		

//AN[5]  实时电流B相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x05)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_3_fy = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}	
//AN[6]  实时母线电压
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x06)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_4_fy = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}	
	
//EE[1]  编码器状态
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x45 && RxCAN[2] == 0x45 && RxCAN[3] == 0x01)			
			{
				EE1_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);					
			}
//TI[1]  驱动器温度		
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x54 && RxCAN[2] == 0x49 && RxCAN[3] == 0x01)			
			{
				
				TI1_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);						
			}
//PX  角度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x58)			
			{
				PX_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
                PX_fy = PX_fy / 186413.0;		
			}
//PY  角度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x59)			
			{
				PY_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
				PY_fy = PY_fw / 186413.0;
			}		
//VX 速度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x56 && RxCAN[2] == 0x58)			
			{
				VX_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}	
//SO 电机使能状态
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x4F)			
			{
				SO_fy = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}	

		}
			CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0); 
}
void CAN1_RX0_IRQHandler(void)
{
		if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
			CanRxMsg RxMessage;
			CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);		//读取数据
			CAN_ID=RxMessage.StdId;
			RxCAN[0]=CAN_ID;
			RxCAN[1]=RxMessage.Data[0];
			RxCAN[2]=RxMessage.Data[1];
			RxCAN[3]=RxMessage.Data[2];
			RxCAN[4]=RxMessage.Data[3];
			RxCAN[5]=RxMessage.Data[4];
			RxCAN[6]=RxMessage.Data[5];
			RxCAN[7]=RxMessage.Data[6];
			RxCAN[8]=RxMessage.Data[7];
	        //pp[13] = 7f  发送37f 返回2FF
	        //pp[13] = 7e  发送37e 返回2FE
//	        GPIO_SetBits(GPIOB,GPIO_Pin_2);//GPIOF9,F10设置高，灯灭


//SD急停速度			
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x53 && RxCAN[2] == 0x44)  			
			{		
				sdf_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]); ;	   
			}
//AC加速度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x43)			
			{
				acf_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
//DC减速度			
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x44 && RxCAN[2] == 0x43)  			
			{		
				dcf_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);;  
			}
//SP设定位置环速度（点对点）
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x50)			
			{
				spf_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
////////////////////////////////////2020.03.31添加////////////////////////////////////////////////////////////////			
//EC  ELMO错误代码
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x45 && RxCAN[2] == 0x43)			
			{
				EC_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
//MF  电机错误代码
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x4d && RxCAN[2] == 0x46)			
			{
				MF_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}			
//SR  状态寄存器
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x52)			
			{
				SR_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}				

//AN[3]  实时电流A相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x03)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_1_fw = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
//				AN_1_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}

//AN[4]  实时电流B相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x04)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_2_fw = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}		

//AN[5]  实时电流B相
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x05)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_3_fw = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}	
//AN[6]  实时母线电压
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x41 && RxCAN[2] == 0x4E && RxCAN[3] == 0x06)			
			{
				iqf_usart_1 = RxCAN[8];
				iqf_usart_2 = RxCAN[7];
				iqf_usart_3 = RxCAN[6];
				iqf_usart_4 = RxCAN[5];
			
				AN_4_fw = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);					
			}	
	
//EE[1]  编码器状态
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x45 && RxCAN[2] == 0x45 && RxCAN[3] == 0x01)			
			{
				EE1_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);					
			}
//TI[1]  驱动器温度		
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x54 && RxCAN[2] == 0x49 && RxCAN[3] == 0x01)			
			{
				
				TI1_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);						
			}
//PX  角度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x58)			
			{
				PX_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
                PX_fw = PX_fw / 186413.6;		//186413.6
			}
//PY  角度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x59)			
			{
				PY_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
				PY_fw = PY_fw / 186413.0;
			}		
//VX 速度
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x56 && RxCAN[2] == 0x58)			
			{
				VX_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}	
//SO 电机使能状态
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x53 && RxCAN[2] == 0x4F)			
			{
				SO_fw = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}	
////////////////////////////////////////////////////////////////////////////////////////////////////				
			
//BG开始运动
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x42 && RxCAN[2] == 0x47)			
			{		
				        BG_cnt++;
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x42 && RxCAN[2] == 0x47)			
			{
					    ;
			}			
//HM寻零
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x48 && RxCAN[2] == 0x4d )		
			{
						if(hm_open_flag == 1)
						{
						   hmf = RxCAN[5];	  
						   flagHmf = 0;
						}
				        
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x48 && RxCAN[2] == 0x4d )		
			{
						hme = RxCAN[5];	  
			}
			
//PR设定走相对位置
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x52)			
			{
						prf = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x50 && RxCAN[2] == 0x52)			
			{
						pre = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}			
			
//PA设定走绝对位置
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x41)			
			{
				        PA_cnt++;
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x50 && RxCAN[2] == 0x41)			
			{
						;
			}
			
//PX写当前编码器值
			if(RxCAN[0] == 0x2Ff && RxCAN[1] == 0x50 && RxCAN[2] == 0x58)			
			{
						pxf_FW = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x50 && RxCAN[2] == 0x58)			
			{
						pxe = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}			
//PE读取位置误差		
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x50 && RxCAN[2] == 0x45)			
			{
						pef = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);	
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x50 && RxCAN[2] == 0x45)			
			{
						pee = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
		


//VX读取主反馈速度（当前速度）
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x56 && RxCAN[2] == 0x58)			
			{
						vxf = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x56 && RxCAN[2] == 0x45)			
			{
						vxe = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
			
//VE读取速度误差
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x56 && RxCAN[2] == 0x45)			
			{
						vef = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x56 && RxCAN[2] == 0x45)			
			{
						vee = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}			

//IQ电流
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x49 && RxCAN[2] == 0x51)			
			{
					    tc.lowlow = RxCAN[5];
				        tc.lowhi = RxCAN[6];
				        tc.hilow = RxCAN[7];
				        tc.hihi = RxCAN[8]; 
				        iqf = tc.val;
				   
				        iqf_usart_1 = RxCAN[8];
				        iqf_usart_2 = RxCAN[7];
				        iqf_usart_3 = RxCAN[6];
				        iqf_usart_4 = RxCAN[5];
				
				
				        DianLiu_FW = AryToFloat(iqf_usart_4,iqf_usart_3,iqf_usart_2,iqf_usart_1);
							
			}
			if(RxCAN[0] == 0x2FE && RxCAN[1] == 0x49 && RxCAN[2] == 0x51)			
			{
						iqe = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);
			}
//		    GPIO_ResetBits(GPIOB,GPIO_Pin_2);//GPIOF9,F10设置高，灯灭
//TI温度			
			if(RxCAN[0] == 0x2FF && RxCAN[1] == 0x54 && RxCAN[2] == 0x49)			
			{
					    tc.lowlow = RxCAN[5];
				        tc.lowhi = RxCAN[6];
				        tc.hilow = RxCAN[7];
				        tc.hihi = RxCAN[8]; 
				        iqf = tc.val;
				   
				        iqf_usart_1 = RxCAN[8];
				        iqf_usart_2 = RxCAN[7];
				        iqf_usart_3 = RxCAN[6];
				        iqf_usart_4 = RxCAN[5];
				
				        WD_FW = (RxCAN[8]<<24) + (RxCAN[7]<<16) + (RxCAN[6]<<8) + (RxCAN[5]);						
			}
		}
			CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0); 
}


