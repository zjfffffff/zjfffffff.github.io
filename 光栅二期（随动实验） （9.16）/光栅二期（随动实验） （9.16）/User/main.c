#include "FSMC.h"
#include "encode.h"
#include "timer.h"
#include "extiz.h"
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"
#include "ioinit.h"
#include "can.h"
#include "elmo.h"
#include "math.h"
#include "stmflash.h"
#include "aaa.h"
#include "MotorOutputPosse.h"
#include "ELMOTEST.h"
#include "SaoMiao.h"
#include "Comm1_Communication.h"
#include "temperature.h"
#include "stm32f4xx_it.h"
#include "usart.h"	
#include "Receive_Send_Data.h"

extern u8 Send_gai_Status;					//粗跟踪模式

void Read_Flash();
int32_t vel_FY = 0;
int32_t acc_FY = 0;
int32_t pos_FY = 0;
int32_t spe_FY = 0;
int32_t sdac_FY = 0;
int32_t dcc_FY = 0;
int32_t sf_FY = 0;
int32_t vel_FW = 0;
int32_t acc_FW = 0;
int32_t pos_FW = 0;
int32_t spe_FW = 0;
int32_t sdac_FW = 0;
int32_t dcc_FW = 0;
int32_t sf_FW = 0;
uint16_t  CAN_ID =0;
uint8_t   hmf =0;
uint8_t   flagHmf =0;
uint8_t   hme =0;
int32_t   prf =0;
int32_t   pre =0;
int32_t   spf_fw = 0;
int32_t   spf_fy = 0;
int32_t   spe_fw = 0;
int32_t   spe_fy = 0;
int32_t   pxf_FW =0;
int32_t   pxe =0;
int32_t   pef =0;
int32_t   pee =0;
int32_t   vxf =0;
int32_t   vxe =0;
int32_t   vef =0;
int32_t   vee =0;
int32_t   acf_fw =0;
int32_t   acf_fy =0;
int32_t   ace =0;
int32_t   sdf_fw = 0;
int32_t   sdf_fy = 0;
int32_t   dcf_fw = 0;
int32_t   dcf_fy = 0;
float iqf = 0;
float iqe = 0;
u8 iqf_usart_1 = 0;
u8 iqf_usart_2 = 0;
u8 iqf_usart_3 = 0;
u8 iqf_usart_4 = 0;
u8 hm_flag = 0;
u8 hm_open_flag = 0;
float DA_JIAOZHENG = 0;
float DA_IN = 0;
float DA_RANGE = 10.0;
int16_t DA_out = 0;
float DA_offset = 0;
float DA_k = 1.00385;
float FW_scan_degree = 0;
float FY_scan_degree = 0;
int FW_scan_degree_out_int = 0;
int FY_scan_degree_out_int = 0;
u8 scan_cnt = 0;
u8 scan_cnt_derection = 0;
extern float FW_encoder_degrees_First;
extern float FY_encoder_degrees_First;
float FY_power_on_degrees = 0;
float FW_power_on_degrees = 0.0;
u8 FW_zero_flag = 0;
extern float FY_encoder_degrees;
extern float FW_encoder_degrees;
extern u32 Time; 
u8 PZdataRX[30];
extern float FW_zero_degree_zheng;
extern float FY_zero_degree; 
extern float FY_light_loop_revise;
extern float FW_light_loop_revise;
extern u8 system_mode,AB_TXData[]; 
extern int Time_Long;
extern int	FW_ScanStepLength;
extern int	FW_Scan_StartAngle;
extern int ScanAngleRang;
int32_t   EC_fw = 0;
int32_t   MF_fw = 0;
int32_t   SR_fw = 0;
float   AN_1_fw = 0;
float   AN_2_fw = 0;
float   AN_3_fw = 0;
float   AN_4_fw = 0;
float   TI1_fw = 0;
float   PX_fw = 0;
float   PY_fw = 0;
int32_t   EE1_fw = 0;
int32_t   VX_fw = 0;
int32_t   SO_fw = 0;
int32_t   EC_fy = 0;
int32_t   MF_fy = 0;
int32_t   SR_fy = 0;
float   AN_1_fy = 0;
float   AN_2_fy = 0;
float   AN_3_fy = 0;
float   AN_4_fy = 0;
float   TI1_fy = 0;
float   PX_fy = 0;
float   PY_fy = 0;
int32_t   EE1_fy = 0;
int32_t   VX_fy = 0;
int32_t   SO_fy = 0;
float NewAngle=14.9959;
extern u8	EXTI2_FLAG;	
float chang_value = 0;
float x_1=0;
float x_2=0;
u8 count_1 = 1;
int16_t FW_power_on_degrees_n=0;
u8 count_111 = 0;

extern float miss_distance_X_float ;
extern float miss_distance_Y_float ;
extern float miss_distance_X_float_sum ;
extern float miss_distance_Y_float_sum ;
extern float z_axis_velocity_1msdistance_sum ;
extern float x_axis_velocity_1msdistance_sum ;
extern double FW_Degree_GS;//水平方向
extern double FY_Degree_GS;//垂直方向
float Lici_flag_GS=1;
int main(void)
{
	
	SystemInit();
	Delay_ms(160);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	IO_Init();
	uart4_init(115200);
	uart2_init(115200);
	uart3_init(115200);
//	GPIO_SetBits(GPIOC,GPIO_Pin_10);
//	GPIO_ResetBits(GPIOC,GPIO_Pin_10);
	
//	Delay_ms(3000);				//等待系统稳定后再与elmo通信
//	CAN1_Mode_Init1(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化环回模式,波特率500Kbps
//	Delay_ms(1000);
//	CAN2_Mode_Init2(CAN_SJW_1tq,CAN_BS2_2tq,CAN_BS1_3tq,14,CAN_Mode_Normal);
	Delay_ms(1000);	
//	NVIC_config();
	CAN1_Mode_Init();
	Delay_ms(1000);		
	CAN2_Mode_Init();
	AskfsmFilterPara();
	AskfsmFilterPara_servo();
	bsp_InitFsmc();
	Delay_ms(500);
	Delay_ms(500);
//	Delay_ms(500);
//	DA12_out(0,0);	
//	Delay_ms(500);
//	Delay_ms(500);
//	Delay_ms(500);
//	Delay_ms(500);
//	Delay_ms(500);
	wRS_FW();//
//    Delay_ms(100);	
	wRS_FY();
//  Delay_ms(100);
//	wSD_FW(0x37E,1864130000);
//	Delay_ms(100);
//	wSD_FY(0x37F,1864130000);
//    Delay_ms(100);	
//	wSD_FW(0x37E,1864130000);
//	Delay_ms(100);
//	wSD_FY(0x37F,1864130000);
//	Delay_ms(100);
//	wSF_FW(0x37E,5);
//	Delay_ms(100);
//	wSF_FY(0x37F,5);
//	Delay_ms(100);
//	rPX_FW(0x37E);//0x37E
//	Delay_ms(100);	
//	PX_FWRead(0x37f);

//	  wAS_FY(0x37f,0); 
//	  wMO_FW(0x37F,1);
	  Delay_ms(300);
//	  wMO_FY(0x37F,1);
//	  Delay_ms(300);
//	 DA12_out(0,9999);
//     Delay_ms(300);
//	  Time = 0;
//	  Delay_ms(300); 			
	  PX_FWRead(0x37F);
		 PX_FYRead(0x37F);
	  Delay_ms(100);
		
	  FW_power_on_degrees = pxf_FW/186413.6 ;
	  FW_encoder_degrees_First = FW_power_on_degrees;		
		
		FY_power_on_degrees = PX_fy;
	  FY_encoder_degrees_First = FY_power_on_degrees;		
	  EXTIX2_Init();
	  TIM10_Init(19,8399);//20*8400/84000000=0.002
//	  Delay_ms(50);
		Send_gai_Status = 0x00;  //待机模式
	  system_mode = 3;
 	while(1)
	{
		if(Lici_flag_GS==1)
		{
		wMO_FW(0x37f,1);//写励磁
		wMO_FY(0x37f,1);
		}
//		Delay_ms(10); 			
//		GPIO_ResetBits(GPIOC,GPIO_Pin_10);

//		DA12_out(9999,x_1);	  //俯仰电机
//		DA12_out(x_2,9999);		//方位电机
//		DA12_out(chang_value,9999);
//	    rIQ(0x37f);
//		rIQ_FW(0x37f);
//		WD_FW_Read(0x37f);
//		WD_FY_Read(0x37f);	
//		EC_FWRead(0x37f);
//		MF_FWRead(0x37f);
//		SR_FWRead(0x37f);
//		AN_1_FWRead(0x37f);
//		AN_2_FWRead(0x37f);
//		AN_3_FWRead(0x37f);
//        AN_4_FWRead(0x37f);
//		EE_1_FWRead(0x37f);
//		TI_1_FWRead(0x37f);
//		PX_FWRead(0x37f);
//		VX_FWRead(0x37f);
//		SO_FWRead(0x37f);
//		EC_FYRead(0x37f);
//		MF_FYRead(0x37f);
//		SR_FYRead(0x37f);
//		AN_1_FYRead(0x37f);
//		AN_2_FYRead(0x37f);
//		AN_3_FYRead(0x37f);
//    AN_4_FYRead(0x37f);
//		EE_1_FYRead(0x37f);
//		TI_1_FYRead(0x37f);
//		PX_FYRead(0x37f);
//		VX_FYRead(0x37f);
//		SO_FYRead(0x37f);
//		Delay_ms(1);
//		Elmo_Send();
	}
}

float AryToFloat_1(char c1, char c2, char c3, char c4)
{    
	union FloatToArray fta;
	fta.ary[0] = c1;    
	fta.ary[1] = c2;    
	fta.ary[2] = c3;    
	fta.ary[3] = c4;    
	return fta.f;
}



void Read_Flash()
{
	union UintToArray T;
	FW_zero_degree_zheng = AryToFloat_1(PZdataRX[0],PZdataRX[1],PZdataRX[2],PZdataRX[3]);
	FY_zero_degree = AryToFloat_1(PZdataRX[4],PZdataRX[5],PZdataRX[6],PZdataRX[7]);
	NewAngle=AryToFloat_1(PZdataRX[8],PZdataRX[9],PZdataRX[10],PZdataRX[11]);
	Time_Long = PZdataRX[16]+PZdataRX[17]*256;
	FW_ScanStepLength = PZdataRX[18]+PZdataRX[19]*256;
	FW_Scan_StartAngle = PZdataRX[26] - FW_zero_degree_zheng;
	ScanAngleRang = PZdataRX[27] - PZdataRX[26];
}

