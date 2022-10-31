#include "temperature.h"
#include "Comm1_Communication.h"
#include "PC_Communication.h"
#include "stm32f4xx.h"
#include "Camera_Config.h"
#include "AHRS_100A.h"
#include "timer.h"
#include "PC_Communication.h"
#include "ioinit.h"
#include "ServoFunction.h"


extern u8 biao_zhi;
extern u16 TuoBa_fLAG ;

u8 biao_i=0;
u8 biao_y=0;

int temperature_liyanjun=0;
float Current_liyanjun=0;


//void temperature_write()
//{
//	FW_temperature_Comm1_Data_Pack();
//}

//int temperature_du()
//{
//		temperature_write();
//		Delay_ms(2);

////		if((TuoBa_fLAG & 0x8000) == 0x8000)
////		{
////				temperature_liyanjun=temperature_Comm1_Com_Read();
////		}
////	
//		
//		return temperature_liyanjun;
//}

//void Current_write()
//{
//	FW_Current_Comm1_Data_Pack();
//}	

//float Current_du() 
//{
//		float Current_liyanjun=0;
//		
//		FW_Current_Comm1_Data_Pack();
//		Delay_ms(2);


////		if((TuoBa_fLAG & 0x8000) == 0x8000)
////		{
////				Current_liyanjun=Current_Comm1_Com_Read();
////		}

//		return Current_liyanjun; 
//}




//int Current_du()
//{
////		int temperature_liyanjun=0;
//		if(biao_zhi==1)
//		{
//				Current_Comm1_Data_Pack();
//				biao_zhi=0;
//		}
//		if((TuoBa_fLAG & 0x8000) == 0x8000)
//		{
//				temperature_liyanjun=Current_Comm1_Com_Read();
//		}
//		return temperature_liyanjun;
//}