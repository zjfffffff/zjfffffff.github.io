#include "stm32f4xx.h"	
#include "ScanFunction.h"
#include "PointFunction.h"
#include "ioinit.h"

void LuJing(void);

float Scan_StepCount_ly = 0;  
float ScanAngleRang_ly = 10;                        //扫描范围
float FW_ScanStepLength_ly = 2.16;                      //扫描步长
float PanFangXiang = 0;
int Scan_Direction_ly = 0x01;                        //判断扫描方向
float FW_Scan_StartAngle_ly = 0;                    //方位扫描起始角度
float FW_Scan_StopAngle_ly = 0;                     //方位扫描结束角度
int Flag_Com_Direction_ly = 1;                    //方向只赋一次值
int FW_Direction_ly = 0;           //右+1  左-1   //方位方向
int FY_Direction_ly = 0;          //上+1   下-1   //俯仰方向
float FY_ScanAngleRang_ly = 1;                      //俯仰扫描范围
float FY_Scan_Zhixing_ly = 0;
int Time_Long_ly = 106;
float step_design_set_ly = 0;
u8 Start_cnt = 1;
u16 watch_Scan_cnt = 0;
float watch_Scan[1200]={0};
u8 Test_Scan_Time = 0;

extern float FW_encoder_degrees;  //俯仰编码器转换为角度
extern float FW_zero_degree_zheng;     //336.64
extern u32 Scan_cnt_ly;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
extern u8 ZhiXiangFlag;
extern float FW_location_kp;
extern float FW_pointing_degree_set;    //指向角设定值
extern float FY_pointing_degree_set;    //指向角设定值

float Scan_FW_Zero = 0;
float Scan_FY_Zero = 0;

u8 SCAN_POINT_FLAG = 0;
float KongJianZhiXiang_FWLY1 = 0;
u8 ZhiXiang_Complete_FlagLY1 = 0;
float KongJianZhiXiang_FYLY1 = 0;
extern unsigned int diaoyong_cnt;

extern u8 Give_Scan_Number;
u16 ScanFunction_Number = 0;

void Scan(void)
{
///////////////////////////////////////判断执行次数///////////////////////////////////	
	if(Give_Scan_Number != 0)
	{
		if(Give_Scan_Number == ScanFunction_Number)
		{
			ScanFunction_Number = 0;
			SCAN_POINT_FLAG = 2;
		}	
	}
	else
	{
		ScanFunction_Number = 0;
	}	
	
		
///////////////////////////////////////内同步前先指向到起始点//////////////////////////////////		
	if(SCAN_POINT_FLAG == 0)
	{
		ZhiXiangFlag = 1;		
		if(KongJianZhiXiang_FWLY1 > SpaceSetValueFW)                 //空间指向方位
		{				
			SpaceSetValueFW = SpaceSetValueFW + 0.001;
            ZhiXiang_Complete_FlagLY1 = 0;				
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWLY1)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWLY1)>-0.01))	
		//if(SpaceSetValueFW >= KongJianZhiXiang_FWLY1)
		{
			SpaceSetValueFW = KongJianZhiXiang_FWLY1;				
		}
			
		if(KongJianZhiXiang_FWLY1 < SpaceSetValueFW)
		{				
			SpaceSetValueFW = SpaceSetValueFW - 0.001;
            ZhiXiang_Complete_FlagLY1 = 0;					
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWLY1)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWLY1)>-0.01))	
		{
			SpaceSetValueFW = KongJianZhiXiang_FWLY1;
		}
		
		if(KongJianZhiXiang_FYLY1 > SpaceSetValueFY)          //空间指向俯仰
		{				
			SpaceSetValueFY = SpaceSetValueFY + 0.001;
			ZhiXiang_Complete_FlagLY1 = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYLY1)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYLY1)>-0.01))	
//		if(SpaceSetValueFY >= KongJianZhiXiang_FYLY1)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYLY1;
		}
			
		if(KongJianZhiXiang_FYLY1 < SpaceSetValueFY)
		{				
			SpaceSetValueFY = SpaceSetValueFY - 0.001;
			ZhiXiang_Complete_FlagLY1 = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYLY1)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYLY1)>-0.01))		
//		if(SpaceSetValueFY <= KongJianZhiXiang_FYLY1)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYLY1;
		}
                                        
        if((SpaceSetValueFY == KongJianZhiXiang_FYLY1) && (SpaceSetValueFW == KongJianZhiXiang_FWLY1)) //指向稳定后发一次脉冲
		{
			Scan_FW_Zero = SpaceSetValueFW;          //指向后此时的位置定为外同步的零点    方位
			Scan_FY_Zero = SpaceSetValueFY;          //指向后此时的位置定为外同步的零点    俯仰
			
			ZhiXiang_Complete_FlagLY1++;
			if(ZhiXiang_Complete_FlagLY1 == 1)
			{
				SCAN_POINT_FLAG = 1;
				GPIO_SetBits(GPIOA,GPIO_Pin_6);
				Delay_us(110);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			}
			else
			{
				ZhiXiang_Complete_FlagLY1 = 2;
			}
		}		
		SpacePoint();	
	}
///////////////////////////////////////////////////////////////////////////////////////////	
	if(SCAN_POINT_FLAG == 1)
	{
		Scan_cnt_ly++;
		LuJing();
		ZhiXiangFlag = 1;
	
//	FW_pointing_degree_set = step_design_set_ly;
//	FY_pointing_degree_set = 0;
	
//		Scan_FW_Zero = SpaceSetValueFW;          //指向后此时的位置定为外同步的零点    方位
//		Scan_FY_Zero = SpaceSetValueFY;          //指向后此时的位置定为外同步的零点    俯仰
	
	
		SpaceSetValueFW = Scan_FW_Zero + step_design_set_ly;
//	SpaceSetValueFY = 0;
		SpaceSetValueFY = Scan_FY_Zero + FY_Scan_Zhixing_ly;
//	FW_pointing_degree_set = step_design_set_ly;
//    FY_pointing_degree_set = 0;
	
//	SpaceSetValueFY = FY_Scan_Zhixing_ly;
		SpacePoint();
	
//	if(Scan_cnt_ly == 1 || Scan_cnt_ly == 107 || Scan_cnt_ly == 213 || Scan_cnt_ly == 319 || Scan_cnt_ly == 425 || Scan_cnt_ly == 531 || Scan_cnt_ly == 637 || Scan_cnt_ly == 743)
//	{
//		FW_location_kp = 120;		
//	}	
//	else if(Scan_cnt_ly == 66 || Scan_cnt_ly == 172 || Scan_cnt_ly == 278 || Scan_cnt_ly == 384 || Scan_cnt_ly == 490 || Scan_cnt_ly == 596 || Scan_cnt_ly == 702 || Scan_cnt_ly == 808)
//	{
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		Delay_us(110);
////      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
//		
//		FW_location_kp = 220;	
//	}
//	
//	else if(Scan_cnt_ly == 849 || Scan_cnt_ly == 955 || Scan_cnt_ly == 1061 || Scan_cnt_ly == 1167 || Scan_cnt_ly == 1273 || Scan_cnt_ly == 1379 || Scan_cnt_ly == 1485 || Scan_cnt_ly == 1591)
//	{
//		FW_location_kp = 120; 
//	}	
//	else if(Scan_cnt_ly == 914 || Scan_cnt_ly == 1020 || Scan_cnt_ly == 1126 || Scan_cnt_ly == 1232 || Scan_cnt_ly == 1338 || Scan_cnt_ly == 1444 || Scan_cnt_ly == 1550 || Scan_cnt_ly == 1656)
//	{
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		Delay_us(110);
////      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
//		FW_location_kp = 220; 
//	}
//	
//	else if(Scan_cnt_ly == 1697 || Scan_cnt_ly == 1803 || Scan_cnt_ly == 1909 || Scan_cnt_ly == 2015 || Scan_cnt_ly == 2121 || Scan_cnt_ly == 2227 || Scan_cnt_ly == 2333 || Scan_cnt_ly == 2439)
//	{
//		FW_location_kp = 120; 
//	}	
//	else if(Scan_cnt_ly == 1762 || Scan_cnt_ly == 1868 || Scan_cnt_ly == 1974 || Scan_cnt_ly == 2080 || Scan_cnt_ly == 2186 || Scan_cnt_ly == 2292 || Scan_cnt_ly == 2398 || Scan_cnt_ly == 2504)
//	{
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		Delay_us(110);
////      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
//		FW_location_kp = 220; 
//	}
//	
//	else if(Scan_cnt_ly == 2545 || Scan_cnt_ly == 2651 || Scan_cnt_ly == 2757 || Scan_cnt_ly == 2863 || Scan_cnt_ly == 2969)
//	{
//		FW_location_kp = 120; 
//	}	
//	else if(Scan_cnt_ly == 2610 || Scan_cnt_ly == 2716 || Scan_cnt_ly == 2822 || Scan_cnt_ly == 2928)
//	{
////		GPIO_SetBits(GPIOA,GPIO_Pin_6);
////		Delay_us(110);
////      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
//		FW_location_kp = 220; 
//	}
	
	
		if(Scan_cnt_ly == 1 || Scan_cnt_ly == 107 || Scan_cnt_ly == 213 || Scan_cnt_ly == 319 || Scan_cnt_ly == 425 || Scan_cnt_ly == 531 || Scan_cnt_ly == 637 || Scan_cnt_ly == 743)
		{
			FW_location_kp = 120;		
		}	
		else if(Scan_cnt_ly == 46 || Scan_cnt_ly == 152 || Scan_cnt_ly == 258 || Scan_cnt_ly == 364 || Scan_cnt_ly == 470 || Scan_cnt_ly == 576 || Scan_cnt_ly == 682 || Scan_cnt_ly == 788)
		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_6);
//		Delay_us(110);
//      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		
			FW_location_kp = 200;	
		}
	
		else if(Scan_cnt_ly == 849 || Scan_cnt_ly == 955 || Scan_cnt_ly == 1061 || Scan_cnt_ly == 1167 || Scan_cnt_ly == 1273 || Scan_cnt_ly == 1379 || Scan_cnt_ly == 1485 || Scan_cnt_ly == 1591)
		{
			FW_location_kp = 120; 
		}	
		else if(Scan_cnt_ly == 894 || Scan_cnt_ly == 1000 || Scan_cnt_ly == 1106 || Scan_cnt_ly == 1212 || Scan_cnt_ly == 1318 || Scan_cnt_ly == 1424 || Scan_cnt_ly == 1530 || Scan_cnt_ly == 1636)
		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_6);
//		Delay_us(110);
//      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			FW_location_kp = 200; 
		}
	
		else if(Scan_cnt_ly == 1697 || Scan_cnt_ly == 1803 || Scan_cnt_ly == 1909 || Scan_cnt_ly == 2015 || Scan_cnt_ly == 2121 || Scan_cnt_ly == 2227 || Scan_cnt_ly == 2333 || Scan_cnt_ly == 2439)
		{
			FW_location_kp = 120; 
		}	
		else if(Scan_cnt_ly == 1742 || Scan_cnt_ly == 1848 || Scan_cnt_ly == 1954 || Scan_cnt_ly == 2060 || Scan_cnt_ly == 2166 || Scan_cnt_ly == 2272 || Scan_cnt_ly == 2378 || Scan_cnt_ly == 2484)
		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_6);
//		Delay_us(110);
//      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			FW_location_kp = 200; 
		}
	
		else if(Scan_cnt_ly == 2545 || Scan_cnt_ly == 2651 || Scan_cnt_ly == 2757 || Scan_cnt_ly == 2863 || Scan_cnt_ly == 2969)
		{
			FW_location_kp = 120; 
		}	
		else if(Scan_cnt_ly == 2590 || Scan_cnt_ly == 2696 || Scan_cnt_ly == 2802 || Scan_cnt_ly == 2908)
		{
//		GPIO_SetBits(GPIOA,GPIO_Pin_6);
//		Delay_us(110);
//      GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			FW_location_kp = 200; 
		}

		Test_Scan_Time++;
//	if(Scan_cnt_ly == 107 || Scan_cnt_ly == 213 || Scan_cnt_ly == 319 || Scan_cnt_ly == 425 || Scan_cnt_ly == 531 || Scan_cnt_ly == 637 || Scan_cnt_ly == 743)
//	{
//		Test_Scan_Time = 0;
//	}
//	else if(Scan_cnt_ly == 849 || Scan_cnt_ly == 955 || Scan_cnt_ly == 1061 || Scan_cnt_ly == 1167 || Scan_cnt_ly == 1273 || Scan_cnt_ly == 1379 || Scan_cnt_ly == 1485 || Scan_cnt_ly == 1591)
//	{
//		Test_Scan_Time = 0;	
//	}
//	else if(Scan_cnt_ly == 1697 || Scan_cnt_ly == 1803 || Scan_cnt_ly == 1909 || Scan_cnt_ly == 2015 || Scan_cnt_ly == 2121 || Scan_cnt_ly == 2227 || Scan_cnt_ly == 2333 || Scan_cnt_ly == 2439)
//	{
//		Test_Scan_Time = 0;	
//	}
//	else if(Scan_cnt_ly == 2545 || Scan_cnt_ly == 2651 || Scan_cnt_ly == 2757 || Scan_cnt_ly == 2863 || Scan_cnt_ly == 2969)
//	{
//		Test_Scan_Time = 0; 
//	}
	
	
	
		if(Scan_cnt_ly == 109 || Scan_cnt_ly == 215 || Scan_cnt_ly == 321 || Scan_cnt_ly == 427 || Scan_cnt_ly == 533 || Scan_cnt_ly == 639 || Scan_cnt_ly == 745)
		{
			Test_Scan_Time = 0;
		}
		else if(Scan_cnt_ly == 851 || Scan_cnt_ly == 957 || Scan_cnt_ly == 1063 || Scan_cnt_ly == 1169 || Scan_cnt_ly == 1275 || Scan_cnt_ly == 1381 || Scan_cnt_ly == 1487 || Scan_cnt_ly == 1593)
		{
			Test_Scan_Time = 0;	
		}
		else if(Scan_cnt_ly == 1699 || Scan_cnt_ly == 1805 || Scan_cnt_ly == 1911 || Scan_cnt_ly == 2017 || Scan_cnt_ly == 2123 || Scan_cnt_ly == 2229 || Scan_cnt_ly == 2335 || Scan_cnt_ly == 2441)
		{
			Test_Scan_Time = 0;	
		}
		else if(Scan_cnt_ly == 2547 || Scan_cnt_ly == 2653 || Scan_cnt_ly == 2759 || Scan_cnt_ly == 2865 || Scan_cnt_ly == 2971)
		{
			Test_Scan_Time = 0; 
		}
	
	}
}




void LuJing(void)
{							
	Scan_StepCount_ly = ScanAngleRang_ly/FW_ScanStepLength_ly-1;	
	if(Scan_Direction_ly == 0x01)    //向右下扫描
	{
		FW_Scan_StopAngle_ly = FW_Scan_StartAngle_ly + ScanAngleRang_ly;
		if(Flag_Com_Direction_ly == 1)
		{
			FW_Direction_ly = 1; 
			FY_Direction_ly = -1;
		}
		Flag_Com_Direction_ly = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=FW_Scan_StopAngle_ly&&(FW_Direction_ly == 1))     //撞右
		if(PanFangXiang >= ScanAngleRang_ly &&(FW_Direction_ly == 1))     //撞右
		{
			FW_Direction_ly = -FW_Direction_ly;			
			FY_Scan_Zhixing_ly = (float)(-FY_ScanAngleRang_ly);
			Start_cnt = 0;
            Scan_cnt_ly = 0;			
		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<= FW_Scan_StartAngle_ly &&(FW_Direction_ly == -1))   //撞左
		if(PanFangXiang <= (-ScanAngleRang_ly) &&(FW_Direction_ly == -1))   //撞左
		{
			ScanFunction_Number++;                   //判断扫描次数
			FW_Direction_ly = -FW_Direction_ly;
			FY_Scan_Zhixing_ly = 0;
		    Scan_cnt_ly = 0;		
		}
		if(Start_cnt == 1)
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StartAngle_ly;
				PanFangXiang = step_design_set_ly;
								

//				watch_Scan[watch_Scan_cnt]=step_design_set_ly;
//				watch_Scan_cnt++;
//				if(watch_Scan_cnt == 1000)
//				{
//					watch_Scan_cnt = 999;
//				}
			}
		}
		else
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly - FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}
		}

		if(FW_Direction_ly == -1)
		{
			step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StopAngle_ly;
			PanFangXiang = step_design_set_ly;
		}						
	}

	if(Scan_Direction_ly == 0x02)    //向左下扫描
	{
		FW_Scan_StopAngle_ly = FW_Scan_StartAngle_ly + ScanAngleRang_ly;
		if(Flag_Com_Direction_ly == 1)
		{
			FW_Direction_ly = -1; 
			FY_Direction_ly = -1;
		}
		Flag_Com_Direction_ly = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=FW_Scan_StopAngle_ly&&(FW_Direction_ly == 1))     //撞右
		if(PanFangXiang >= ScanAngleRang_ly &&(FW_Direction_ly == 1))     //撞右
		{
			FW_Direction_ly = -FW_Direction_ly;			
			FY_Scan_Zhixing_ly = 0;

            Scan_cnt_ly = 0;
			ScanFunction_Number++;                   //判断扫描次数			
		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<= FW_Scan_StartAngle_ly &&(FW_Direction_ly == -1))   //撞左
		if(PanFangXiang <= (-ScanAngleRang_ly) &&(FW_Direction_ly == -1))   //撞左
		{
			FW_Direction_ly = -FW_Direction_ly;
			FY_Scan_Zhixing_ly = (float)(-FY_ScanAngleRang_ly);
		    Scan_cnt_ly = 0;
			Start_cnt = 0;			
		}
		if(Start_cnt == 1)
		{
			if(FW_Direction_ly == -1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StartAngle_ly;
				PanFangXiang = step_design_set_ly;	
			}
		}
		else
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly - FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}
			
			if(FW_Direction_ly == -1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}	
		}
					
	}

	if(Scan_Direction_ly == 0x03)    //向右上扫描
	{
		FW_Scan_StopAngle_ly = FW_Scan_StartAngle_ly + ScanAngleRang_ly;
		if(Flag_Com_Direction_ly == 1)
		{
			FW_Direction_ly = 1; 
			FY_Direction_ly = -1;
		}
		Flag_Com_Direction_ly = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=FW_Scan_StopAngle_ly&&(FW_Direction_ly == 1))     //撞右
		if(PanFangXiang >= ScanAngleRang_ly &&(FW_Direction_ly == 1))     //撞右
		{
			FW_Direction_ly = -FW_Direction_ly;			
			FY_Scan_Zhixing_ly = (float)FY_ScanAngleRang_ly;
			Start_cnt = 0;
            Scan_cnt_ly = 0;			
		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<= FW_Scan_StartAngle_ly &&(FW_Direction_ly == -1))   //撞左
		if(PanFangXiang <= (-ScanAngleRang_ly) &&(FW_Direction_ly == -1))   //撞左
		{
			ScanFunction_Number++;                   //判断扫描次数
			FW_Direction_ly = -FW_Direction_ly;
			FY_Scan_Zhixing_ly = 0;
		    Scan_cnt_ly = 0;		
		}
		if(Start_cnt == 1)
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StartAngle_ly;
				PanFangXiang = step_design_set_ly;
								

//				watch_Scan[watch_Scan_cnt]=step_design_set_ly;
//				watch_Scan_cnt++;
//				if(watch_Scan_cnt == 1000)
//				{
//					watch_Scan_cnt = 999;
//				}
			}
		}
		else
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly - FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}
		}

		if(FW_Direction_ly == -1)
		{
			step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StopAngle_ly;
			PanFangXiang = step_design_set_ly;
		}						
	}
	
	if(Scan_Direction_ly == 0x04)    //向左上扫描
	{
		FW_Scan_StopAngle_ly = FW_Scan_StartAngle_ly + ScanAngleRang_ly;
		if(Flag_Com_Direction_ly == 1)
		{
			FW_Direction_ly = -1; 
			FY_Direction_ly = -1;
		}
		Flag_Com_Direction_ly = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=FW_Scan_StopAngle_ly&&(FW_Direction_ly == 1))     //撞右
		if(PanFangXiang >= ScanAngleRang_ly &&(FW_Direction_ly == 1))     //撞右
		{
			FW_Direction_ly = -FW_Direction_ly;			
			FY_Scan_Zhixing_ly = 0;

            Scan_cnt_ly = 0;
			ScanFunction_Number++;                   //判断扫描次数			
		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<= FW_Scan_StartAngle_ly &&(FW_Direction_ly == -1))   //撞左
		if(PanFangXiang <= (-ScanAngleRang_ly) &&(FW_Direction_ly == -1))   //撞左
		{
			FW_Direction_ly = -FW_Direction_ly;
			FY_Scan_Zhixing_ly = (float)FY_ScanAngleRang_ly;
		    Scan_cnt_ly = 0;
			Start_cnt = 0;			
		}
		if(Start_cnt == 1)
		{
			if(FW_Direction_ly == -1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StartAngle_ly;
				PanFangXiang = step_design_set_ly;	
			}
		}
		else
		{
			if(FW_Direction_ly == 1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly - FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}
			
			if(FW_Direction_ly == -1)
			{
				step_design_set_ly = (Scan_cnt_ly / Time_Long_ly)*FW_ScanStepLength_ly * FW_Direction_ly + FW_Scan_StopAngle_ly;
				PanFangXiang = step_design_set_ly;
			}	
		}
					
	}
	
	
}


