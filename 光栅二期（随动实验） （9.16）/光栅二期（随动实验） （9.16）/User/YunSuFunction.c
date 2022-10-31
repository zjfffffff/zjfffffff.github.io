#include "stm32f4xx.h"	
#include "YunSuFunction.h"
#include "PointFunction.h"
#include "ioinit.h"

void YunSu_LuJing(void);


int YunSu_Direction_ly = 0x01;    
float FW_YunSu_StopAngle_ly = 0;                     //方位扫描结束角度
float FW_YunSu_StartAngle_ly = 0;                    //方位扫描起始角度
float YunSuAngleRang_ly = 25;  
int Flag_YunSu_Direction_ly = 1;                    //方向只赋一次值
int FW_YunSu_Direction_ly = 0;           //右+1  左-1   //方位方向
int FY_YunSu_Direction_ly = 0;          //上+1   下-1   //俯仰方向
float YunSu_PanFangXiang = 0;
float FY_YunSuAngleRang_ly = 1;                      //俯仰扫描范围
float FY_YunSu_Zhixing_ly = 0;
u8 YunSu_Start_cnt = 1;
int YunSu_Time_Long_ly = 1;
float FW_YunSuStepLength_ly = 0.02;                      //扫描步长
float YunSu_step_design_set_ly = 0;
int huanXiang_flag = 0;

int X1_ly_cnt = 0;
u16 X2_ly_cnt = 0;
float X1_ly[1500] = {0};
float X2_ly[1500] = {0};
extern u32 YunSu_cnt_ly;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
extern u8 ZhiXiangFlag;
extern float miss_distance_X_float;
float SpaceSetValueFY_Set = -0.05;
u8 YunSu_POINT_FLAG = 0;
float KongJianZhiXiang_FWYunSu = 0;
float KongJianZhiXiang_FYYunSu = 0;
u8 ZhiXiang_Complete_FlagYunSu = 0;
float YunSu_FW_Zero = 0;
float YunSu_FY_Zero = 0;
extern unsigned int diaoyong_cnt;
extern u8 Give_Scan_Number;
u16 YunSuFunction_Number = 0;

void YunSuScan(void)
{
///////////////////////////////////////判断执行次数///////////////////////////////////	
	if(Give_Scan_Number != 0)
	{
		if(Give_Scan_Number == YunSuFunction_Number)
		{
			YunSuFunction_Number = 0;
			YunSu_POINT_FLAG = 2;
		}	
	}
	else
	{
		YunSuFunction_Number = 0;
	}	
	
	
	
///////////////////////////////////////匀速前先指向到起始点//////////////////////////////////		
	if(YunSu_POINT_FLAG == 0)
	{
		ZhiXiangFlag = 1;		
		if(KongJianZhiXiang_FWYunSu > SpaceSetValueFW)                 //空间指向方位
		{				
			SpaceSetValueFW = SpaceSetValueFW + 0.001;
            ZhiXiang_Complete_FlagYunSu = 0;				
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWYunSu)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWYunSu)>-0.01))	
		//if(SpaceSetValueFW >= KongJianZhiXiang_FWLY1)
		{
			SpaceSetValueFW = KongJianZhiXiang_FWYunSu;				
		}
			
		if(KongJianZhiXiang_FWYunSu < SpaceSetValueFW)
		{				
			SpaceSetValueFW = SpaceSetValueFW - 0.001;
            ZhiXiang_Complete_FlagYunSu = 0;					
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWYunSu)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWYunSu)>-0.01))	
		{
			SpaceSetValueFW = KongJianZhiXiang_FWYunSu;
		}
		
		if(KongJianZhiXiang_FYYunSu > SpaceSetValueFY)          //空间指向俯仰
		{				
			SpaceSetValueFY = SpaceSetValueFY + 0.001;
			ZhiXiang_Complete_FlagYunSu = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYYunSu)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYYunSu)>-0.01))	
//		if(SpaceSetValueFY >= KongJianZhiXiang_FYLY1)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYYunSu;
		}
			
		if(KongJianZhiXiang_FYYunSu < SpaceSetValueFY)
		{				
			SpaceSetValueFY = SpaceSetValueFY - 0.001;
			ZhiXiang_Complete_FlagYunSu = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYYunSu)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYYunSu)>-0.01))		
//		if(SpaceSetValueFY <= KongJianZhiXiang_FYLY1)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYYunSu;
		}
                                        
        if((SpaceSetValueFY == KongJianZhiXiang_FYYunSu) && (SpaceSetValueFW == KongJianZhiXiang_FWYunSu)) //指向稳定后发一次脉冲
		{
			YunSu_FW_Zero = SpaceSetValueFW;          //指向后此时的位置定为外同步的零点    方位
			YunSu_FY_Zero = SpaceSetValueFY;          //指向后此时的位置定为外同步的零点    俯仰
			
			ZhiXiang_Complete_FlagYunSu++;
			if(ZhiXiang_Complete_FlagYunSu == 1)
			{
				YunSu_POINT_FLAG = 1;
				GPIO_SetBits(GPIOA,GPIO_Pin_6);
				Delay_us(110);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
			}
			else
			{
				ZhiXiang_Complete_FlagYunSu = 2;
			}
		}		
		SpacePoint();
	}
///////////////////////////////////////////////////////////////////////////////////////////			
	if(YunSu_POINT_FLAG == 1)
	{	
		YunSu_cnt_ly++;
		YunSu_LuJing();
		ZhiXiangFlag = 1;
		SpaceSetValueFW = YunSu_FW_Zero + YunSu_step_design_set_ly;
		SpaceSetValueFY = YunSu_FY_Zero + FY_YunSu_Zhixing_ly;
//		SpaceSetValueFW = YunSu_step_design_set_ly;
//		if( (YunSu_PanFangXiang <= 12) && (YunSu_PanFangXiang >= -12) )
//		{
//			SpaceSetValueFY = -0.19;
//		}
//		else
//		{
//			SpaceSetValueFY = SpaceSetValueFY_Set;
//		}
//	SpaceSetValueFY = -0.19;
		SpacePoint();
	}
}




void YunSu_LuJing(void)
{							
	if(YunSu_Direction_ly == 0x01)    //向右下扫描
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = 1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //撞右
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = (float)(-FY_YunSuAngleRang_ly);
			YunSu_Start_cnt = 0;
            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //撞左
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;
			FY_YunSu_Zhixing_ly = 0;
		    YunSu_cnt_ly = 0;
            huanXiang_flag = 2;
			X1_ly_cnt = 0;	
			YunSuFunction_Number++;
//			YunSu_PanFangXiang = (-YunSuAngleRang_ly);
//            FY_YunSu_Zhixing_ly = 0;			
		}
		
		if(YunSu_Start_cnt == 1)
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StartAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;			
			}
		}
		else
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly - FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY + 0.00015;
			}
		}

		if(FW_YunSu_Direction_ly == -1)
		{
			YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StopAngle_ly;
			YunSu_PanFangXiang = YunSu_step_design_set_ly;
//			SpaceSetValueFY = SpaceSetValueFY - 0.00015;
		}
		

		
//					if(Give_Scan_Number != 0)
//					{
//						if(Scan_Number == Give_Scan_Number)
//						{
//							Scan_Number = 0;
//							system_mode = 1;
//						}	
//					}
//					else
//					{
//							Scan_Number = 0;
//					}	
					
	}

	if(YunSu_Direction_ly == 0x02)    //向左下扫描
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = -1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //撞右
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = 0;

            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
			YunSuFunction_Number++;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //撞左
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;
			FY_YunSu_Zhixing_ly = (float)(-FY_YunSuAngleRang_ly);

		    YunSu_cnt_ly = 0;
            huanXiang_flag = 2;
			X1_ly_cnt = 0;	
			YunSu_Start_cnt = 0;
//			YunSu_PanFangXiang = (-YunSuAngleRang_ly);
//            FY_YunSu_Zhixing_ly = 0;			
		}
		
		if(YunSu_Start_cnt == 1)
		{
			if(FW_YunSu_Direction_ly == -1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StartAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;			
			}
		}
		else
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly - FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY + 0.00015;
			}
			if(FW_YunSu_Direction_ly == -1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY - 0.00015;
			}	
		}
					
	}
	
	if(YunSu_Direction_ly == 0x03)    //向右上扫描
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = 1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //撞右
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = (float)FY_YunSuAngleRang_ly;
			YunSu_Start_cnt = 0;
            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //撞左
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;
			FY_YunSu_Zhixing_ly = 0;
		    YunSu_cnt_ly = 0;
            huanXiang_flag = 2;
			X1_ly_cnt = 0;	
			YunSuFunction_Number++;
//			YunSu_PanFangXiang = (-YunSuAngleRang_ly);
//            FY_YunSu_Zhixing_ly = 0;			
		}
		
		if(YunSu_Start_cnt == 1)
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StartAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;			
			}
		}
		else
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly - FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY + 0.00015;
			}
		}

		if(FW_YunSu_Direction_ly == -1)
		{
			YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StopAngle_ly;
			YunSu_PanFangXiang = YunSu_step_design_set_ly;
//			SpaceSetValueFY = SpaceSetValueFY - 0.00015;
		}
					
	}
	
	if(YunSu_Direction_ly == 0x04)    //向左上扫描
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = -1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //撞右
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = 0;

            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
			YunSuFunction_Number++;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //撞左
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;
			FY_YunSu_Zhixing_ly = (float)FY_YunSuAngleRang_ly;

		    YunSu_cnt_ly = 0;
            huanXiang_flag = 2;
			X1_ly_cnt = 0;	
			YunSu_Start_cnt = 0;
//			YunSu_PanFangXiang = (-YunSuAngleRang_ly);
//            FY_YunSu_Zhixing_ly = 0;			
		}
		
		if(YunSu_Start_cnt == 1)
		{
			if(FW_YunSu_Direction_ly == -1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StartAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;			
			}
		}
		else
		{
			if(FW_YunSu_Direction_ly == 1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly - FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY + 0.00015;
			}
			if(FW_YunSu_Direction_ly == -1)
			{
				YunSu_step_design_set_ly = (YunSu_cnt_ly / YunSu_Time_Long_ly)*FW_YunSuStepLength_ly * FW_YunSu_Direction_ly + FW_YunSu_StopAngle_ly;
				YunSu_PanFangXiang = YunSu_step_design_set_ly;
//				SpaceSetValueFY = SpaceSetValueFY - 0.00015;
			}	
		}
					
	}
	
}