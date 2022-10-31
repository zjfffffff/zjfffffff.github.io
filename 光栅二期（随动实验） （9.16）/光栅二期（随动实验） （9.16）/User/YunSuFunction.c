#include "stm32f4xx.h"	
#include "YunSuFunction.h"
#include "PointFunction.h"
#include "ioinit.h"

void YunSu_LuJing(void);


int YunSu_Direction_ly = 0x01;    
float FW_YunSu_StopAngle_ly = 0;                     //��λɨ������Ƕ�
float FW_YunSu_StartAngle_ly = 0;                    //��λɨ����ʼ�Ƕ�
float YunSuAngleRang_ly = 25;  
int Flag_YunSu_Direction_ly = 1;                    //����ֻ��һ��ֵ
int FW_YunSu_Direction_ly = 0;           //��+1  ��-1   //��λ����
int FY_YunSu_Direction_ly = 0;          //��+1   ��-1   //��������
float YunSu_PanFangXiang = 0;
float FY_YunSuAngleRang_ly = 1;                      //����ɨ�跶Χ
float FY_YunSu_Zhixing_ly = 0;
u8 YunSu_Start_cnt = 1;
int YunSu_Time_Long_ly = 1;
float FW_YunSuStepLength_ly = 0.02;                      //ɨ�貽��
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
///////////////////////////////////////�ж�ִ�д���///////////////////////////////////	
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
	
	
	
///////////////////////////////////////����ǰ��ָ����ʼ��//////////////////////////////////		
	if(YunSu_POINT_FLAG == 0)
	{
		ZhiXiangFlag = 1;		
		if(KongJianZhiXiang_FWYunSu > SpaceSetValueFW)                 //�ռ�ָ��λ
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
		
		if(KongJianZhiXiang_FYYunSu > SpaceSetValueFY)          //�ռ�ָ����
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
                                        
        if((SpaceSetValueFY == KongJianZhiXiang_FYYunSu) && (SpaceSetValueFW == KongJianZhiXiang_FWYunSu)) //ָ���ȶ���һ������
		{
			YunSu_FW_Zero = SpaceSetValueFW;          //ָ����ʱ��λ�ö�Ϊ��ͬ�������    ��λ
			YunSu_FY_Zero = SpaceSetValueFY;          //ָ����ʱ��λ�ö�Ϊ��ͬ�������    ����
			
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
	if(YunSu_Direction_ly == 0x01)    //������ɨ��
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = 1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //ײ��
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = (float)(-FY_YunSuAngleRang_ly);
			YunSu_Start_cnt = 0;
            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //ײ��
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

	if(YunSu_Direction_ly == 0x02)    //������ɨ��
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = -1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //ײ��
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = 0;

            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
			YunSuFunction_Number++;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //ײ��
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
	
	if(YunSu_Direction_ly == 0x03)    //������ɨ��
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = 1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //ײ��
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = (float)FY_YunSuAngleRang_ly;
			YunSu_Start_cnt = 0;
            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //ײ��
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
	
	if(YunSu_Direction_ly == 0x04)    //������ɨ��
	{
		FW_YunSu_StopAngle_ly = FW_YunSu_StartAngle_ly + YunSuAngleRang_ly;
		if(Flag_YunSu_Direction_ly == 1)
		{
			FW_YunSu_Direction_ly = -1; 
			FY_YunSu_Direction_ly = -1;
		}
		Flag_YunSu_Direction_ly = 0;					
		if(YunSu_PanFangXiang >= YunSuAngleRang_ly &&(FW_YunSu_Direction_ly == 1))     //ײ��
		{
			FW_YunSu_Direction_ly = -FW_YunSu_Direction_ly;			
			FY_YunSu_Zhixing_ly = 0;

            YunSu_cnt_ly = 0;
			huanXiang_flag = 1;
			X2_ly_cnt = 0;
			YunSuFunction_Number++;
		}
		if(YunSu_PanFangXiang <= (-YunSuAngleRang_ly) &&(FW_YunSu_Direction_ly == -1))   //ײ��
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