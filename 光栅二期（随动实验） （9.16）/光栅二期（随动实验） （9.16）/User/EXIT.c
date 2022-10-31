#include "stm32f4xx.h"	
#include "EXIT.h"
#include "PointFunction.h"
#include "ioinit.h"
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"

void WaiTongBu_LuJing(void);


int WaiTongBu_Scan_StepCount = 0;  
float WaiTongBu_step_design_set = 0;
u32 WaiTongBu_MaiChong_cnt=0;
int WaiTongBu_ScanAngleRang = 6;                        //ɨ�跶Χ
float WaiTongBu_FY_ScanAngleRang = 1;
float WaiTongBu_FW_ScanStepLength = 2;                      //ɨ�貽��
int WaiTongBu_Scan_Direction = 0x01;                        //�ж�ɨ�跽��
float WaiTongBu_FW_Scan_StopAngle = 0;                     //��λɨ������Ƕ�
int	WaiTongBu_FW_Scan_StartAngle = 0;                    //��λɨ����ʼ�Ƕ�
int WaiTongBu_Flag_Com_Direction = 1;                    //����ֻ��һ��ֵ
int WaiTongBu_FW_Direction = 0;           //��+1  ��-1   //��λ����
int WaiTongBu_FY_Direction = 0;          //��+1   ��-1   //��������
float WaiTongBu_Cha = 0;
float WaiTongBu_PanFangXiang = 0;
float FY_WaiTongBu_ZhiXing = 0;
u8 WaiTongBu_Start_cnt = 1;
u16 Chang_Kp_CNT = 0;
u8 EXIT_POINT_FLAG = 0;

extern u8 ZhiXiangFlag;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
extern u8 EXTI2_FLAG;	
extern float FW_location_kp;

extern u8 Give_Scan_Number;
u16 EXIT_Scan_Number = 0;
float KongJianZhiXiang_FWLY = 0;
float KongJianZhiXiang_FYLY = 0;
u8 ZhiXiang_Complete_FlagLY = 0;

extern unsigned int diaoyong_cnt;

float EXIT_FW_Zero = 0;
float EXIT_FY_Zero = 0;
extern int fasong_cnt;

u8 EXIT_LianXu_FLAG = 0;
u8 EXIT_Song = 0;

u8 MoShiQieHuan = 0;
float KJZX_FW_last = 0;
float step_EXIT_ly_prepass = 0;
float step_EXIT_Y_prepass = 0;
u8 BeginMove_flag = 0;
u16 BeginMove_CNT = 0;
u8 WaiTongBu_BianLiang = 0;
u8 Zhuang_Right = 0;
u8 Zhuang_Left = 0;

void ExitTongBu()
{
///////////////////////////////////////�ж�ִ�д���///////////////////////////////////	
//	if(Give_Scan_Number != 0)
//	{
//		if(Give_Scan_Number == EXIT_Scan_Number)
//		{
//			EXIT_Scan_Number = 0;
//			EXTI2_FLAG = 0;
//		}	
//	}
//	else
//	{
//		EXIT_Scan_Number = 0;
//	}	
///////////////////////////////////////��ͬ��ǰ��ָ����ʼ��//////////////////////////////////		
	if(EXIT_POINT_FLAG == 0)
	{
		ZhiXiangFlag = 1;		
		if(KongJianZhiXiang_FWLY > SpaceSetValueFW)                 //�ռ�ָ��λ
		{				
			SpaceSetValueFW = SpaceSetValueFW + 0.001;
            ZhiXiang_Complete_FlagLY = 0;				
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWLY)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWLY)>-0.01))
//		if(SpaceSetValueFW >= KongJianZhiXiang_FWLY)
		{
			SpaceSetValueFW = KongJianZhiXiang_FWLY;				
		}
			
		if(KongJianZhiXiang_FWLY < SpaceSetValueFW)
		{				
			SpaceSetValueFW = SpaceSetValueFW - 0.001;
            ZhiXiang_Complete_FlagLY = 0;					
		}
		if(((SpaceSetValueFW - KongJianZhiXiang_FWLY)<0.01)&&((SpaceSetValueFW - KongJianZhiXiang_FWLY)>-0.01))
		{
			SpaceSetValueFW = KongJianZhiXiang_FWLY;
		}
		
		if(KongJianZhiXiang_FYLY > SpaceSetValueFY)          //�ռ�ָ����
		{				
			SpaceSetValueFY = SpaceSetValueFY + 0.001;
			ZhiXiang_Complete_FlagLY = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYLY)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYLY)>-0.01))
	//	if(SpaceSetValueFY >= KongJianZhiXiang_FYLY)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYLY;
		}
			
		if(KongJianZhiXiang_FYLY < SpaceSetValueFY)
		{				
			SpaceSetValueFY = SpaceSetValueFY - 0.001;
			ZhiXiang_Complete_FlagLY = 0;					
		}
		if(((SpaceSetValueFY - KongJianZhiXiang_FYLY)<0.01)&&((SpaceSetValueFY - KongJianZhiXiang_FYLY)>-0.01))
	//	if(SpaceSetValueFY <= KongJianZhiXiang_FYLY)
		{
			SpaceSetValueFY = KongJianZhiXiang_FYLY;
		}
                                        
     if((SpaceSetValueFY == KongJianZhiXiang_FYLY) && (SpaceSetValueFW == KongJianZhiXiang_FWLY)) //ָ���ȶ���һ������
		{
			EXIT_FW_Zero = SpaceSetValueFW;          //ָ����ʱ��λ�ö�Ϊ��ͬ�������    ��λ
			EXIT_FY_Zero = SpaceSetValueFY;          //ָ����ʱ��λ�ö�Ϊ��ͬ�������    ����
			
			ZhiXiang_Complete_FlagLY++;
			if(ZhiXiang_Complete_FlagLY == 1)
			{
				GPIO_SetBits(GPIOA,GPIO_Pin_6);
				Delay_us(110);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
				KJZX_FW_last = KongJianZhiXiang_FWLY;
				MoShiQieHuan = 2;
			}
			else
			{
				ZhiXiang_Complete_FlagLY = 2;
			}
//			EXTI2_FLAG = 1;	
		}		
		SpacePoint();	
	}
	
		if(EXIT_LianXu_FLAG == 1)
		{
				GPIO_SetBits(GPIOA,GPIO_Pin_6);
				Delay_us(110);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);
				EXIT_LianXu_FLAG = 0;
		}
	
		if(KJZX_FW_last == KongJianZhiXiang_FWLY && MoShiQieHuan == 1)
		{
				MoShiQieHuan = 0;
			  GPIO_SetBits(GPIOA,GPIO_Pin_6);
				Delay_us(110);
				GPIO_ResetBits(GPIOA,GPIO_Pin_6);			
		}
		
		
/////////////////////////////////////////////ִ����ͬ��ģʽ////////////////////////////////////////////		

	if(EXTI2_FLAG == 1)
	{
		EXIT_POINT_FLAG = 1;	
		fasong_cnt = 0;
		EXTI2_FLAG = 0;
//		EXIT_Scan_Number++;
		Chang_Kp_CNT++;
		WaiTongBu_MaiChong_cnt++;
		WaiTongBu_LuJing();
		ZhiXiangFlag = 1;
		
        BeginMove_flag = 1;
		
		if(Chang_Kp_CNT == 1 )
		{
			FW_location_kp = 120;
		}

		EXIT_Song = 1;
	}
	
	if(BeginMove_flag == 1 && EXIT_POINT_FLAG != 0)
	{
//		BeginMove_CNT++;
		step_EXIT_ly_prepass = FsmLeadLag1(WaiTongBu_step_design_set,FSM_X,0); //��Ծǰ���˲�
		step_EXIT_Y_prepass = FsmLeadLag1(FY_WaiTongBu_ZhiXing,FSM_Y,0);
		SpaceSetValueFW = EXIT_FW_Zero + step_EXIT_ly_prepass;
		SpaceSetValueFY = EXIT_FY_Zero + step_EXIT_Y_prepass; 
		SpacePoint();
//		if(BeginMove_CNT == 104)
//		{
//			BeginMove_CNT = 0;
//			BeginMove_flag = 0;
//		}
	}
	
	
	
	
}



void WaiTongBu_LuJing(void)
{				
	WaiTongBu_Scan_StepCount = WaiTongBu_ScanAngleRang/WaiTongBu_FW_ScanStepLength-1;
	if(WaiTongBu_Scan_Direction == 0x01)    //������ɨ��
	{
		
		WaiTongBu_BianLiang = WaiTongBu_ScanAngleRang / WaiTongBu_FW_ScanStepLength;
		WaiTongBu_FW_Scan_StopAngle = WaiTongBu_BianLiang * WaiTongBu_FW_ScanStepLength;
		
		
//		WaiTongBu_FW_Scan_StopAngle = WaiTongBu_FW_Scan_StartAngle + WaiTongBu_ScanAngleRang;
		if(WaiTongBu_Flag_Com_Direction == 1)
		{
			WaiTongBu_FW_Direction = 1; 
			WaiTongBu_FY_Direction = -1;
		}
		WaiTongBu_Flag_Com_Direction = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=((float)FW_Scan_StopAngle - WuCha)&&(FW_Direction == 1))     //ײ��
		if(WaiTongBu_PanFangXiang >= WaiTongBu_ScanAngleRang &&(WaiTongBu_FW_Direction == 1))     //ײ��
		{
			Zhuang_Right++;
			if(Zhuang_Right == 1)          //ִֻ�и���
			{
				FY_WaiTongBu_ZhiXing = (float)(-WaiTongBu_FY_ScanAngleRang);            //�����¸�
			}
			if(Zhuang_Right == 2)         //��λ����ɨ��
			{
				Zhuang_Right = 0;
				WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
				WaiTongBu_MaiChong_cnt = 0;
				WaiTongBu_Start_cnt = 0;
			}

		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<=((float)FW_Scan_StartAngle + WuCha)&&(FW_Direction == -1))   //ײ��
		if(WaiTongBu_PanFangXiang <= (-WaiTongBu_ScanAngleRang) &&(WaiTongBu_FW_Direction == -1))   //ײ��
		{
			Zhuang_Left++;
			if(Zhuang_Left == 1)          //ִֻ�и���
			{
				FY_WaiTongBu_ZhiXing = 0;            //�����¸�
			}
			if(Zhuang_Left == 2)         //��λ����ɨ��
			{
				Zhuang_Left = 0;
				WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
				WaiTongBu_MaiChong_cnt = 0;
			}
		}
		
		if(WaiTongBu_Start_cnt == 1)
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				if(Zhuang_Right == 1)
				{
					;
				}
				else
				{
					WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction +(float)WaiTongBu_FW_Scan_StartAngle;
					WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
				}
			}
		}
		else
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				if(Zhuang_Right == 1)
				{
					;
				}
				else
				{
					WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction -(float)WaiTongBu_FW_Scan_StopAngle;
					WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
				}
			}
		}				
		if(WaiTongBu_FW_Direction == -1)
		{
			if(Zhuang_Left == 1)
			{
				;
			}
			else
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction + (float)WaiTongBu_FW_Scan_StopAngle ;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}				
	}

	if(WaiTongBu_Scan_Direction == 0x02)    //������ɨ��
	{
		WaiTongBu_FW_Scan_StopAngle = WaiTongBu_FW_Scan_StartAngle + WaiTongBu_ScanAngleRang;
		if(WaiTongBu_Flag_Com_Direction == 1)
		{
			WaiTongBu_FW_Direction = -1; 
			WaiTongBu_FY_Direction = -1;
		}
		WaiTongBu_Flag_Com_Direction = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=((float)FW_Scan_StopAngle - WuCha)&&(FW_Direction == 1))     //ײ��
		if(WaiTongBu_PanFangXiang >= WaiTongBu_ScanAngleRang &&(WaiTongBu_FW_Direction == 1))     //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;
	        FY_WaiTongBu_ZhiXing = 0;

		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<=((float)FW_Scan_StartAngle + WuCha)&&(FW_Direction == -1))   //ײ��
		if(WaiTongBu_PanFangXiang <= (-WaiTongBu_ScanAngleRang) &&(WaiTongBu_FW_Direction == -1))   //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;

			WaiTongBu_Start_cnt = 0;
			FY_WaiTongBu_ZhiXing = (float)(-WaiTongBu_FY_ScanAngleRang);            //�����¸�
		}
		
		if(WaiTongBu_Start_cnt == 1)
		{
			if(WaiTongBu_FW_Direction == -1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction +(float)WaiTongBu_FW_Scan_StartAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}
		else
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction -(float)WaiTongBu_FW_Scan_StopAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
			if(WaiTongBu_FW_Direction == -1)
			{
			WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction + (float)WaiTongBu_FW_Scan_StopAngle ;
		    WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}				
				
	}

	if(WaiTongBu_Scan_Direction == 0x03)    //������ɨ��
	{
		WaiTongBu_FW_Scan_StopAngle = WaiTongBu_FW_Scan_StartAngle + WaiTongBu_ScanAngleRang;
		if(WaiTongBu_Flag_Com_Direction == 1)
		{
			WaiTongBu_FW_Direction = 1; 
			WaiTongBu_FY_Direction = -1;
		}
		WaiTongBu_Flag_Com_Direction = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=((float)FW_Scan_StopAngle - WuCha)&&(FW_Direction == 1))     //ײ��
		if(WaiTongBu_PanFangXiang >= WaiTongBu_ScanAngleRang &&(WaiTongBu_FW_Direction == 1))     //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;
			FY_WaiTongBu_ZhiXing = (float)WaiTongBu_FY_ScanAngleRang;            //�����¸�
			WaiTongBu_Start_cnt = 0;
		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<=((float)FW_Scan_StartAngle + WuCha)&&(FW_Direction == -1))   //ײ��
		if(WaiTongBu_PanFangXiang <= (-WaiTongBu_ScanAngleRang) &&(WaiTongBu_FW_Direction == -1))   //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;
	        FY_WaiTongBu_ZhiXing = 0;
		}
		
		if(WaiTongBu_Start_cnt == 1)
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction +(float)WaiTongBu_FW_Scan_StartAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}
		else
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction -(float)WaiTongBu_FW_Scan_StopAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}				
		if(WaiTongBu_FW_Direction == -1)
		{
			WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction + (float)WaiTongBu_FW_Scan_StopAngle ;
		    WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
		}				
	}

	if(WaiTongBu_Scan_Direction == 0x04)    //������ɨ��
	{
		WaiTongBu_FW_Scan_StopAngle = WaiTongBu_FW_Scan_StartAngle + WaiTongBu_ScanAngleRang;
		if(WaiTongBu_Flag_Com_Direction == 1)
		{
			WaiTongBu_FW_Direction = -1; 
			WaiTongBu_FY_Direction = -1;
		}
		WaiTongBu_Flag_Com_Direction = 0;					
//		if((FW_encoder_degrees-FW_zero_degree_zheng)>=((float)FW_Scan_StopAngle - WuCha)&&(FW_Direction == 1))     //ײ��
		if(WaiTongBu_PanFangXiang >= WaiTongBu_ScanAngleRang &&(WaiTongBu_FW_Direction == 1))     //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;
	        FY_WaiTongBu_ZhiXing = 0;

		}
//		if((FW_encoder_degrees-FW_zero_degree_zheng)<=((float)FW_Scan_StartAngle + WuCha)&&(FW_Direction == -1))   //ײ��
		if(WaiTongBu_PanFangXiang <= (-WaiTongBu_ScanAngleRang) &&(WaiTongBu_FW_Direction == -1))   //ײ��
		{
			WaiTongBu_FW_Direction = -WaiTongBu_FW_Direction;
			WaiTongBu_MaiChong_cnt = 1;

			WaiTongBu_Start_cnt = 0;
			FY_WaiTongBu_ZhiXing = (float)WaiTongBu_FY_ScanAngleRang;            //�����¸�
		}
		
		if(WaiTongBu_Start_cnt == 1)
		{
			if(WaiTongBu_FW_Direction == -1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction +(float)WaiTongBu_FW_Scan_StartAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}
		else
		{
			if(WaiTongBu_FW_Direction == 1)
			{
				WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction -(float)WaiTongBu_FW_Scan_StopAngle;
				WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
			if(WaiTongBu_FW_Direction == -1)
			{
			WaiTongBu_step_design_set = (float)WaiTongBu_MaiChong_cnt * WaiTongBu_FW_ScanStepLength * (float)WaiTongBu_FW_Direction + (float)WaiTongBu_FW_Scan_StopAngle ;
		    WaiTongBu_PanFangXiang = WaiTongBu_step_design_set;
			}
		}				
				
	}


	
}
