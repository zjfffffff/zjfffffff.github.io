#include "stm32f4xx.h"	
#include "TestScan.h"
#include "aaa.h"


#define Write_Test_Scan_ZhuangTai      ((u32)(0x64001000))    //���ڶ�ȡ��ַ                ELMO���ϴ���ַ
#define Write_Test_Scan_ZhuangTai_En   ((u32)(0x64001800))    //���ڷ���ʹ�ܵ�ַ            ELMO���ϴ�ʹ��

u8 Test_Scan_TXData[150] = {0};
u16 Test_Scan_ZhenJiShu = 0;
u16 Test_Scan_Sum = 0;
u16 Scan_cnt = 0;
u16 Test_Scan_CNT = 0;
u8 Qie_Huan = 0;

extern union FloatToArray Temp_Data;
extern float FW_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float yaw_attitude_float;
extern float pitch_attitude_float;
extern float roll_attitude_float;
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float FW_zero_degree_zheng;     //336.64
extern float FY_zero_degree;         //323.02
extern u8 Test_Scan_Time;
extern float WaiTongBu_step_design_set,FY_WaiTongBu_ZhiXing;

void Test_Scan_Send()
{
	Test_Scan_TXData[0] = 0xEB;   //EB   90
	Test_Scan_TXData[1] = 0x90;   //EB   90
	Test_Scan_TXData[2] = 0x16;   //EB   90
	
	Temp_Data.f=(FW_encoder_degrees - FW_zero_degree_zheng);                  //��������λ
	//Temp_Data.f=FW_buchang_degree;	//��������λ
//	Temp_Data.f=FY_buchang_degree;
	Test_Scan_TXData[3]=Temp_Data.ary[3];
	Test_Scan_TXData[4]=Temp_Data.ary[2];
	Test_Scan_TXData[5]=Temp_Data.ary[1];
	Test_Scan_TXData[6]=Temp_Data.ary[0];
	
	Temp_Data.f=(FY_encoder_degrees - FY_zero_degree);                  //����������
//	Temp_Data.f=FY_buchang_degree; 
	//����������
	//Temp_Data.f=FY_encoder_degrees;
	//Temp_Data.f=FY_WaiTongBu_ZhiXing;
	Test_Scan_TXData[7]=Temp_Data.ary[3];
	Test_Scan_TXData[8]=Temp_Data.ary[2];
	Test_Scan_TXData[9]=Temp_Data.ary[1];
	Test_Scan_TXData[10]=Temp_Data.ary[0];
	
	Temp_Data.f=yaw_attitude_float; 	//���Կռ���̬��λ
//	Temp_Data.f=WaiTongBu_step_design_set;
	Test_Scan_TXData[11]=Temp_Data.ary[3];
	Test_Scan_TXData[12]=Temp_Data.ary[2];
	Test_Scan_TXData[13]=Temp_Data.ary[1];
	Test_Scan_TXData[14]=Temp_Data.ary[0];
	
	Temp_Data.f= -pitch_attitude_float;                  //���Կռ���̬����
	Test_Scan_TXData[15]=Temp_Data.ary[3];
	Test_Scan_TXData[16]=Temp_Data.ary[2];
	Test_Scan_TXData[17]=Temp_Data.ary[1];
	Test_Scan_TXData[18]=Temp_Data.ary[0];
	
//	Temp_Data.f=roll_attitude_float;                  //���Կռ���̬���
	Temp_Data.f=-0.15;                  //���Կռ���̬���
	Test_Scan_TXData[19]=Temp_Data.ary[3];
	Test_Scan_TXData[20]=Temp_Data.ary[2];
	Test_Scan_TXData[21]=Temp_Data.ary[1];
	Test_Scan_TXData[22]=Temp_Data.ary[0];

	Test_Scan_TXData[23]=Qie_Huan;
	
	Test_Scan_TXData[24]=Test_Scan_Time;
	
	Test_Scan_ZhenJiShu++;
	Test_Scan_TXData[26] = Test_Scan_ZhenJiShu;
	Test_Scan_TXData[25] = Test_Scan_ZhenJiShu >> 8;



	Test_Scan_Sum = 0;
	for(Scan_cnt=2; Scan_cnt<27;Scan_cnt++)
	{
		Test_Scan_Sum = Test_Scan_TXData[Scan_cnt] + Test_Scan_Sum;
	}
	
	Test_Scan_TXData[28] = Test_Scan_Sum;
	Test_Scan_TXData[27] = Test_Scan_Sum >>8;

	
	Test_Scan_TXData[29] = 0xFE;	                      //FE
	
	
//	GPIO_ToggleBits(GPIOE, GPIO_Pin_2);
		for(Test_Scan_CNT=0; Test_Scan_CNT<30; Test_Scan_CNT++)
		{
			*(uint32_t*)(Write_Test_Scan_ZhuangTai)= Test_Scan_TXData[Test_Scan_CNT];
		}
		*(uint32_t*)(Write_Test_Scan_ZhuangTai_En)= 1;
}