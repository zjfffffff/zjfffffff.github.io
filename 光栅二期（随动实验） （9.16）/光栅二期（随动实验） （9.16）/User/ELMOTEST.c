#include "stm32f4xx.h"	
#include "ELMOTEST.h"
#include "aaa.h"
#include "ioinit.h"

#define Write_ELMO_ZhuangTai        ((u32)(0x64004000))  //串口读取地址                ELMO向上传地址
#define Write_ELMO_ZhuangTai_En     ((u32)(0x64004800))  //串口发送使能地址            ELMO向上传使能

u8 ELMO_TXData[150] = {0};
u16 ELMO_ZhenJiShu = 0;
u8 ELMO_Sum = 0;
u16 ly_cnt = 0;
u16 ELMO_CNT = 0;

extern int32_t   EC_fw;
extern int32_t   MF_fw;
extern int32_t   SR_fw;
extern float   AN_1_fw;
extern float   AN_2_fw;
extern float   AN_3_fw;
extern float   AN_4_fw;
extern float   TI1_fw;
extern float   PX_fw;
extern float   PY_fw;
extern int32_t   EE1_fw;
extern int32_t   VX_fw;
extern int32_t   SO_fw;
 
 
extern int32_t   EC_fy;
extern int32_t   MF_fy;
extern int32_t   SR_fy;
extern float   AN_1_fy;
extern float   AN_2_fy;
extern float   AN_3_fy;
extern float   AN_4_fy;
extern float   TI1_fy;
extern float   PX_fy;
extern float   PY_fy;
extern int32_t   EE1_fy;
extern int32_t   VX_fy;
extern int32_t   SO_fy;

extern union FloatToArray Temp_Data;//
extern float FW_encoder_degrees;  //俯仰编码器转换为角度
extern float FY_encoder_degrees;  //俯仰编码器转换为角度
extern u32 Time;
extern float temp1_f;
extern float temp2_f;



void Elmo_Send()
{
	ELMO_TXData[0] = 0xEB;   //EB   90
	ELMO_TXData[1] = 0x90;   //EB   90
	
	ELMO_TXData[2] = EC_fw;
	ELMO_TXData[3] = EC_fw >> 8;
	ELMO_TXData[4] = EC_fw >> 16;
	ELMO_TXData[5] = EC_fw >> 24;

	ELMO_TXData[6] = MF_fw;
	ELMO_TXData[7] = MF_fw >> 8;
	ELMO_TXData[8] = MF_fw >> 16;
	ELMO_TXData[9] = MF_fw >> 24;
		
	ELMO_TXData[10] = SR_fw;
	ELMO_TXData[11] = SR_fw >> 8;
	ELMO_TXData[12] = SR_fw >> 16;
	ELMO_TXData[13] = SR_fw >> 24;
	
	Temp_Data.f=AN_1_fw;
	ELMO_TXData[14]=Temp_Data.ary[0];
	ELMO_TXData[15]=Temp_Data.ary[1];
	ELMO_TXData[16]=Temp_Data.ary[2];
	ELMO_TXData[17]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_2_fw;
	ELMO_TXData[18]=Temp_Data.ary[0];
	ELMO_TXData[19]=Temp_Data.ary[1];
	ELMO_TXData[20]=Temp_Data.ary[2];
	ELMO_TXData[21]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_3_fw;
	ELMO_TXData[22]=Temp_Data.ary[0];
	ELMO_TXData[23]=Temp_Data.ary[1];
	ELMO_TXData[24]=Temp_Data.ary[2];
	ELMO_TXData[25]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_4_fw;
	ELMO_TXData[26]=Temp_Data.ary[0];
	ELMO_TXData[27]=Temp_Data.ary[1];
	ELMO_TXData[28]=Temp_Data.ary[2];
	ELMO_TXData[29]=Temp_Data.ary[3];
	
	ELMO_TXData[30] = EE1_fw;
	ELMO_TXData[31] = EE1_fw >> 8;
	ELMO_TXData[32] = EE1_fw >> 16;
	ELMO_TXData[33] = EE1_fw >> 24;
	
	ELMO_TXData[34] = SO_fw;
	ELMO_TXData[35] = SO_fw >> 8;
	ELMO_TXData[36] = SO_fw >> 16;
	ELMO_TXData[37] = SO_fw >> 24;


	ELMO_TXData[38] = EC_fy;
	ELMO_TXData[39] = EC_fy >> 8;
	ELMO_TXData[40] = EC_fy >> 16;
	ELMO_TXData[41] = EC_fy >> 24;
	
	ELMO_TXData[42] = MF_fy;
	ELMO_TXData[43] = MF_fy >> 8;
	ELMO_TXData[44] = MF_fy >> 16;
	ELMO_TXData[45] = MF_fy >> 24;
	
	ELMO_TXData[46] = SR_fy;
	ELMO_TXData[47] = SR_fy >> 8;
	ELMO_TXData[48] = SR_fy >> 16;
	ELMO_TXData[49] = SR_fy >> 24;
	
	Temp_Data.f=AN_1_fy;
	ELMO_TXData[50]=Temp_Data.ary[0];
	ELMO_TXData[51]=Temp_Data.ary[1];
	ELMO_TXData[52]=Temp_Data.ary[2];
	ELMO_TXData[53]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_2_fy;
	ELMO_TXData[54]=Temp_Data.ary[0];
	ELMO_TXData[55]=Temp_Data.ary[1];
	ELMO_TXData[56]=Temp_Data.ary[2];
	ELMO_TXData[57]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_3_fy;
	ELMO_TXData[58]=Temp_Data.ary[0];
	ELMO_TXData[59]=Temp_Data.ary[1];
	ELMO_TXData[60]=Temp_Data.ary[2];
	ELMO_TXData[61]=Temp_Data.ary[3];
	
	Temp_Data.f=AN_4_fy;
	ELMO_TXData[62]=Temp_Data.ary[0];
	ELMO_TXData[63]=Temp_Data.ary[1];
	ELMO_TXData[64]=Temp_Data.ary[2];
	ELMO_TXData[65]=Temp_Data.ary[3];
	
	ELMO_TXData[66] = EE1_fy;
	ELMO_TXData[67] = EE1_fy >> 8;
	ELMO_TXData[68] = EE1_fy >> 16;
	ELMO_TXData[69] = EE1_fy >> 24;
	
	ELMO_TXData[70] = SO_fy;
	ELMO_TXData[71] = SO_fy >> 8;
	ELMO_TXData[72] = SO_fy >> 16;
	ELMO_TXData[73] = SO_fy >> 24;
	
	
	
	Temp_Data.f=FW_encoder_degrees;//粗跟踪伺服方位角度
	ELMO_TXData[74]=Temp_Data.ary[0];
	ELMO_TXData[75]=Temp_Data.ary[1];
	ELMO_TXData[76]=Temp_Data.ary[2];
	ELMO_TXData[77]=Temp_Data.ary[3];

	Temp_Data.f=FY_encoder_degrees;//粗跟踪伺服俯仰角度
	ELMO_TXData[78]=Temp_Data.ary[0];
	ELMO_TXData[79]=Temp_Data.ary[1];
	ELMO_TXData[80]=Temp_Data.ary[2];
	ELMO_TXData[81]=Temp_Data.ary[3];
	
	Temp_Data.f=temp1_f;   //板载温度
	ELMO_TXData[82]=Temp_Data.ary[0];
	ELMO_TXData[83]=Temp_Data.ary[1];
	ELMO_TXData[84]=Temp_Data.ary[2];
	ELMO_TXData[85]=Temp_Data.ary[3];
	
	Temp_Data.f=temp2_f;   //板载温度
	ELMO_TXData[86]=Temp_Data.ary[0];
	ELMO_TXData[87]=Temp_Data.ary[1];
	ELMO_TXData[88]=Temp_Data.ary[2];
	ELMO_TXData[89]=Temp_Data.ary[3];
	
	Temp_Data.f=PX_fw;   //方位角度
	ELMO_TXData[90]=Temp_Data.ary[0];
	ELMO_TXData[91]=Temp_Data.ary[1];
	ELMO_TXData[92]=Temp_Data.ary[2];
	ELMO_TXData[93]=Temp_Data.ary[3];
	
	ELMO_TXData[94]=VX_fw;    //方位速度
	ELMO_TXData[95]=VX_fw >>8;   
	ELMO_TXData[96]=VX_fw >>16;
	ELMO_TXData[97]=VX_fw >>24;
	
	Temp_Data.f=PX_fy;   //俯仰角度
	ELMO_TXData[98]=Temp_Data.ary[0];
	ELMO_TXData[99]=Temp_Data.ary[1];
	ELMO_TXData[100]=Temp_Data.ary[2];
	ELMO_TXData[101]=Temp_Data.ary[3];
	
	Temp_Data.f=VX_fy;   //俯仰速度
	ELMO_TXData[102]=VX_fy;
	ELMO_TXData[103]=VX_fy>>8;
	ELMO_TXData[104]=VX_fy>>16;
	ELMO_TXData[105]=VX_fy>>24;
	
	
	ELMO_TXData[106]=Time;
	ELMO_TXData[107]=Time >> 8;
	ELMO_TXData[108]=Time >> 16;
	ELMO_TXData[109]=Time >> 24;
	
	ELMO_ZhenJiShu++;
	ELMO_TXData[110] = ELMO_ZhenJiShu;
	ELMO_TXData[111] = ELMO_ZhenJiShu >> 8;


	ELMO_Sum = 0;
	for(ly_cnt=2; ly_cnt<112;ly_cnt++)
	{
		ELMO_Sum = ELMO_TXData[ly_cnt] + ELMO_Sum;
	}
	
	ELMO_TXData[112] = ELMO_Sum;
	ELMO_TXData[113] = 0xFE;	                      //FE
	
	
//	GPIO_ToggleBits(GPIOE, GPIO_Pin_2);
		for(ELMO_CNT=0; ELMO_CNT<114; ELMO_CNT++)
		{
			*(uint32_t*)(Write_ELMO_ZhuangTai)= ELMO_TXData[ELMO_CNT];
		}
		*(uint32_t*)(Write_ELMO_ZhuangTai_En)= 1;
		Delay_ms(100);
}