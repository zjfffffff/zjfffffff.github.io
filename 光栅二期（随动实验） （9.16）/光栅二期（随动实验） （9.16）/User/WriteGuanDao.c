#include "stm32f4xx.h"	
#include "WriteGuanDao.h"
#include "aaa.h"
#include "ioinit.h"

#define Write_GuanDao_ZhuangTai        ((u32)(0x64002800))     //             惯导向上传地址
#define Write_GuanDao_ZhuangTai_En     ((u32)(0x64002C00))     //             惯导向上传使能

union UintToArray GuanDao_Data;//
union IntToArray  GuanDao_InData;
union Int16ToArray1 GuanDao_ShortData;

u8 WR_GuanDao[27] = {0};
u16 GunDao_cnt = 0;
u8 Sum_GuanDao = 0;
u16 WRGuanDao_CNT = 0;
s32 WeiDu = 0;
s32 JingDu = 0;
s32 GaoDu = 0;

extern float Lattitude_f,Longitude_f,Altitude_f;
void WriteGuanDao()
{
	WR_GuanDao[0] = 0x99;        //帧头
	WR_GuanDao[1] = 0x66;        //帧头
	WR_GuanDao[2] = 0x17;        //数据长度
	
	
	GuanDao_Data.k=0;               //时间
	WR_GuanDao[3]=GuanDao_Data.ary[0];
	WR_GuanDao[4]=GuanDao_Data.ary[1];
	WR_GuanDao[5]=GuanDao_Data.ary[2];
	WR_GuanDao[6]=GuanDao_Data.ary[3];
	
	WeiDu = (43.82 * 11930464.70555555);
//	WeiDu = (Lattitude_f * 11930464.70555555);
//	WeiDu = (10.67 * 11930464.70555555);
	GuanDao_InData.k=WeiDu;               //纬度   成都:30.67
	WR_GuanDao[7]=GuanDao_InData.ary[0];
	WR_GuanDao[8]=GuanDao_InData.ary[1];
	WR_GuanDao[9]=GuanDao_InData.ary[2];
	WR_GuanDao[10]=GuanDao_InData.ary[3];
	
	GaoDu = (242 * 131080.000427);
//	GaoDu = (Altitude_f * 131080.000427);
//	GaoDu = (100 * 131080.000427);
	GuanDao_InData.k=GaoDu;               //高度
	WR_GuanDao[11]=GuanDao_InData.ary[0];
	WR_GuanDao[12]=GuanDao_InData.ary[1];
	WR_GuanDao[13]=GuanDao_InData.ary[2];
	WR_GuanDao[14]=GuanDao_InData.ary[3];
	
	JingDu = (125.3 * 11930464.70555555);
//	JingDu = (Longitude_f * 11930464.70555555);
//	JingDu = (64.06 * 11930464.70555555);
	GuanDao_InData.k=JingDu;               //经度  成都:104.06
	WR_GuanDao[15]=GuanDao_InData.ary[0];
	WR_GuanDao[16]=GuanDao_InData.ary[1];
	WR_GuanDao[17]=GuanDao_InData.ary[2];
	WR_GuanDao[18]=GuanDao_InData.ary[3];
     
	GuanDao_ShortData.k=0;               //北向速度
	WR_GuanDao[19]=GuanDao_ShortData.ary[0];
	WR_GuanDao[20]=GuanDao_ShortData.ary[1];
	
	GuanDao_ShortData.k=0;               //天向速度
	WR_GuanDao[21]=GuanDao_ShortData.ary[0];
	WR_GuanDao[22]=GuanDao_ShortData.ary[1];
	
	GuanDao_ShortData.k=0;               //东向速度
	WR_GuanDao[23]=GuanDao_ShortData.ary[0];
	WR_GuanDao[24]=GuanDao_ShortData.ary[1];

//	GuanDao_ShortData.k=0;               //横滚角
//	WR_GuanDao[25]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[26]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //航向角
//	WR_GuanDao[27]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[28]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //俯仰角
//	WR_GuanDao[29]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[30]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //X轴角速度
//	WR_GuanDao[31]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[32]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //Y轴角速度
//	WR_GuanDao[33]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[34]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //Z轴角速度
//	WR_GuanDao[35]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[36]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //X轴加速度
//	WR_GuanDao[37]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[38]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //Y轴加速度
//	WR_GuanDao[39]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[40]=GuanDao_ShortData.ary[1];
//	
//	GuanDao_ShortData.k=0;               //Z轴加速度
//	WR_GuanDao[41]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[42]=GuanDao_ShortData.ary[1];
		WR_GuanDao[25]=0x01;
//	GuanDao_ShortData.k=0x01;               //状态字
//	WR_GuanDao[43]=GuanDao_ShortData.ary[0];
//	WR_GuanDao[44]=GuanDao_ShortData.ary[1];
	
	Sum_GuanDao = 0;
	for(GunDao_cnt=2;GunDao_cnt<26;GunDao_cnt++)
	{
		Sum_GuanDao = Sum_GuanDao + WR_GuanDao[GunDao_cnt];
	}
	WR_GuanDao[26] = Sum_GuanDao;
	
	
	for(WRGuanDao_CNT=0; WRGuanDao_CNT<27; WRGuanDao_CNT++)
	{
		*(uint32_t*)(Write_GuanDao_ZhuangTai)= WR_GuanDao[WRGuanDao_CNT];
	}
	*(uint32_t*)(Write_GuanDao_ZhuangTai_En)= 1;
//	Delay_ms(10);
	
	
}