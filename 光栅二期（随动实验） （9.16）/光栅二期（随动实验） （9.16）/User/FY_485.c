#include "stm32f4xx.h"	
#include "FY_485.h"
#include "ioinit.h"


#define ADDR_485        ((u32)(0x64001800))     //485串口地址
#define En_485          ((u32)(0x64001C00))     //485串口使能位 
#define READ485_Flag    ((u32)(0x6400F000))

#define Read_FPGA_ADDR  ((u32)(0x64001800))

u32 FPGA_485_ADDR = 0;
u8 Mode_Chose = 1;
u8 rs485buf[14] = {0};
float FY_Target_Position = 0;
float FY_Speed_Position = 0;
int64_t Buff_Target = 0;
s32 Voltage = 0;
u32 Read_485Flag = 0;
u8 Read_485_Buff[13] = {0};
extern float FY_encoder_degrees;  
int64_t Buff_Speed = 0;

void FY_485control(void)
{
	if(Mode_Chose == 1)     //位置环控制
	{
		Buff_Target = FY_Target_Position * 100;    //传递位置信息
		if(Buff_Target > 35000)
		{
			Buff_Target = 35000;
		}
		if(Buff_Target < 1000)
		{
			Buff_Target = 1000;
		}
		rs485buf[0]=0x3e;
		rs485buf[1]=0xa3;
		rs485buf[2]=0x01;
		rs485buf[3]=0x08;
		rs485buf[4]=0xea;
		rs485buf[5]=Buff_Target;
		rs485buf[6]=Buff_Target>>8;
		rs485buf[7]=Buff_Target>>16;
		rs485buf[8]=Buff_Target>>24;
		rs485buf[9]=Buff_Target>>32;
		rs485buf[10]=Buff_Target>>40;
		rs485buf[11]=Buff_Target>>48;
		rs485buf[12]=Buff_Target>>56;
		rs485buf[13]=rs485buf[8]+rs485buf[7]+rs485buf[6]+rs485buf[5]+rs485buf[9]+rs485buf[10]+rs485buf[11]+rs485buf[12];
		Write_485();
		Read_485();
	}
	if(Mode_Chose == 2)     //速度环控制
	{
		Buff_Speed = FY_Speed_Position * 100;    //传递位置信息
		rs485buf[0]=0x3e;
		rs485buf[1]=0xa2;
		rs485buf[2]=0x1;
		rs485buf[3]=0x4;
		rs485buf[4]=0xe5;
		rs485buf[5]=Buff_Speed;
		rs485buf[6]=Buff_Speed>>8;
		rs485buf[7]=Buff_Speed>>16;
		rs485buf[8]=Buff_Speed>>24;
		rs485buf[9]=rs485buf[8]+rs485buf[7]+rs485buf[6]+rs485buf[5];
		Write_voltage();//发送5个字节 
		Read_485();
	}
	
}

void Write_485()
{
	//union UintToArray ia;
	//ia.k = 300;
	u8 i_485 = 0;
	
	for(i_485=0; i_485<14; i_485++)
	{
		*(uint32_t*)(ADDR_485)= rs485buf[i_485];
	}
	Delay_us(1);
	*(uint32_t*)(En_485)= 1;
	//AA_fail=0X01;

}
void Write_voltage()
{
	//union UintToArray ia;
	//ia.k = 300;
	u8 i_485 = 0;
	
	for(i_485=0; i_485<10; i_485++)
	{
		*(uint32_t*)(ADDR_485)= rs485buf[i_485];
	}
	Delay_us(1);
	*(uint32_t*)(En_485)= 1;
	//AA_fail=0X01;

}
u16 FY_485_encoder = 0;
float FY_485_encoder_degrees = 0;
float FY_485_encoder_float = 0;
u16 FY_485_Speed = 0;
float FY_485_SpeedFloat = 0;

void Read_485()
{
	u8 i = 0;
	Read_485Flag=*(uint32_t*)(READ485_Flag);
	if((Read_485Flag&0x0004) == 0x0004)
	{
			FPGA_485_ADDR=Read_FPGA_ADDR;
			for(i=0;i<13;i++)
			{
				Read_485_Buff[i]=*(uint32_t*)(FPGA_485_ADDR);
				FPGA_485_ADDR=FPGA_485_ADDR+2;
			}
			FY_485_encoder = (Read_485_Buff[11] << 8) + Read_485_Buff[10];
			FY_485_encoder_float = FY_485_encoder;
			FY_485_encoder_degrees = FY_485_encoder_float * 0.0054932;
			FY_encoder_degrees = FY_485_encoder_degrees;
			
			FY_485_Speed = (Read_485_Buff[9] << 8) + Read_485_Buff[8];
			FY_485_SpeedFloat = (float)FY_485_Speed;
			
			
	}
	
}