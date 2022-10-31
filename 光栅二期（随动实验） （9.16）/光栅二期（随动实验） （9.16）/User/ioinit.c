#include "ioinit.h"
#include "FSMC.h"
u16 ad1_out_u = 0;
void IO_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOF时钟
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟
  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
  GPIO_ResetBits(GPIOE,GPIO_Pin_2|GPIO_Pin_3);//GPIOF9,F10设置高，灯灭

	  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
  GPIO_ResetBits(GPIOA,GPIO_Pin_6);//GPIOF9,F10设置高，灯灭
	
	  //GPIOC10,初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	GPIO_SetBits(GPIOC, GPIO_Pin_1);//初始化
	
		  //GPIOD11,初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化
  
}
void Delay_ms(int num)
{
	//__IO uint32_t nCount=0x3FFFFF;  //0x3FFFFF这个延时是100ms
	__IO uint32_t nCount=0xA3D7;  //0xA3D7这个延时是1ms
	int i;
	for (i=0; i<num; i++)
	{
		while(nCount--)
		{
		}
		nCount=0xA3D7;
	}
}
void Delay_us(int num)   //这个不是很准确，延时 1us实际上是 1.16us
{
	//__IO uint32_t nCount=0x3FFFFF;  //0x3FFFFF这个延时是100ms
	__IO uint32_t nCount=0x29;  //0x29这个延时是1us
	int i;
	for (i=0; i<num; i++)
	{
		while(nCount--)
		{
		}
		nCount=0x29;
	}
}  

float OFFset = 0;


/*
		da_in1 = 9999
		da_in2 = FY_output_value_da
*/
u16 			ad2_out_u = 0;
float F_biaozhi1 = -0.0655;
float F_biaozhi2 = -0.075;
void DA12_out(float da_in1,float da_in2)
{

	int16_t ad1_out_int = 0;
	
//	u16 			ad2_out_u = 0;
	int16_t   ad2_out_int = 0;


	if(da_in1 != 9999)
	{
		if(da_in1 > 9)
		{
			da_in1 = 9;
		}
		if(da_in1 < -9 )
		{
			da_in1 = -9;
		}	
//		if(da_in1 > 2)
//		{
//			da_in1 = 2;
//		}
//		if(da_in1 < -2 )
//		{
//			da_in1 = -2;
//		}

		da_in1 = da_in1 + 10.0018034 + F_biaozhi1;      //	主伺服板 da_in1 = da_in1 - 0.056789;
		ad1_out_int = da_in1 / 10.0 * 32767;
		ad1_out_u = ad1_out_int;
		*(uint32_t*)(DA21)= (ad1_out_u)&0x000FFFF;//B900
		// *(uint32_t*)(DA2H)= (addat>>16)&0x0003;
		//	*(uint32_t*)(DA22)= (addat1)&0x000FFFF;
		*(uint32_t*)(DA1_EN)= 1;
	}	
	if(da_in2 != 9999)
	{
		if(da_in2 > 9)
		{
			da_in2 = 9;
		}
		if(da_in2 < -9 )
		{
			da_in2 = -9;
		}
//		if(da_in2 > 2)
//		{
//			da_in2 = 2;
//		}
//		if(da_in2 < -2 )
//		{
//			da_in2 = -2;
//		}
		da_in2 = da_in2 + 9.99899483 + F_biaozhi2;    // 主伺服板 da_in2 = da_in2 - 0.095021;  俯仰电机
		ad2_out_int = da_in2 / 10.0 * 32767;
		ad2_out_u = ad2_out_int;
		*(uint32_t*)(DA22)= (ad2_out_u)&0x000FFFF;//BA00
		// *(uint32_t*)(DA2H)= (addat>>16)&0x0003;
	//	*(uint32_t*)(DA22)= (addat1)&0x000FFFF;
		*(uint32_t*)(DA2_EN)= 1;
	}

	
//	*(uint32_t*)(DA2_EN)= 1;	//Delay_ms(1);// FPGA_READ_TEST_ADDR+=2;
//	*(uint32_t*)(DA1_EN)= 1;

}


//史哥DA输出程序
//		*(uint32_t*)(DA21)= (addat)&0x000FFFF;
//	    // *(uint32_t*)(DA2H)= (addat>>16)&0x0003;
//		*(uint32_t*)(DA22)= (addat1)&0x000FFFF;
//		*(uint32_t*)(DA2_EN)= 1;	//Delay_ms(1);// FPGA_READ_TEST_ADDR+=2;
//		wMO(0x37f,1);
//		if(jieyue_flag == 1)
//		{
//			wPR(0x37f,186413);
//			Delay_us(100);
//			wBG(0x37f);
//			jieyue_flag = 0;			
//		}

//董老师DA分辨率测试
//		if(timer_cnt > 20)
//		{
//			timer_cnt = 0;
//		}
//		else
//		{
//			timer_cnt++;
//		}
//		if(timer_cnt < 10)
//		{
//			timer_cnt_1++;
//		}
//		else if(timer_cnt < 20)
//		{
//			timer_cnt_1--;
//		}
//		addat = timer_cnt_1 * DA_step_k; 