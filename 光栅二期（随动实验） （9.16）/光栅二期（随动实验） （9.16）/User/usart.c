//#include "stm32f4xx_it.h"
#include "usart.h"	

////////////////////////////////////////////////////////////////////////////////// 	 

//Data_value Temp_32;
//Gyro_DATA Gyro_RX;

//#define SF_ADDR_1K     ((u32)(0x64004800))

#define COM_Falg_ADDR  		((u32)(0x6400F000))  //读取串口1~16的状态
#define COM_Falg_ADDR_H 	((u32)(0x6400F800))	//读取串口17~20的状态


//#define SF_RX_1K_Len 54
//#define SF_TX_1K_Len 14

#define SF_RX_Len 34
#define SF_TX_Len 34


u8 SF_i=0;
u32 SF_ADDR_tmp=0;
u16 SF_COMFalg,SF_COMFalg_H;
u8  SF_TXData[SF_TX_Len],SF_Data[SF_RX_Len];
//u8  SF_Data_1K[SF_RX_1K_Len];



//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
//#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	



//初始化IO 串口4 
//bound:波特率
void uart4_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART1时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA9复用为UART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA10复用为UART4
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(UART4, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}


//初始化IO 串口2
//bound:波特率
void uart2_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10复用为USART1
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口2 
	
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

}


//初始化IO 串口3
//bound:波特率
void uart3_init(u32 bound_3)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10复用为USART3
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound_3;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

//	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}
	
u8 Res=0;

void UART4_IRQnHandler(void)                	//串口4中断服务程序    读取脱靶量
{
		u8 i=0;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(UART4);//(USART1->DR);	//读取接收到的数据
				USART_RX_BUF[i]=Res ;
				i++;
				if(i == 4)
				{
						i = 0;
				}
				
		}
		USART_ClearFlag(UART4, USART_IT_RXNE);
} 


void USART2_IRQHandler(void)                	//串口2中断服务程序    读取脱靶量
{
		u8 i=0;
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
				USART_RX_BUF[i]=Res ;
				i++;
				if(i == 4)
				{
						i = 0;
				}
				
		}
		USART_ClearFlag(USART2, USART_IT_RXNE);
} 

/////////////////////////////////////////////////////////////////////////////////////////////////
void USART3_IRQHandler(void)                	//串口3中断服务程序     读取总控
{
		u8 i=0;
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
				USART_RX_BUF[i]=Res ;
				i++;
				if(i == 4)
				{
						i = 0;
				}
				
		}
		USART_ClearFlag(USART3, USART_IT_RXNE);
} 


u16 cnt=0;
void SF_Com_Falg_Read()
{
		SF_COMFalg=*(uint32_t*)(COM_Falg_ADDR);
		SF_COMFalg_H=*(uint32_t*)(COM_Falg_ADDR_H);
//		if((SF_COMFalg & 0x0001)==1)						//RS422  串口1：机上脱靶量（921600）
//		{
//				Com_Read(0x64000800);
//				Com_Write(0x64000800,0x64000c00,SF_TX_Len);
//		}
//		if((SF_COMFalg & 0x0002)== 0x0002)			//RS422串口2：机上脱靶量（921600）
//		{
//				Com_Read(0x64001000);
//				Com_Write(0x64001000,0x64001400,SF_TX_Len);
//		} 
		if((SF_COMFalg & 0x0004)== 0x0004)			//RS422串口3：机上脱靶量（921600）
		{
				Com_Read(0x64001800);
				Com_Write(0x64001800,0x64001C00,SF_TX_Len);
		}
//		if((SF_COMFalg & 0x0008)== 0x0008)			//RS422串口4：光纤陀螺（115200）
//		{
//				Com_Read(0x64002000);
//				Com_Write(0x64002000,0x64002400,SF_TX_Len);
//		}
		if((SF_COMFalg & 0x0010)== 0x0010)			//RS422串口5：机上脱靶量（921600）
		{
				Com_Read(0x64002800);
				Com_Write(0x64002800,0x64002c00,SF_TX_Len);
		}
		if((SF_COMFalg & 0x0020)== 0x0020)			//RS422串口6：机上脱靶量（921600）
		{
				Com_Read(0x64003000);
				Com_Write(0x64003000,0x64003400,SF_TX_Len);
		}
		if(((SF_COMFalg>>6)&1)==1)						//RS422串口7：机下总控（921600）
		{
				Com_Read(0x64003800);
				Com_Write(0x64003800,0x64003C00,SF_TX_Len);
		}
		if(((SF_COMFalg>>7)&1)==1)			//RS422串口8：（921600）
		{
				Com_Read(0x64004000);
				Com_Write(0x64004000,0x64004400,SF_TX_Len);
		}
		if(((SF_COMFalg>>8)&1)==1)			//RS422串口9：（921600）//读惯导
		{
				Com_Read(0x64004800);
				Com_Write(0x64004800,0x64004c00,SF_TX_Len);
		}
		if(((SF_COMFalg>>9)&1)==1)			//RS422串口10：
		{
				Com_Read(0x64005000);
				Com_Write(0x64005000,0x64005400,SF_TX_Len);
		}
		if(((SF_COMFalg>>10)&1)==1)			//RS422串口11：机上脱靶量
		{
				Com_Read(0x64005800);
				Com_Write(0x64005800,0x64005C00,SF_TX_Len);
		}
		if(((SF_COMFalg>>11)&1)==1)			//RS422串口12：机上脱靶量
		{
				Com_Read(0x64006000);
				Com_Write(0x64006000,0x64006400,40);
		}
		if(((SF_COMFalg>>12)&1)==1)			//RS232串口13：机上脱靶量
		{
				Com_Read(0x64006800);
				Com_Write(0x64006800,0x64006C00,40);
		}
		if(((SF_COMFalg>>13)&1)==1)			//RS232串口14：机上脱靶量
		{
				Com_Read(0x64007000);
				Com_Write(0x64007000,0x64007400,40);
		}
		if(((SF_COMFalg>>14)&1)==1)			//RS232串口15：机上脱靶量
		{
				Com_Read(0x64007800);
				Com_Write(0x64007800,0x64007C00,40);
		}
		if(((SF_COMFalg)>>15&1)==1)			//RS232串口16：机上脱靶量
		{
				Com_Read(0x6400c000);
				Com_Write(0x6400c000,0x6400c400,40);
		}
		if(((SF_COMFalg_H)&1)==1)				//RS232串口17：机上脱靶量
		{
				Com_Read(0x6400c800);
				Com_Write(0x6400c800,0x6400cC00,40);
		}
		if(((SF_COMFalg_H>>1)&1)==1)		//RS232串口18：机上脱靶量
		{
				Com_Read(0x64009000);
				Com_Write(0x64009000,0x64009400,40);
		}
		if(((SF_COMFalg_H>>2)&1)==1)		///RS232串口19：机上脱靶量
		{
				Com_Read(0x64009800);
				Com_Write(0x64009800,0x64009C00,40);
		}
		if(((SF_COMFalg_H>>3)&1)==1)		//RS232串口20：机上脱靶量
		{
				Com_Read(0x6400A000);
				Com_Write(0x6400A000,0x6400A400,40);
		}
}



void Com_Read(u32 ADDR)
{
	SF_ADDR_tmp=ADDR;
  for(SF_i=0; SF_i<SF_RX_Len; SF_i++)
	{
		SF_Data[SF_i]=*(uint32_t*)(SF_ADDR_tmp);
		SF_ADDR_tmp = SF_ADDR_tmp+2;
	}
}

void Com_Write(u32 ADDR,u32 Com_En,u16 TX_Len)
{
	
	for(SF_i=0; SF_i<TX_Len; SF_i++)
	{
		*(uint32_t*)(ADDR)= SF_Data[SF_i];
	}
	*(uint32_t*)(Com_En)= 1;

}



