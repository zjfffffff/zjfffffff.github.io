//#include "stm32f4xx_it.h"
#include "usart.h"	

////////////////////////////////////////////////////////////////////////////////// 	 

//Data_value Temp_32;
//Gyro_DATA Gyro_RX;

//#define SF_ADDR_1K     ((u32)(0x64004800))

#define COM_Falg_ADDR  		((u32)(0x6400F000))  //��ȡ����1~16��״̬
#define COM_Falg_ADDR_H 	((u32)(0x6400F800))	//��ȡ����17~20��״̬


//#define SF_RX_1K_Len 54
//#define SF_TX_1K_Len 14

#define SF_RX_Len 34
#define SF_TX_Len 34


u8 SF_i=0;
u32 SF_ADDR_tmp=0;
u16 SF_COMFalg,SF_COMFalg_H;
u8  SF_TXData[SF_TX_Len],SF_Data[SF_RX_Len];
//u8  SF_Data_1K[SF_RX_1K_Len];



//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif


#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
//#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	



//��ʼ��IO ����4 
//bound:������
void uart4_init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART1ʱ��
 
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4); //GPIOA9����ΪUART4
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); //GPIOA10����ΪUART4
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}


//��ʼ��IO ����2
//bound:������
void uart2_init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10����ΪUSART1
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 
	
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

}


//��ʼ��IO ����3
//bound:������
void uart3_init(u32 bound_3)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10����ΪUSART3
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound_3;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

//	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}
	
u8 Res=0;

void UART4_IRQnHandler(void)                	//����4�жϷ������    ��ȡ�Ѱ���
{
		u8 i=0;
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(UART4);//(USART1->DR);	//��ȡ���յ�������
				USART_RX_BUF[i]=Res ;
				i++;
				if(i == 4)
				{
						i = 0;
				}
				
		}
		USART_ClearFlag(UART4, USART_IT_RXNE);
} 


void USART2_IRQHandler(void)                	//����2�жϷ������    ��ȡ�Ѱ���
{
		u8 i=0;
		if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
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
void USART3_IRQHandler(void)                	//����3�жϷ������     ��ȡ�ܿ�
{
		u8 i=0;
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
		{
				Res =USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
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
//		if((SF_COMFalg & 0x0001)==1)						//RS422  ����1�������Ѱ�����921600��
//		{
//				Com_Read(0x64000800);
//				Com_Write(0x64000800,0x64000c00,SF_TX_Len);
//		}
//		if((SF_COMFalg & 0x0002)== 0x0002)			//RS422����2�������Ѱ�����921600��
//		{
//				Com_Read(0x64001000);
//				Com_Write(0x64001000,0x64001400,SF_TX_Len);
//		} 
		if((SF_COMFalg & 0x0004)== 0x0004)			//RS422����3�������Ѱ�����921600��
		{
				Com_Read(0x64001800);
				Com_Write(0x64001800,0x64001C00,SF_TX_Len);
		}
//		if((SF_COMFalg & 0x0008)== 0x0008)			//RS422����4���������ݣ�115200��
//		{
//				Com_Read(0x64002000);
//				Com_Write(0x64002000,0x64002400,SF_TX_Len);
//		}
		if((SF_COMFalg & 0x0010)== 0x0010)			//RS422����5�������Ѱ�����921600��
		{
				Com_Read(0x64002800);
				Com_Write(0x64002800,0x64002c00,SF_TX_Len);
		}
		if((SF_COMFalg & 0x0020)== 0x0020)			//RS422����6�������Ѱ�����921600��
		{
				Com_Read(0x64003000);
				Com_Write(0x64003000,0x64003400,SF_TX_Len);
		}
		if(((SF_COMFalg>>6)&1)==1)						//RS422����7�������ܿأ�921600��
		{
				Com_Read(0x64003800);
				Com_Write(0x64003800,0x64003C00,SF_TX_Len);
		}
		if(((SF_COMFalg>>7)&1)==1)			//RS422����8����921600��
		{
				Com_Read(0x64004000);
				Com_Write(0x64004000,0x64004400,SF_TX_Len);
		}
		if(((SF_COMFalg>>8)&1)==1)			//RS422����9����921600��//���ߵ�
		{
				Com_Read(0x64004800);
				Com_Write(0x64004800,0x64004c00,SF_TX_Len);
		}
		if(((SF_COMFalg>>9)&1)==1)			//RS422����10��
		{
				Com_Read(0x64005000);
				Com_Write(0x64005000,0x64005400,SF_TX_Len);
		}
		if(((SF_COMFalg>>10)&1)==1)			//RS422����11�������Ѱ���
		{
				Com_Read(0x64005800);
				Com_Write(0x64005800,0x64005C00,SF_TX_Len);
		}
		if(((SF_COMFalg>>11)&1)==1)			//RS422����12�������Ѱ���
		{
				Com_Read(0x64006000);
				Com_Write(0x64006000,0x64006400,40);
		}
		if(((SF_COMFalg>>12)&1)==1)			//RS232����13�������Ѱ���
		{
				Com_Read(0x64006800);
				Com_Write(0x64006800,0x64006C00,40);
		}
		if(((SF_COMFalg>>13)&1)==1)			//RS232����14�������Ѱ���
		{
				Com_Read(0x64007000);
				Com_Write(0x64007000,0x64007400,40);
		}
		if(((SF_COMFalg>>14)&1)==1)			//RS232����15�������Ѱ���
		{
				Com_Read(0x64007800);
				Com_Write(0x64007800,0x64007C00,40);
		}
		if(((SF_COMFalg)>>15&1)==1)			//RS232����16�������Ѱ���
		{
				Com_Read(0x6400c000);
				Com_Write(0x6400c000,0x6400c400,40);
		}
		if(((SF_COMFalg_H)&1)==1)				//RS232����17�������Ѱ���
		{
				Com_Read(0x6400c800);
				Com_Write(0x6400c800,0x6400cC00,40);
		}
		if(((SF_COMFalg_H>>1)&1)==1)		//RS232����18�������Ѱ���
		{
				Com_Read(0x64009000);
				Com_Write(0x64009000,0x64009400,40);
		}
		if(((SF_COMFalg_H>>2)&1)==1)		///RS232����19�������Ѱ���
		{
				Com_Read(0x64009800);
				Com_Write(0x64009800,0x64009C00,40);
		}
		if(((SF_COMFalg_H>>3)&1)==1)		//RS232����20�������Ѱ���
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



