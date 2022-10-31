#include "stm32f4xx.h"
#include "FSMC.h"

void bsp_InitFsmc(void)
{
	FSMC_NORSRAMInitTypeDef       FSMC_NSInitStructure;
	FSMC_NORSRAMTimingInitTypeDef ReadWriteTiming;
	FSMC_NORSRAMTimingInitTypeDef WriteTiming;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIOE�������
	
	GPIO_InitStructure.GPIO_Pin=0xf03f;
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIOF�������
	
	GPIO_InitStructure.GPIO_Pin=0x163f;
	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOG�������
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);////
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);////
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);//

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);//

	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15 , GPIO_AF_FSMC);//

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5 , GPIO_AF_FSMC);//
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource12 , GPIO_AF_FSMC);//���Ź��ܸ�������

	FSMC_NSInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM2 ;//��������洢��ı�ź�����
  FSMC_NSInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;//���õ�ַ���������߹��ܸ��ã������ã�
  FSMC_NSInitStructure.FSMC_MemoryType =  FSMC_MemoryType_SRAM;//���ô洢���� SRAM
  FSMC_NSInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//�������ݿ��
  FSMC_NSInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;//����дʹ��
  FSMC_NSInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Enable;//�첽ģʽ������Ҫ����
  FSMC_NSInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;//��չģʽʹ��λ �������ö�д����ʱ��
  FSMC_NSInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;//�첽ģʽ������Ҫ����
	
  ReadWriteTiming.FSMC_AddressSetupTime = 10;//3��HCLK��ϵͳʱ�ӣ�ʱ��   ���ö�ʱ��
  ReadWriteTiming.FSMC_AddressHoldTime = 0;//�첽ģʽ ����Ҫ����
  ReadWriteTiming.FSMC_DataSetupTime = 10;//3��HCLK��ϵͳʱ�ӣ�ʱ��
  ReadWriteTiming.FSMC_BusTurnAroundDuration = 1;
  ReadWriteTiming.FSMC_CLKDivision = 0;
  ReadWriteTiming.FSMC_DataLatency = 0;
  ReadWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A; //�첽 Aģʽ  �������ö�д��ͬʱ��
	
  WriteTiming.FSMC_AddressSetupTime = 10;//����дʱ��
  WriteTiming.FSMC_AddressHoldTime = 0;
  WriteTiming.FSMC_DataSetupTime = 10;
  WriteTiming.FSMC_BusTurnAroundDuration = 0;
  WriteTiming.FSMC_CLKDivision = 0;
  WriteTiming.FSMC_DataLatency = 0;
  WriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;//�첽 Aģʽ  �������ö�д��ͬʱ��
	
	FSMC_NSInitStructure.FSMC_ReadWriteTimingStruct=&ReadWriteTiming;
	FSMC_NSInitStructure.FSMC_WriteTimingStruct=&WriteTiming;;
	
	FSMC_NORSRAMInit(&FSMC_NSInitStructure);
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2,ENABLE);	

}
