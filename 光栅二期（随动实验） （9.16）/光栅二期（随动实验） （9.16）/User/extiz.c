#include "extiz.h"
u8 	z_cnt = 0;
u8	EXTI2_FLAG = 0;	
u32  EXTI2_CNT=0;
extern int fasong_cnt;
extern float aa_test;
void EXTI9_5_IRQHandler(void)               //������PE4 = Z+
{

	EXTI2_FLAG = 1;
    EXTI2_CNT++;	
//if(EXTI2_CNT<=10)
//	 aa_test=aa_test+0.02;
//	else if(EXTI2_CNT<=20)
//		aa_test=aa_test-0.02;
//	if(EXTI2_CNT>30)EXTI2_CNT=0;
//     z_cnt++;
//	 TIM_SetCounter(TIM2,0);
//	 EXTI_ClearITPendingBit(EXTI9_5_IRQn);//���LINE4�ϵ��жϱ�־λ  
	EXTI_ClearITPendingBit(EXTI_Line7);  
}
	   
//�ⲿ�жϳ�ʼ������
//��ʼ��PE2~4,PA0Ϊ�ж�����.
void EXTIX2_Init(void) 
{
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOA,GPIOEʱ��
// 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //KEY0 KEY1 KEY2��Ӧ����
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
////	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOE2,3,4
// 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
//	
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//PC2 ���ӵ��ж���2
//	
//	/* ����EXTI_Line2,3,4 */
//	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�����ش���
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
//    EXTI_Init(&EXTI_InitStructure);//����
// 
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//�����ȼ�2
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
//    NVIC_Init(&NVIC_InitStructure);//����
	
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PF7 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource7);

  /* Configure EXTI Line7 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
//  NVIC_InitTypeDef   NVIC_InitStructure;
//	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);	
	 


	   
}