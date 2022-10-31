#include "encode.h"
#include "stm32f4xx.h"	
/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/

void Encoder_Init_TIM2(void)      //������PA0 = A+ & PB3 = B+
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	//  NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //ʹ�ܶ�ʱ��2��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //ʹ��PA�˿�ʱ��
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  //ʹ��PB�˿�ʱ��

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);  //PA0����Ϊ��ʱ��2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  //PB3����Ϊ��ʱ��2 

	// �ź� A
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// �ź� B
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/*******************TIM2 ����Ϊ������ģʽ***********************/
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);     //��TIM_TimeBaseStructure�еĲ�����ȱʡֵ����
	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = 0xffffffff; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	//ʹ�ñ�����ģʽ3 ��TIM_IC1�ļ��ԣ� TIM_IC2�ļ��� 
	TIM_ICStructInit(&TIM_ICInitStructure);  //���ṹ���е�����ȱʡ����
	TIM_ICInitStructure.TIM_ICFilter = 0;    //ѡ������Ƚ��˲��� 
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);    //���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);   //���и����ж�
	//Reset counter  ��λ������
	TIM_SetCounter(TIM2,0);      //��ռ�����
	TIM_Cmd(TIM2, ENABLE);       //����TIM4��ʱ��
}
