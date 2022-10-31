#ifndef CAMERA_H
#define CAMERA_H
#include "stm32f4xx.h"
#include "ASCII.h"
#include "FSMC.h"
#include "delay.h"

/*���ò�����ַ*/
#define Camera_Restart_Addr 									((u16)(0x1200))//�������
#define Save_Configuration_Addr								((u16)(0x1201))//���浱ǰ����
#define Reset_Addr														((u16)(0x1202))//�ָ���������
#define Exposure_Time_Addr1										((u16)(0x1205))//�ع�ʱ���
#define Exposure_Time_Addr2										((u16)(0x1206))//�ع�ʱ���
#define X_Migration_Addr											((u16)(0x120B))//Xƫ��
#define Y_Migration_Addr											((u16)(0x120C))//Yƫ��
#define X_Length															((u16)(0x120D))//X����
#define Y_Length															((u16)(0x120E))//Y����
#define ADC_Gain															((u16)(0x1218))//ADC����
#define PGA_Gain															((u16)(0x1220))//PGA����
#define Tracking_Algorithm_Addr								((u16)(0x1302))//�����㷨
#define Small_Window_Switch										((u16)(0x1303))//С�����л�
#define Threshold_Addr												((u16)(0x1306))//��ֵ����


#define Window_L															((u16)(0x0000))//�󴰿�
#define Window_S															((u16)(0x0001))//С����
#define Barycentre														((u16)(0x0000))//����
#define Centroid															((u16)(0x0001))//����

#endif


extern u32 Exposure_Time;//�ع�ʱ��
extern u16 X_Migration_Setting;//����Xƫ��
extern u16 Y_Migration_Setting;//����Yƫ��
extern u16 X_Length_Setting;//����X����
extern u16 Y_Length_Setting;//����Y����
extern u8 Windows_Switch;//�л�����
extern u16 Threshold;//��ֵ
extern u8 Tracing_Algorithm;//�����㷨
///////////////////////
extern u16 X_Migration_Setting1;//����Xƫ��
extern u32 Exposure_Time1;//�ع�ʱ��
extern u16 X_Length_Setting1;//����X����
extern u16 Y_Length_Setting1;//����Y����
extern u16 Y_Migration_Setting1;//����Yƫ��
extern u16 Threshold1;//��ֵ
extern u8 Tracing_Algorithm1;//�����㷨
extern u8 Windows_Switch1;//�л�����

////////////////////
extern u16 Image_Threshold;//����ͼ����ֵ
extern u8 Image_Gray_Value;//ͼ��ƽ���Ҷ�ֵ
extern u8 Background_Gray_Value;//����ƽ���Ҷ�ֵ
extern u8 Spot_Gray_Value;//���ƽ���Ҷ�ֵ
extern u8 Camera_Window;//�������״̬
extern u8 Coarse_Image_Symbol;//ͼ��ָ��ٱ�־
extern u8 Precise_Image_Symbol;//ͼ�񾫸��ٱ�־
extern u8 Image_Algorithm;//ͼ���㷨
extern u32 Integral_Time;//����ʱ��

extern u8 Camera_Error,Camera_ADD_Error;
/////////////////////
extern u16 Image_Threshold1;//����ͼ����ֵ
extern u8 Image_Gray_Value1;//ͼ��ƽ���Ҷ�ֵ
extern u8 Background_Gray_Value1;//����ƽ���Ҷ�ֵ
extern u8 Spot_Gray_Value1;//���ƽ���Ҷ�ֵ
extern u8 Camera_Window1;//�������״̬
extern u8 Coarse_Image_Symbol1;//ͼ��ָ��ٱ�־
extern u8 Precise_Image_Symbol1;//ͼ�񾫸��ٱ�־
extern u8 Image_Algorithm1;//ͼ���㷨
extern u32 Integral_Time1;//����ʱ��

extern u8 Camera_Error1,Camera_ADD_Error1;
extern u8 Camera_Step,Camera_TX_falg;

////////////////



void Camera_Param_Set1(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer1(u32 ADDR,u8 Data[]);
void Camera_Answer_Param1(u16 Param_Addr,u16 Param_Data);
void Restore_Factory_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Save_Configuration_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);

void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer(u32 ADDR,u8 Data[]);
void Camera_Answer_Param(u16 Param_Addr,u16 Param_Data);
void Restore_Factory_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Save_Configuration_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Set_chu(u32 ADDR,u8 Data[]);
void Camera_Param_Set2(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set2(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer2(u32 ADDR,u8 Data[]);
void Camera_Answer_Param2(u16 Param_Addr,u16 Param_Data);
