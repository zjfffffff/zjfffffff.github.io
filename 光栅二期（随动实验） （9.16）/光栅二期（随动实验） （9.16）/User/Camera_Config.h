#include "stm32f4xx.h"
#include "ASCII.h"

#define CAM_Com_Flag		0x0010					//������ڽ��ձ�־λ
#define CAM_ADDR            ((u32)(0x64002800))
#define CAM_Com_En          ((u32)(0x64002c00))
/*���ò�����ַ*/
#define Camera_Restart_Addr 								((u16)(0x1200))//�������
#define Save_Configuration_Addr								((u16)(0x1201))//���浱ǰ����
#define Reset_Addr											((u16)(0x1202))//�ָ���������
#define Exposure_Time_Addr1									((u16)(0x1205))//�ع�ʱ���
#define Exposure_Time_Addr2									((u16)(0x1206))//�ع�ʱ���
#define X_Migration_Addr									((u16)(0x120B))//Xƫ��
#define Y_Migration_Addr									((u16)(0x120C))//Yƫ��
#define X_Length											((u16)(0x120D))//X����
#define Y_Length											((u16)(0x120E))//Y����
#define ADC_Gain											((u16)(0x1218))//ADC����
#define PGA_Gain											((u16)(0x1220))//PGA����
#define Tracking_Algorithm_Addr								((u16)(0x1302))//�����㷨
#define Small_Window_Switch									((u16)(0x1303))//С�����л�
#define Threshold_Addr										((u16)(0x1306))//��ֵ����
#define Window_L											((u16)(0x0000))//�󴰿�
#define Window_S											((u16)(0x0001))//С����
#define Barycentre											((u16)(0x0000))//����
#define Centroid											((u16)(0x0001))//����

void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Write(u8 Data[],u8 n);
void Restore_Factory_Settings();
void Save_Configuration_Settings();
