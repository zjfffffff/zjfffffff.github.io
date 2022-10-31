#ifndef AHRS_100A_H
#define AHRS_100A_H

#include "stm32f4xx.h"
#include "FSMC.h"
#include "math.h"

float AHRS_100A_Decode(u8 byte_1,u8 byte_2,u8 byte_3,float LSB);
void Gyro_Read(void);
float Gyro_RMS(float Xi[],int n);
void Gyroscope_data_Process(void);

typedef struct
{
	float Gyro_X;//X ����������
	float Gyro_Y;//Y ����������
	float Gyro_Z;//Z ����������
	float Accl_X;//X ����ٶ�����
	float Accl_Y;//Y ����ٶ�����
	float Accl_Z;//Z ����ٶ�����
	float Temperature;//�¶�
	float Roll;//����Ƕ�����
	float Pitch;//�����Ƕ�����
	float Yaw;//ƫ���Ƕ�����
}AHRS_100A_Data;

extern AHRS_100A_Data Gyro;

extern float Gyro_X_pj;
extern float Gyro_Y_pj;
extern float Gyro_Z_pj;
extern float Pitch_pj;
extern float Roll_pj;
extern float Yaw_pj;

#endif
