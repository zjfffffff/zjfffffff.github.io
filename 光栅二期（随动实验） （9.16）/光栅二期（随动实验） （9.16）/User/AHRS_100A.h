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
	float Gyro_X;//X 轴陀螺数据
	float Gyro_Y;//Y 轴陀螺数据
	float Gyro_Z;//Z 轴陀螺数据
	float Accl_X;//X 轴加速度数据
	float Accl_Y;//Y 轴加速度数据
	float Accl_Z;//Z 轴加速度数据
	float Temperature;//温度
	float Roll;//横滚角度数据
	float Pitch;//俯仰角度数据
	float Yaw;//偏航角度数据
}AHRS_100A_Data;

extern AHRS_100A_Data Gyro;

extern float Gyro_X_pj;
extern float Gyro_Y_pj;
extern float Gyro_Z_pj;
extern float Pitch_pj;
extern float Roll_pj;
extern float Yaw_pj;

#endif
