#include "AHRS_100A.h"


#define Comm1_Addr 											((u32)(0x64001000))//串口1收发地址
/*
*********************************************************************************************************
*	函 数 名: AHRS_100A_Decode
*	功能说明: 惯导解码程序
*********************************************************************************************************
*/
AHRS_100A_Data Gyro;
u8 Gyro2_data[100]={0};
//float Gyro_X_Save[1000]={0};
//float Gyro_Y_Save[1000]={0};
//float Gyro_Z_Save[1000]={0};
//float Pitch_Save[1000]={0};
//float Roll_Save[1000]={0};
//float Yaw_Save[1000]={0};
float Gyro_X_toutle=0;
float Gyro_Y_toutle=0;
float Gyro_Z_toutle=0;
float Pitch_toutle=0;
float Roll_toutle=0;
float Yaw_toutle=0;
float Gyro_X_pj=0;
float Gyro_Y_pj=0;
float Gyro_Z_pj=0;
float Pitch_pj=0;
float Roll_pj=0;
float Yaw_pj=0;

float S1=0,S2=0,S3=0,S4=0,S5=0,S6=0;
float pj_1=0,pj_2=0,pj_3=0,pj_4=0,pj_5=0,pj_6=0;
float toutle1=0,toutle2=0,toutle3=0,toutle4=0,toutle5=0,toutle6=0;


extern float X_slave_1ms_distance;
extern float X_slave_1ms_distance_SV;
extern float Z_slave_1ms_distance;

/************惯导解码程序************/
float AHRS_100A_Decode(u8 byte_1,u8 byte_2,u8 byte_3,float LSB)
{
	float data;
	s32 imid0,imid1,imid2,imid3;
	imid0=(byte_1)&0x7f;
	imid1=(byte_2)&0x7f;
	imid2=(byte_3)&0x3f;
	imid3=(imid2<<14)|(imid1<<7)|imid0;
	if(imid3&0X00080000)
	{
		imid3=imid3|0xfff00000;
	}
	data=(float)imid3*LSB;	
	return data;
}

/**************惯导数据提取**************/
int Gyro_i=0,Gyro_n=1;
void Gyroscope_data_Process(void)
{
	float GYRO_LBS=0.001;
//	float GYRO_LBS=0.00025;
	float POS_LBS=0.01;
	/****************************************///国产2号惯导
	Gyro_Read();
	if(Gyro2_data[0]==0x80)//帧头
	{
		if(Gyro2_data[29]==0xff)//帧尾
		{
			
			Gyro.Gyro_X=AHRS_100A_Decode(Gyro2_data[1],Gyro2_data[2],Gyro2_data[3],GYRO_LBS);
			Gyro.Gyro_Y=AHRS_100A_Decode(Gyro2_data[4],Gyro2_data[5],Gyro2_data[6],GYRO_LBS);
			Gyro.Gyro_Z=AHRS_100A_Decode(Gyro2_data[7],Gyro2_data[8],Gyro2_data[9],GYRO_LBS);
			
//			A_x_byte_1=gyro2_data[10];
//			A_x_byte_2=gyro2_data[11];
//			
//			
//			A_y_byte_1=gyro2_data[12];			
//			A_y_byte_2=gyro2_data[13];
//			
//			
//			A_z_byte_1=gyro2_data[14];
//			A_z_byte_2=gyro2_data[15];
//			
//			
//			temple_byte_1=gyro2_data[16];
//			temple_byte_2=gyro2_data[17];
//			
			Gyro.Pitch=AHRS_100A_Decode(Gyro2_data[18],Gyro2_data[19],Gyro2_data[20],POS_LBS);
			Gyro.Roll=AHRS_100A_Decode(Gyro2_data[21],Gyro2_data[22],Gyro2_data[23],POS_LBS);
			Gyro.Yaw=AHRS_100A_Decode(Gyro2_data[24],Gyro2_data[25],Gyro2_data[26],POS_LBS);
			
			
			
			X_slave_1ms_distance = Gyro.Gyro_Y * 0.001; //* (-0.001);             //镜子在1ms内走的路程，取反
			X_slave_1ms_distance_SV = X_slave_1ms_distance;                  //镜子动θ，光动2θ


			Z_slave_1ms_distance = Gyro.Gyro_Z * (-0.001f);
			
			
//			COUNT_byte_1=gyro2_data[27];
//			COUNT_byte_2=gyro2_data[28];
/*******************惯导测试程序********************/
//			Gyro_X_Save[Gyro_i]=Gyro.Gyro_X;
//			Gyro_Y_Save[Gyro_i]=Gyro.Gyro_Y;
//			Gyro_Z_Save[Gyro_i]=Gyro.Gyro_Z;
//			Pitch_Save[Gyro_i]=Gyro.Pitch;
//			Roll_Save[Gyro_i]=Gyro.Roll;
//			Yaw_Save[Gyro_i]=Gyro.Yaw;
			
/***************************************************/			
		}
	}
}

/**
  * @brief	读惯导数据
  * @data	  
  */
void Gyro_Read(void)
{
	u8 i;
	u32 readADDRTemp=0;
	readADDRTemp=Comm1_Addr;	
	for(i=0;i<30;i++)//国产2号30，ADI20
	{
		Gyro2_data[i]=*(uint32_t*)(readADDRTemp);
		readADDRTemp+=2;
	}		
}


/***********求均方根RMS***********/
float Gyro_RMS(float Xi[],int n)
{
	int i;
	float S=0;
	float X_mean=0,Temp=0;
	for(i=0;i<n;i++)
	{
		X_mean+=Xi[i];
	}
	X_mean/=n;
	
	for(i=0;i<n;i++)
	{
		Temp+=pow((Xi[i]-X_mean),2);
	}
	S=sqrt(Temp/n);
	return S;
}

