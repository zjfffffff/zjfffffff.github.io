#include "Comm1_Communication.h"
#include "PC_Communication.h"
#include "stm32f4xx.h"
#include "Camera_Config.h"
#include "AHRS_100A.h"
#include "timer.h"
#include "PC_Communication.h"
#include "ServoFunction.h"
#include "temperature.h"
#include  <math.h>



typedef struct
{
    float AzimuthDriveVoltage;							//方位驱动电压AN[6]
    float PitchDriveVoltage;								//俯仰驱动电压AN[6]
    float AzimuthDriverTemperature;					//方位温度TI
    float PitchDriverTemperature;						//俯仰温度TI
    float AzimuthDriverErrorCode;						//方位驱动器故障代码EC
    float PitchDriverErrorCode;							//俯仰驱动器故障代码EC
    float AzimuthMotorAPhaseCurrent;				//方位电机A相电流 AN[3]
    float PitchMotorAPhaseCurrent;					//俯仰电机A相电流 AN[3]
    float AzimuthMotorBPhaseCurrent;				//方位电机B相电流 AN[4]
    float PitchMotorBPhaseCurrent;					//俯仰电机B相电流 AN[4]
    float AzimuthMotorCPhaseCurrent;				//方位电机C相电流 AN[5]
    float PitchMotorCPhaseCurrent;					//俯仰电机C相电流 AN[5]
    float AzimuthMotorBusVoltage;						//方位电机模拟输入电压1 AN[1]
    float PitchMotorBusVoltage;							//俯仰电机模拟输入电压1 AN[1]
}AgreementCode;
AgreementCode Elmo_AgreementCode;



typedef union
{
	struct
	{
		float TI;
		float IQ;
		float EC;
		float MF;
		float SR;
		float EE_1;
		float AN_6;
		float AN_1;
		float AN_3;
		float AN_4;
		float AN_5;
		float PX;
		float UM;
		float VX;
		float AN_2;
	}Data_byte;
}Data_value;
Data_value Elmo_FW;

typedef union
{
	struct
	{
		float TI;
		float IQ;
		float EC;
		float MF;
		float SR;
		float EE_1;
		float AN_6;
		float AN_1;
		float AN_3;
		float AN_4;
		float AN_5;
		float PX;
		float UM;
		float VX;
		float AN_2;
	}Data_byte;
}Data_value1;
Data_value1 Elmo_FY;




u8  temperature_Comm1_TXD[temperature_Comm1_TX_Len],temperature_Comm1_RXD[50];
u8  Data_Comm1_TXD[Data_Comm1_TX_Len];

u8  FY_Comm_TXD[FY_Comm_TX_Len],FY_Comm_RXD[50];
u8  FY_Data_Comm_TXD[FY_Data_Comm_TX_Len];




float temperature_l=0;
float temperature_2=0;
float temperature_3=0;
float temperature_4=0;


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////方位

//温度TI
void FW_temperature_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x54;
		temperature_Comm1_TXD[1]=0x49;
		temperature_Comm1_TXD[2]=0x3B;

		FW_temperature_Comm1_Com_Write();
}


//电流IQ
void FW_Current_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x49;
		temperature_Comm1_TXD[1]=0x51;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}

//驱动器故障代码EC
void FW_Drive_Fault_Code_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x45;
		temperature_Comm1_TXD[1]=0x43;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}


//电机故障代码MF
void FW_Motor_Fault_Code_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x4D;
		temperature_Comm1_TXD[1]=0x46;
		temperature_Comm1_TXD[2]=0x3B;

		FW_temperature_Comm1_Com_Write();
}


//状态寄存器SR
void FW_State_Register_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x53;
		temperature_Comm1_TXD[1]=0x52;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}


//编码器状态EE[1]
void FW_Encoder_Status_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x45;
		Data_Comm1_TXD[1]=0x45;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x31;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
		FW_Data_Comm1_Com_Write(); 
}


//驱动电压AN[6]
void FW_Slaving_Voltage_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x36;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
		FW_Data_Comm1_Com_Write();
}


//电机模拟输入电压1 AN[1]
void FW_Motor_Analog_Input_Voltage_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x31;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
		FW_Data_Comm1_Com_Write();
}


//电机A相电流 AN[3]
void FW_Motor_A_Current_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x33;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
		FW_Data_Comm1_Com_Write();
}


//电机B相电流 AN[4]
void FW_Motor_B_Current_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x34;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
		FW_Data_Comm1_Com_Write();
}


//电机C相电流 AN[5]
void FW_Motor_C_Current_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x35;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;  
	
	  FW_Data_Comm1_Com_Write();
}


//Elmo编码器位置PX
void FW_Encoder_Position_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x50;
		temperature_Comm1_TXD[1]=0x58;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}

//用户模式UM
void FW_User_Mode_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x55;
		temperature_Comm1_TXD[1]=0x4D;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}

//速度VX
void FW_Speed_Comm1_Data_Pack(void)
{
		temperature_Comm1_TXD[0]=0x56;
		temperature_Comm1_TXD[1]=0x58;
		temperature_Comm1_TXD[2]=0x3B;
	
		FW_temperature_Comm1_Com_Write();
}

//电机模拟输入电压2 AN[2]
void FW_Motor_Analog_Input2_Voltage_Comm1_Data_Pack(void)
{
		Data_Comm1_TXD[0]=0x41;
		Data_Comm1_TXD[1]=0x4E;
		Data_Comm1_TXD[2]=0x5B;
		Data_Comm1_TXD[3]=0x32;
		Data_Comm1_TXD[4]=0x5D;  
		Data_Comm1_TXD[5]=0x3B;   
	
	  FW_Data_Comm1_Com_Write();
}



u8 PC_i_i=0;
u8 PC_i_i1=0;
float Current[10]={0};


//读取数据
float FW_temperature_Comm1_Com_Read()
{
//	//	float Current[10]={0};
//		float Current1=0;
//		float Current2=0;
//		float Current3=0;
//		float Current4=0;
//		float Current5=0;
//		float Current6=0;
//		float Current7=0;

		u8 PC_i=0;
		u8 i=0;
		u32 Comm1_ADDR_tmp;
		Comm1_ADDR_tmp=Comm1_ADDR;
		for(PC_i=0; PC_i<temperature_Comm1_RX_Len; PC_i++)
		{
				temperature_Comm1_RXD[PC_i]=*(uint32_t*)(Comm1_ADDR_tmp);
				Comm1_ADDR_tmp = Comm1_ADDR_tmp+2;
		}
		
		temperature_l=0;
		temperature_2=0;
		PC_i_i=0;
		PC_i_i1=0;
		for(i=0;i<10;i++)
		{
				Current[i]=0;
		}
		
		
		//温度TI读取
		if((temperature_Comm1_RXD[0]==0x54) & (temperature_Comm1_RXD[1]==0x49)&(temperature_Comm1_RXD[2]==0x3B))//帧头
		{
				if(temperature_Comm1_RXD[3]==0x2D)//负号-
				{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i-1;
									break;
							}
					}
					for(i=4;i<PC_i_i;i++)
					{
							Current[i-4]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l-Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.TI=temperature_l;
					Elmo_AgreementCode.AzimuthDriverTemperature=temperature_l;
				}
				else
				{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.TI=temperature_l;
					Elmo_AgreementCode.AzimuthDriverTemperature=temperature_l;
				}
		}
		//电流IQ读取
		else if((temperature_Comm1_RXD[0]==0x49) & (temperature_Comm1_RXD[1]==0x51)&(temperature_Comm1_RXD[2]==0x3B))
		{
				if(temperature_Comm1_RXD[3]==0x2D)//负号-
				{
						for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
										break;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
								for(i=4;i<PC_i_i;i++)
								{
										Current[i-4]=temperature_Comm1_RXD[i]&0x0f;
								}
								for(i=0;i<PC_i_i-4;i++)//PC_i_i=5;
								{
										temperature_l=temperature_l-Current[i]*pow(10,PC_i_i-5-i);
								}
								
								for(i=PC_i_i+1;i<PC_i_i1;i++)
								{
										Current[i]=temperature_Comm1_RXD[i]&0x0f;
								}
								for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
								{
										temperature_2=(float)(temperature_2-Current[i]*pow(0.1,i-PC_i_i));
								}
								Elmo_FW.Data_byte.IQ=(float)(temperature_l+temperature_2);
						}
				}
				else
				{
						for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=3;i<PC_i_i;i++)
						{
								Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.IQ=temperature_l;
				}
		}
		//驱动器故障代码EC读取
		else if((temperature_Comm1_RXD[0]==0x45) & (temperature_Comm1_RXD[1]==0x43)&(temperature_Comm1_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.EC=temperature_l;
					Elmo_AgreementCode.AzimuthDriverErrorCode=temperature_l;

		}
		//电机故障代码MF读取
		else if((temperature_Comm1_RXD[0]==0x4D) & (temperature_Comm1_RXD[1]==0x46)&(temperature_Comm1_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.MF=temperature_l;
		}
		//状态寄存器SR读取
		else if((temperature_Comm1_RXD[0]==0x53) & (temperature_Comm1_RXD[1]==0x52)&(temperature_Comm1_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.SR=temperature_l;
		}
		//编码器状态EE[1]读取
		else if((temperature_Comm1_RXD[0]==0x45) & (temperature_Comm1_RXD[1]==0x45)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x31)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{
					for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=6;i<PC_i_i;i++)
					{
							Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-6;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
					}
					Elmo_FW.Data_byte.EE_1=temperature_l;
				//	Elmo_AgreementCode.AzimuthDriveVoltage=temperature_l;
		}
		//驱动电压AN[6]读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x36)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{
					for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x2E)
							{
									PC_i_i=PC_i;
									break;
							}
							else if(PC_i_i==0)
							{
									if(temperature_Comm1_RXD[PC_i]==0x3B)
									{
											PC_i_i=PC_i;
											break;
									}
							}
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i1=PC_i;
									break;
							}
					}
					for(i=6;i<PC_i_i;i++)
					{
							Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-6;i++)
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
					}
					
					for(i=PC_i_i+1;i<PC_i_i1;i++)
					{
							Current[i]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
					{
							temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
					}
					temperature_l=(float)(temperature_l+temperature_2);
					Elmo_FW.Data_byte.AN_6=temperature_l;
					Elmo_AgreementCode.AzimuthDriveVoltage=temperature_l;
		}
		//电机模拟输入电压AN[1] 读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x31)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{
				if(temperature_Comm1_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(-temperature_l-temperature_2);
						Elmo_FW.Data_byte.AN_1=temperature_l;
						Elmo_AgreementCode.AzimuthMotorBusVoltage=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.AN_1=temperature_l;
						Elmo_AgreementCode.AzimuthMotorBusVoltage=temperature_l;
				}		
		}
		//电机A相电流 AN[3] 读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x33)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{
				if(temperature_Comm1_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(-temperature_l-temperature_2);
						Elmo_FW.Data_byte.AN_3=temperature_l;
						Elmo_AgreementCode.AzimuthMotorAPhaseCurrent=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.AN_3=temperature_l;
				}
		}
		//电机B相电流 AN[4] 读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x34)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{	
				if(temperature_Comm1_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(-temperature_l-temperature_2);
						Elmo_FW.Data_byte.AN_4=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.AN_4=temperature_l;
				}		
		}
		//电机C相电流 AN[5] 读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x35)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{	
				if(temperature_Comm1_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(-temperature_l-temperature_2);
						Elmo_FW.Data_byte.AN_5=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.AN_5=temperature_l;
				}		
		}
		//Elmo编码器位置PX
		else if((temperature_Comm1_RXD[0]==0x50) & (temperature_Comm1_RXD[1]==0x58)&(temperature_Comm1_RXD[2]==0x3B))
		{
				for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
				{
						if(temperature_Comm1_RXD[PC_i]==0x3B)
						{
								PC_i_i=PC_i;
								break;
						}
				}
				for(i=3;i<PC_i_i;i++)
				{
						Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
				}
				for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
				{
						temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
				}
				Elmo_FW.Data_byte.PX=temperature_l;
		}
		//用户模式UM
		else if((temperature_Comm1_RXD[0]==0x55) & (temperature_Comm1_RXD[1]==0x4D)&(temperature_Comm1_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FW.Data_byte.UM=temperature_l;
		}
		//速度VX
		else if((temperature_Comm1_RXD[0]==0x56) & (temperature_Comm1_RXD[1]==0x58)&(temperature_Comm1_RXD[2]==0x3B))
		{
				if(temperature_Comm1_RXD[3]==0x2D)//负号-
				{
						for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=4;i<PC_i_i;i++)
						{
								Current[i-4]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-4;i++)//PC_i_i=5;
						{
								temperature_l=temperature_l-Current[i]*pow(10,PC_i_i-5-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2-Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.VX=temperature_l;
				}
				else
				{
						for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=3;i<PC_i_i;i++)
						{
								Current[i-3]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-4-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.VX=temperature_l;
				}
		}
		//电机模拟输入电压2 AN[2] 读取
		else if((temperature_Comm1_RXD[0]==0x41) & (temperature_Comm1_RXD[1]==0x4E)&(temperature_Comm1_RXD[2]==0x5B)&(temperature_Comm1_RXD[3]==0x32)&(temperature_Comm1_RXD[4]==0x5D)&(temperature_Comm1_RXD[5]==0x3B))
		{
				if(temperature_Comm1_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(-temperature_l-temperature_2);
						Elmo_FW.Data_byte.AN_2=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<temperature_Comm1_RX_Len;PC_i++)
						{
								if(temperature_Comm1_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(temperature_Comm1_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_l=temperature_l+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=temperature_Comm1_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_2=(float)(temperature_2+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_l=(float)(temperature_l+temperature_2);
						Elmo_FW.Data_byte.AN_2=temperature_l;
				}
					
		}
		
		return temperature_l;
}

void FW_temperature_Comm1_Com_Write()
{
		u8 PC_i=0;
	//	u32 Comm1_ADDR_tmp;
		for(PC_i=0; PC_i<temperature_Comm1_TX_Len; PC_i++)
		{
			*(uint32_t*)(Comm1_ADDR)= temperature_Comm1_TXD[PC_i];
		}
		*(uint32_t*)(Comm1_Com_En)= 1;
}


void FW_Data_Comm1_Com_Write()
{
		u8 PC_i=0;
	//	u32 Comm1_ADDR_tmp;
		for(PC_i=0; PC_i<Data_Comm1_TX_Len; PC_i++)
		{
			*(uint32_t*)(Comm1_ADDR)= Data_Comm1_TXD[PC_i];
		}
		*(uint32_t*)(Comm1_Com_En)= 1;
}






///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/////俯仰

//温度TI
void FY_temperature_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x54;
		FY_Comm_TXD[1]=0x49;
		FY_Comm_TXD[2]=0x3B;

		FY_temperature_Comm1_Com_Write();
}


//电流IQ
void FY_Current_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x49;
		FY_Comm_TXD[1]=0x51;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}

//驱动器故障代码EC
void FY_Drive_Fault_Code_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x45;
		FY_Comm_TXD[1]=0x43;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}


//电机故障代码MF
void FY_Motor_Fault_Code_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x4D;
		FY_Comm_TXD[1]=0x46;
		FY_Comm_TXD[2]=0x3B;

		FY_temperature_Comm1_Com_Write();
}


//状态寄存器SR
void FY_State_Register_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x53;
		FY_Comm_TXD[1]=0x52;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}


//编码器状态EE[1]
void FY_Encoder_Status_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x45;
		FY_Data_Comm_TXD[1]=0x45;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x31;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
		FY_Data_Comm1_Com_Write(); 
}


//驱动电压AN[6]
void FY_Slaving_Voltage_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x36;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
		FY_Data_Comm1_Com_Write();
}


//电机模拟输入电压1 AN[1]
void FY_Motor_Analog_Input_Voltage_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x31;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
		FY_Data_Comm1_Com_Write();
}


//电机A相电流 AN[3]
void FY_Motor_A_Current_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x33;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
		FY_Data_Comm1_Com_Write();
}


//电机B相电流 AN[4]
void FY_Motor_B_Current_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x34;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
		FY_Data_Comm1_Com_Write();
}


//电机B相电流 AN[5]
void FY_Motor_C_Current_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x35;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;  
	
	  FY_Data_Comm1_Com_Write();
}


//Elmo编码器位置PX
void FY_Encoder_Position_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x50;
		FY_Comm_TXD[1]=0x58;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}

//用户模式UM
void FY_User_Mode_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x55;
		FY_Comm_TXD[1]=0x4D;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}

//速度VX
void FY_Speed_Comm1_Data_Pack(void)
{
		FY_Comm_TXD[0]=0x56;
		FY_Comm_TXD[1]=0x58;
		FY_Comm_TXD[2]=0x3B;
	
		FY_temperature_Comm1_Com_Write();
}

//电机模拟输入电压2 AN[2]
void FY_Motor_Analog_Input2_Voltage_Comm1_Data_Pack(void)
{
		FY_Data_Comm_TXD[0]=0x41;
		FY_Data_Comm_TXD[1]=0x4E;
		FY_Data_Comm_TXD[2]=0x5B;
		FY_Data_Comm_TXD[3]=0x32;
		FY_Data_Comm_TXD[4]=0x5D;  
		FY_Data_Comm_TXD[5]=0x3B;   
	
	  FY_Data_Comm1_Com_Write();
}



//读取数据
float FY_temperature_Comm1_Com_Read()
{
		u8 PC_i=0;
		u8 i=0;
		u32 Comm1_ADDR_tmp;
		Comm1_ADDR_tmp=Data_Comm1_ADDR; 
		for(PC_i=0; PC_i<FY_Comm_RX_Len; PC_i++)
		{
				FY_Comm_RXD[PC_i]=*(uint32_t*)(Comm1_ADDR_tmp);
				Comm1_ADDR_tmp = Comm1_ADDR_tmp+2;
		}
		
		temperature_3=0;
		temperature_4=0;
		PC_i_i=0;
		PC_i_i1=0;
		for(i=0;i<10;i++)
		{
				Current[i]=0;
		}
		
		
		//温度TI读取
		if((FY_Comm_RXD[0]==0x54) & (FY_Comm_RXD[1]==0x49)&(FY_Comm_RXD[2]==0x3B))//帧头
		{
				if(FY_Comm_RXD[3]==0x2D)//负号-
				{
					for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i-1;
									break;
							}
					}
					for(i=4;i<PC_i_i;i++)
					{
							Current[i-4]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_l=temperature_l-Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.TI=temperature_l;
					Elmo_AgreementCode.PitchDriverTemperature=temperature_l;
				}
				else
				{
					for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(temperature_Comm1_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.TI=temperature_3;
					Elmo_AgreementCode.PitchDriverTemperature=temperature_3;
				}
		}
		//电流IQ读取
		else if((FY_Comm_RXD[0]==0x49) & (FY_Comm_RXD[1]==0x51)&(FY_Comm_RXD[2]==0x3B))
		{
				if(FY_Comm_RXD[3]==0x2D)//负号-
				{
						for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
								for(i=4;i<PC_i_i;i++)
								{
										Current[i-4]=FY_Comm_RXD[i]&0x0f;
								}
								for(i=0;i<PC_i_i-4;i++)//PC_i_i=5;
								{
										temperature_3=temperature_3-Current[i]*pow(10,PC_i_i-5-i);
								}
								
								for(i=PC_i_i+1;i<PC_i_i1;i++)
								{
										Current[i]=FY_Comm_RXD[i]&0x0f;
								}
								for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
								{
										temperature_4=(float)(temperature_4-Current[i]*pow(0.1,i-PC_i_i));
								}
								Elmo_FY.Data_byte.IQ=(float)(temperature_3+temperature_4);
						}
				}
				else
				{
						for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=3;i<PC_i_i;i++)
						{
								Current[i-3]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.IQ=temperature_3;
				}
		}
		//驱动器故障代码EC读取
		else if((FY_Comm_RXD[0]==0x45) & (FY_Comm_RXD[1]==0x43)&(FY_Comm_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.EC=temperature_3;
					Elmo_AgreementCode.PitchDriverErrorCode=temperature_3;

		}
		//电机故障代码MF读取
		else if((FY_Comm_RXD[0]==0x4D) & (FY_Comm_RXD[1]==0x46)&(FY_Comm_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.MF=temperature_3;
		}
		//状态寄存器SR读取
		else if((FY_Comm_RXD[0]==0x53) & (FY_Comm_RXD[1]==0x52)&(FY_Comm_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.SR=temperature_3;
		}
		//编码器状态EE[1]读取
		else if((FY_Comm_RXD[0]==0x45) & (FY_Comm_RXD[1]==0x45)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x31)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{
					for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=6;i<PC_i_i;i++)
					{
							Current[i-6]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-6;i++)//PC_i_i=5;
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
					}
					Elmo_FY.Data_byte.EE_1=temperature_3;
				//	Elmo_AgreementCode.AzimuthDriveVoltage=temperature_l;
		}
		//驱动电压AN[6]读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x36)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{
					for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x2E)
							{
									PC_i_i=PC_i;
							}
							else if(PC_i_i==0)
							{
									if(FY_Comm_RXD[PC_i]==0x3B)
									{
											PC_i_i=PC_i;
											break;
									}
							}
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i1=PC_i;
									break;
							}
					}
					for(i=6;i<PC_i_i;i++)
					{
							Current[i-6]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-6;i++)
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
					}
					
					for(i=PC_i_i+1;i<PC_i_i1;i++)
					{
							Current[i]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
					{
							temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
					}
					temperature_3=(float)(temperature_3+temperature_4);
					Elmo_FY.Data_byte.AN_6=temperature_3;
					Elmo_AgreementCode.PitchDriveVoltage=temperature_3;
		}
		//电机模拟输入电压AN[1] 读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x31)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{
				if(FY_Comm_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(-temperature_3-temperature_4);
						Elmo_FY.Data_byte.AN_1=temperature_3;
						Elmo_AgreementCode.PitchMotorBusVoltage=temperature_3;
				}
				else
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.AN_1=temperature_3;
						Elmo_AgreementCode.PitchMotorBusVoltage=temperature_3;
				}		
		}
		//电机A相电流 AN[3] 读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x33)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{
				if(FY_Comm_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(-temperature_3-temperature_4);
						Elmo_FY.Data_byte.AN_3=temperature_3;
						Elmo_AgreementCode.PitchMotorAPhaseCurrent=temperature_3;
				}
				else
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.AN_3=temperature_3;
						Elmo_AgreementCode.PitchMotorAPhaseCurrent=temperature_3;
				}
		}
		//电机B相电流 AN[4] 读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x34)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{	
				if(FY_Comm_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(-temperature_3-temperature_4);
						Elmo_FY.Data_byte.AN_4=temperature_3;
						Elmo_AgreementCode.PitchMotorBPhaseCurrent=temperature_3;
				}
				else
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.AN_4=temperature_3;
						Elmo_AgreementCode.PitchMotorBPhaseCurrent=temperature_3;
				}		
		}
		//电机C相电流 AN[5] 读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x35)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{	
				if(FY_Comm_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(-temperature_3-temperature_4);
						Elmo_FY.Data_byte.AN_5=temperature_3;
						Elmo_AgreementCode.PitchMotorCPhaseCurrent=temperature_3;
				}
				else
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.AN_5=temperature_3;
						Elmo_AgreementCode.PitchMotorCPhaseCurrent=temperature_3;
				}		
		}
		//Elmo编码器位置PX
		else if((FY_Comm_RXD[0]==0x50) & (FY_Comm_RXD[1]==0x58)&(FY_Comm_RXD[2]==0x3B))
		{
				for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
				{
						if(FY_Comm_RXD[PC_i]==0x3B)
						{
								PC_i_i=PC_i;
								break;
						}
				}
				for(i=3;i<PC_i_i;i++)
				{
						Current[i-3]=FY_Comm_RXD[i]&0x0f;
				}
				for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
				{
						temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
				}
				Elmo_FY.Data_byte.PX=temperature_3;
		}
		//用户模式UM
		else if((FY_Comm_RXD[0]==0x55) & (FY_Comm_RXD[1]==0x4D)&(FY_Comm_RXD[2]==0x3B))
		{
					for(PC_i=3;PC_i<temperature_Comm1_RX_Len;PC_i++)
					{
							if(FY_Comm_RXD[PC_i]==0x3B)
							{
									PC_i_i=PC_i;
									break;
							}
					}
					for(i=3;i<PC_i_i;i++)
					{
							Current[i-3]=FY_Comm_RXD[i]&0x0f;
					}
					for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
					{
							temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
					}
					Elmo_FY.Data_byte.UM=temperature_3;
		}
		//速度VX
		else if((FY_Comm_RXD[0]==0x56) & (FY_Comm_RXD[1]==0x58)&(FY_Comm_RXD[2]==0x3B))
		{
				if(FY_Comm_RXD[3]==0x2D)//负号-
				{
						for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=4;i<PC_i_i;i++)
						{
								Current[i-4]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-4;i++)//PC_i_i=5;
						{
								temperature_3=temperature_3-Current[i]*pow(10,PC_i_i-5-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4-Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.VX=temperature_3;
				}
				else
				{
						for(PC_i=3;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=3;i<PC_i_i;i++)
						{
								Current[i-3]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-3;i++)//PC_i_i=5;
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-4-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.VX=temperature_3;
				}
		}
		//电机模拟输入电压2 AN[2] 读取
		else if((FY_Comm_RXD[0]==0x41) & (FY_Comm_RXD[1]==0x4E)&(FY_Comm_RXD[2]==0x5B)&(FY_Comm_RXD[3]==0x32)&(FY_Comm_RXD[4]==0x5D)&(FY_Comm_RXD[5]==0x3B))
		{
				if(FY_Comm_RXD[6]==0x2D)//负号-
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(FY_Comm_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=7;i<PC_i_i;i++)
						{
								Current[i-7]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-7;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-8-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(-temperature_3-temperature_4);
						Elmo_FY.Data_byte.AN_2=temperature_3;
						//Elmo_AgreementCode.AzimuthMotorBusVoltage=temperature_l;
				}
				else
				{
						for(PC_i=6;PC_i<FY_Comm_RX_Len;PC_i++)
						{
								if(FY_Comm_RXD[PC_i]==0x2E)
								{
										PC_i_i=PC_i;
								}
								else if(PC_i_i==0)
								{
										if(FY_Comm_RXD[PC_i]==0x3B)
										{
												PC_i_i=PC_i;
												break;
										}
								}
								if(temperature_Comm1_RXD[PC_i]==0x3B)
								{
										PC_i_i1=PC_i;
										break;
								}
						}
						for(i=6;i<PC_i_i;i++)
						{
								Current[i-6]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=0;i<PC_i_i-6;i++)
						{
								temperature_3=temperature_3+Current[i]*pow(10,PC_i_i-7-i);
						}
						
						for(i=PC_i_i+1;i<PC_i_i1;i++)
						{
								Current[i]=FY_Comm_RXD[i]&0x0f;
						}
						for(i=PC_i_i+1;i<PC_i_i1;i++)//PC_i_i=5;
						{
								temperature_4=(float)(temperature_4+Current[i]*pow(0.1,i-PC_i_i));
						}
						temperature_3=(float)(temperature_3+temperature_4);
						Elmo_FY.Data_byte.AN_2=temperature_3;
						//Elmo_AgreementCode.AzimuthMotorBusVoltage=temperature_l;
				}
					
		}
		
		return temperature_3;
}

void FY_temperature_Comm1_Com_Write()
{
		u8 PC_i=0;
	//	u32 Comm1_ADDR_tmp;
		for(PC_i=0; PC_i<temperature_Comm1_TX_Len; PC_i++)
		{
			*(uint32_t*)(Data_Comm1_ADDR)= temperature_Comm1_TXD[PC_i];
		}
		*(uint32_t*)(Data_Comm1_Com_En)= 1;
}


void FY_Data_Comm1_Com_Write()
{
		u8 PC_i=0;
	//	u32 Comm1_ADDR_tmp;
		for(PC_i=0; PC_i<Data_Comm1_TX_Len; PC_i++)
		{
			*(uint32_t*)(Data_Comm1_ADDR)= Data_Comm1_TXD[PC_i];
		}
		*(uint32_t*)(Data_Comm1_Com_En)= 1;
}





/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
//////函数写

int Elmo_Write_Data(u8 prs,u8 biao_zhi)
{
		switch(prs)
		{	
			
				case 1:
									switch(biao_zhi)
									{
										case 1:FW_temperature_Comm1_Data_Pack();        break;						//温度TI	
										case 2:FW_Current_Comm1_Data_Pack();             break;						//电流IQ
										case 3:FW_Drive_Fault_Code_Comm1_Data_Pack();    break;						//驱动器故障代码EC
										case 4:FW_Motor_Fault_Code_Comm1_Data_Pack();    break;						//电机故障代码MF
										case 5:FW_State_Register_Comm1_Data_Pack();      break;						//状态寄存器SR
										case 6:FW_Encoder_Status_Comm1_Data_Pack();      break;						//编码器状态EE[1]
										case 7:FW_Slaving_Voltage_Comm1_Data_Pack();     break;						//驱动电压AN[6]
										case 8:FW_Motor_Analog_Input_Voltage_Comm1_Data_Pack();  break;		//电机模拟输入电压1 AN[1]
										case 9:FW_Motor_A_Current_Comm1_Data_Pack();     break;						//电机A相电流 AN[3]
										case 10:FW_Motor_B_Current_Comm1_Data_Pack();    break;						//电机B相电流 AN[4]
										case 11:FW_Motor_C_Current_Comm1_Data_Pack();    break;						//电机C相电流 AN[5]
										case 12:FW_Encoder_Position_Comm1_Data_Pack();   break;						//Elmo编码器位置PX
										case 13:FW_User_Mode_Comm1_Data_Pack();   break;										//用户模式UM
										case 14:FW_Speed_Comm1_Data_Pack();   break;												//速度VX
										case 15:FW_Motor_Analog_Input2_Voltage_Comm1_Data_Pack();   break;	//电机模拟输入电压2 AN[2]
									}
									break;
				case 2:
									switch(biao_zhi)
									{
										case 1:FY_temperature_Comm1_Data_Pack();      biao_zhi=0;	 break;									//温度	
										case 2:FY_Current_Comm1_Data_Pack();          biao_zhi=0;	 break;											//电流IQ
										case 3:FY_Drive_Fault_Code_Comm1_Data_Pack(); biao_zhi=0;  break;						//驱动器故障代码EC
										case 4:FY_Motor_Fault_Code_Comm1_Data_Pack(); biao_zhi=0;  break;						//电机故障代码MF
										case 5:FY_State_Register_Comm1_Data_Pack();   biao_zhi=0;  break;							//状态寄存器SR
										case 6:FY_Encoder_Status_Comm1_Data_Pack();   biao_zhi=0;  break;							//编码器状态EE[1]
										case 7:FY_Slaving_Voltage_Comm1_Data_Pack();  biao_zhi=0;  break;						  //驱动电压AN[6]
										case 8:FY_Motor_Analog_Input_Voltage_Comm1_Data_Pack();biao_zhi=0;  break;	//电机模拟输入电压1 AN[1]
										case 9:FY_Motor_A_Current_Comm1_Data_Pack();  biao_zhi=0;  break;						  //电机A相电流 AN[3]
										case 10:FY_Motor_C_Current_Comm1_Data_Pack(); biao_zhi=0;  break;						//电机B相电流 AN[4]
										case 11:FY_Motor_C_Current_Comm1_Data_Pack(); biao_zhi=0;  break;						//电机C相电流 AN[5]
										case 12:FY_Encoder_Position_Comm1_Data_Pack(); biao_zhi=0; break;						//Elmo编码器位置PX
										case 13:FY_User_Mode_Comm1_Data_Pack(); biao_zhi=0;  break;										//用户模式UM
										case 14:FY_Speed_Comm1_Data_Pack(); biao_zhi=0;  break;												//速度VX
										case 15:FY_Motor_Analog_Input2_Voltage_Comm1_Data_Pack(); biao_zhi=0;  break;	//电机模拟输入电压2 AN[2]
									}
									break;	
		}
}



/*
//判断编码器的圈数		
		if(count_flag == 1)
		{
//			read_FY_encoder();
				
				if(count_f==1)
				{
						count_1++;
				}
				if(count_1 == 1 )
				{
						FY_encoder_degrees_First=FY_encoder_degrees;//俯仰编码器转换为角度
				}
				else if(count_1 == 2 )
				{
						FY_encoder_degrees_Second = FY_encoder_degrees;
						FY_Difference = FY_encoder_degrees_First - FY_encoder_degrees_Second;	
						if(FY_Difference > -200.0 && FY_Difference < 200.0)
						{
								k_count = k_count;
						}
						else if(FY_Difference < -200.0)
						{
								k_count ++;
						}
						else if(FY_Difference > 200.0)
						{
								k_count --;
						}
						
						FY_encoder_degrees_First=FY_encoder_degrees_Second;
						count_flag=0;
						count_f=0;
				}
				
				count_1 = 0;
		}

*/