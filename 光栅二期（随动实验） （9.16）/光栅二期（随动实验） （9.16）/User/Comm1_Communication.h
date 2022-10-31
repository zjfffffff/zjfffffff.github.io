#ifndef _Comm1_Communication_H
#define _Comm1_Communication_H


#include "stm32f4xx.h"



#define Comm1_ADDR             ((u32)(0x64008000))//1000
#define Comm1_Com_En           ((u32)(0x64008400))//2400


#define Data_Comm1_ADDR             ((u32)(0x64001000))
#define Data_Comm1_Com_En          	((u32)(0x64002400))


#define Data_Comm1_TX_Len 6
#define temperature_Comm1_TX_Len 3
#define temperature_Comm1_RX_Len 30

#define FY_Data_Comm_TX_Len 6
#define FY_Comm_TX_Len 3
#define FY_Comm_RX_Len 16


//·½Î»
float FW_temperature_Comm1_Com_Read();
void FW_temperature_Comm1_Com_Write();
void FW_Data_Comm1_Com_Write();

void FW_temperature_Comm1_Data_Pack(void);
void FW_Current_Comm1_Data_Pack(void);
void FW_Drive_Fault_Code_Comm1_Data_Pack(void);
void FW_Motor_Fault_Code_Comm1_Data_Pack(void);
void FW_State_Register_Comm1_Data_Pack(void);
void FW_Encoder_Status_Comm1_Data_Pack(void);
void FW_Slaving_Voltage_Comm1_Data_Pack(void);
void FW_Motor_Analog_Input_Voltage_Comm1_Data_Pack(void);
void FW_Motor_A_Current_Comm1_Data_Pack(void);
void FW_Motor_B_Current_Comm1_Data_Pack(void);
void FW_Motor_C_Current_Comm1_Data_Pack(void);
void FW_Encoder_Position_Comm1_Data_Pack(void);
void FW_User_Mode_Comm1_Data_Pack(void);
void FW_Speed_Comm1_Data_Pack(void);
void FW_Motor_Analog_Input2_Voltage_Comm1_Data_Pack(void);



//¸©Ñö
float FY_temperature_Comm1_Com_Read();
void FY_temperature_Comm1_Com_Write();
void FY_Data_Comm1_Com_Write();

void FY_temperature_Comm1_Data_Pack(void);
void FY_Current_Comm1_Data_Pack(void);
void FY_Drive_Fault_Code_Comm1_Data_Pack(void);
void FY_Motor_Fault_Code_Comm1_Data_Pack(void);
void FY_State_Register_Comm1_Data_Pack(void);
void FY_Encoder_Status_Comm1_Data_Pack(void);
void FY_Slaving_Voltage_Comm1_Data_Pack(void);
void FY_Motor_Analog_Input_Voltage_Comm1_Data_Pack(void);
void FY_Motor_A_Current_Comm1_Data_Pack(void);
void FY_Motor_B_Current_Comm1_Data_Pack(void);
void FY_Motor_C_Current_Comm1_Data_Pack(void);
void FY_Encoder_Position_Comm1_Data_Pack(void);
void FY_User_Mode_Comm1_Data_Pack(void);
void FY_Speed_Comm1_Data_Pack(void);
void FY_Motor_Analog_Input2_Voltage_Comm1_Data_Pack(void);



//Ð´ÃüÁî
int Elmo_Write_Data(u8 prs,u8 biao_zhi);



#endif

