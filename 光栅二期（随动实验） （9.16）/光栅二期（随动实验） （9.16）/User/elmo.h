#ifndef __ELMO_H
#define __ELMO_H	 
#include "stm32f4xx.h"

void wRS_FY(void);
void wUM(uint16_t CAN_ID,uint8_t CAN_DATA4);
void rUM(uint16_t CAN_ID);
void rMO(void);
void wMO_FY(uint16_t CAN_ID,uint8_t CAN_DATA4);
void rPA(uint16_t CAN_ID);
void wPA_FY(uint16_t CAN_ID,int32_t position);
void wPR_FY(uint16_t CAN_ID,int32_t position);
void wPX_FY(uint16_t CAN_ID,int32_t position);
void wJV(uint16_t CAN_ID,int32_t speed);
void rSP(void);
void wSP_FY(uint16_t CAN_ID,int32_t speed);
void wAC_FY(uint16_t CAN_ID,int32_t acceleration);
void wBG_FY(uint16_t CAN_ID);
void wST_FY(uint16_t CAN_ID);
void rPX_FY(uint16_t CAN_ID);
void rHM(uint16_t CAN_ID,uint8_t INDEX);
void wHM(uint16_t CAN_ID,uint8_t INDEX,uint8_t CAN_DATA4);
void rVX(uint16_t CAN_ID);
void wTC(uint16_t CAN_ID,float cur);
void wCL(uint16_t CAN_ID,uint8_t INDEX,float cur);
void wPL(uint16_t CAN_ID,uint8_t INDEX,float cur);
void rVE(uint16_t CAN_ID);
void rPE(uint16_t CAN_ID);
void rAC(uint16_t CAN_ID);
void rIQ(uint16_t CAN_ID);
void rIQ_FW(uint16_t CAN_ID);    
void wSD_FY(uint16_t CAN_ID,int32_t sd_acceleration);
void wDC_FY(uint16_t CAN_ID,int32_t acceleration);
void wSF_FY(uint16_t CAN_ID,int32_t sf_value);
uint32_t f_to_i(float x);

void wRS_FW(void);
void wPA_FW(uint16_t CAN_ID,int32_t position);
void wBG_FW(uint16_t CAN_ID);
void wMO_FW(uint16_t CAN_ID,uint8_t CAN_DATA4);
void wSP_FW(uint16_t CAN_ID,int32_t speed);  
void rPX_FW(uint16_t CAN_ID);
void wPX_FW(uint16_t CAN_ID,int32_t position);
void wSD_FW(uint16_t CAN_ID,int32_t sd_acceleration);
void wAC_FW(uint16_t CAN_ID,int32_t acceleration);
void wDC_FW(uint16_t CAN_ID,int32_t acceleration);
void wSF_FW(uint16_t CAN_ID,int32_t sf_value); 
void wST_FW(uint16_t CAN_ID);   
void wPR_FW(uint16_t CAN_ID,int32_t position);
void WD_FW_Read(uint16_t CAN_ID);
void WD_FY_Read(uint16_t CAN_ID);
void EC_FWRead(uint16_t CAN_ID);
void MF_FWRead(uint16_t CAN_ID);
void SR_FWRead(uint16_t CAN_ID);
void AN_1_FWRead(uint16_t CAN_ID);
void AN_2_FWRead(uint16_t CAN_ID);
void AN_3_FWRead(uint16_t CAN_ID); 
void AN_4_FWRead(uint16_t CAN_ID);
void EE_1_FWRead(uint16_t CAN_ID);  
void TI_1_FWRead(uint16_t CAN_ID);
void PX_FWRead(uint16_t CAN_ID);
void PY_FWRead(uint16_t CAN_ID);
void VX_FWRead(uint16_t CAN_ID);
void SO_FWRead(uint16_t CAN_ID);

void EC_FYRead(uint16_t CAN_ID);   //ELMO错误代码
void MF_FYRead(uint16_t CAN_ID);   //电机错误代码
void SR_FYRead(uint16_t CAN_ID);   //状态寄存器
void AN_1_FYRead(uint16_t CAN_ID);   //实时电流A相   
void AN_2_FYRead(uint16_t CAN_ID);  //实时电流B相
void AN_3_FYRead(uint16_t CAN_ID);  //实时电流C相
void AN_4_FYRead(uint16_t CAN_ID);	  //实时母线电压
void EE_1_FYRead(uint16_t CAN_ID);   //编码器状态
void TI_1_FYRead(uint16_t CAN_ID);   //驱动器温度
void PX_FYRead(uint16_t CAN_ID);     //角度    度
void VX_FYRead(uint16_t CAN_ID);      //速度   数值/秒
void SO_FYRead(uint16_t CAN_ID);      //电机使能状态
void wAS_FY(uint16_t CAN_ID,float vol);    //写偏执电压 CAN2
#endif
