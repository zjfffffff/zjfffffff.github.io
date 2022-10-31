#ifndef __FSMCOMPENSATORPARA_servo_h
#define __FSMCOMPENSATORPARA_servo_h

float X0_fsm_wn_servo=24.5;
float X0_fsm_kesi_servo=0.16;
float X0_fsm_T1_servo=1;
float X0_fsm_T2_servo=400;

//编码器低通滤波★
float X1_fsmTao1_servo=0;
float X1_fsmTao2_servo=0;
float X1_fsmT1_servo=80;
float X1_fsmT2_servo=0;

//光闭环一阶校正
float X2_fsmTao1_servo=1;
float X2_fsmTao2_servo=0;
float X2_fsmT1_servo=0.2;
float X2_fsmT2_servo=0;

float X3_fsmTao1_servo=1.4;
float X3_fsmTao2_servo=1.4;
float X3_fsmT1_servo=0.2;
float X3_fsmT2_servo=0.2;

float X4_fsmTao1_servo=0;
float X4_fsmTao2_servo=0;
float X4_fsmT1_servo=7;
float X4_fsmT2_servo=0;

float X5_fsmTao1_servo=60;     
float X5_fsmTao2_servo=0.5;    
float X5_fsmT1_servo=0;     
float X5_fsmT2_servo=0;


float X6_fsmTao1_servo=482;
float X6_fsmTao2_servo=0.8;
float X6_fsmT1_servo=0;
float X6_fsmT2_servo=0;

//光闭环二阶校正★
float X7_fsmTao1_servo=0.5;     //原来是1
float X7_fsmTao2_servo=0.5;     //原来是1
float X7_fsmT1_servo=0.08;
float X7_fsmT2_servo=0.08;




float Y0_fsm_wn_servo=24.5;
float Y0_fsm_kesi_servo=0.16;
float Y0_fsm_T1_servo=1;
float Y0_fsm_T2_servo=400;

//脱靶量80Hz低通滤波★
float Y1_fsmTao1_servo=0;
float Y1_fsmTao2_servo=0;
float Y1_fsmT1_servo=80;
float Y1_fsmT2_servo=0;

//光闭环一阶校正
float Y2_fsmTao1_servo=1;
float Y2_fsmTao2_servo=0;
float Y2_fsmT1_servo=0.2;
float Y2_fsmT2_servo=0;

float Y3_fsmTao1_servo=1.4;
float Y3_fsmTao2_servo=1.4;
float Y3_fsmT1_servo=0.2;
float Y3_fsmT2_servo=0.2;

float Y4_fsmTao1_servo=0;
float Y4_fsmTao2_servo=0;
float Y4_fsmT1_servo=7;
float Y4_fsmT2_servo=0;

float Y5_fsmTao1_servo=60;     
float Y5_fsmTao2_servo=0.5;    
float Y5_fsmT1_servo=0;     
float Y5_fsmT2_servo=0;

float Y6_fsmTao1_servo=482;
float Y6_fsmTao2_servo=0.8;
float Y6_fsmT1_servo=0;
float Y6_fsmT2_servo=0;

//光闭环二阶校正★
float Y7_fsmTao1_servo=1;       //原来是1
float Y7_fsmTao2_servo=1;       //原来是1
float Y7_fsmT1_servo=0.2;
float Y7_fsmT2_servo=0.2;

#endif 