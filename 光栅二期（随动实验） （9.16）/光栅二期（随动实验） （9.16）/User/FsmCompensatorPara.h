#ifndef __FSMCOMPENSATORPARA_h
#define __FSMCOMPENSATORPARA_h

//※阶跃前置滤波
float X0_fsmTao1=0;
float X0_fsmTao2=0;
float X0_fsmT1=80;
float X0_fsmT2=0;

//※编码器80Hz低通滤波    
float X1_fsmTao1=0;
float X1_fsmTao2=0;
float X1_fsmT1=80;
float X1_fsmT2=0;

//※稳定速度前馈滤波   
float X2_fsmTao1=0;
float X2_fsmTao2=0;
float X2_fsmT1=2;
float X2_fsmT2=0;

//※（三个参数中的）二阶校正  
float X3_fsmTao1=0.4;
float X3_fsmTao2=0.4;
float X3_fsmT1=0.2;
float X3_fsmT2=0.2;

//※光闭环速度前馈滤波
float X4_fsmTao1=0;
float X4_fsmTao2=0;
float X4_fsmT1=2;
float X4_fsmT2=0;

//※（三个参数中的）一阶校正   
float X5_fsmTao1=1;     
float X5_fsmTao2=0;   
float X5_fsmT1=0.2;     
float X5_fsmT2=0;

float X6_fsmTao1=0;
float X6_fsmTao2=0;
float X6_fsmT1=100;
float X6_fsmT2=0;

float X7_fsmTao1=1.6;
float X7_fsmTao2=1.6;
float X7_fsmT1=0.2;
float X7_fsmT2=0.2;




//阶跃前置滤波
float Y0_fsmTao1=0;
float Y0_fsmTao2=0;
float Y0_fsmT1=80;
float Y0_fsmT2=0;

//稳定80Hz低通滤波
float Y1_fsmTao1=0;
float Y1_fsmTao2=0;
float Y1_fsmT1=80;
float Y1_fsmT2=0;

//稳定速度前馈滤波
float Y2_fsmTao1=0;
float Y2_fsmTao2=0;
float Y2_fsmT1=2;
float Y2_fsmT2=0;

//（三个参数中的）二阶校正
float Y3_fsmTao1=0.4;
float Y3_fsmTao2=0.4;
float Y3_fsmT1=0.2;
float Y3_fsmT2=0.2;

//光闭环速度前馈滤波
float Y4_fsmTao1=0;
float Y4_fsmTao2=0;
float Y4_fsmT1=2;
float Y4_fsmT2=0;

//（三个参数中的）一阶校正 
float Y5_fsmTao1=1;    
float Y5_fsmTao2=0;    
float Y5_fsmT1=0.2;     
float Y5_fsmT2=0;

float Y6_fsmTao1=0;
float Y6_fsmTao2=0;
float Y6_fsmT1=100;
float Y6_fsmT2=0;

float Y7_fsmTao1=1.6;
float Y7_fsmTao2=1.6;
float Y7_fsmT1=0.2;
float Y7_fsmT2=0.2;


#endif 