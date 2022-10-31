#include "FsmCompensator_servo.h"
#include "FsmCompensatorPara_servo.h"

#define FSM_PI_servo 3.14159265358979323846
#define FSM_TS_servo 0.01
#define FSM_X_servo 0
#define FSM_Y_servo 1
#define FSM_PATA_NUB_servo 10
#define FSM_SAVE_DATA_LENTH_servo 100

//加二阶滞后程序
//加二阶超前程序

//========================================================
float fsmLeadLag1T1Hz_servo=0;
float fsmLeadLag1Tao1Hz_servo=0;

float fsmLeadLag2T1Hz_servo=0;
float fsmLeadLag2Tao1Hz_servo=0;
float fsmLeadLag2T2Hz_servo=0;  
float fsmLeadLag2Tao2Hz_servo=0;


double fsmTustinT_servo;
double fsmTustinBz_servo[3],fsmTustinBs_servo[3],fsmTustinAz_servo[3],fsmTustinAs_servo[3];

float erFsm1_servo[2][3][FSM_PATA_NUB_servo];
float euFsm1_servo[2][3][FSM_PATA_NUB_servo];
float erFsm2_servo[2][3][FSM_PATA_NUB_servo];
float euFsm2_servo[2][3][FSM_PATA_NUB_servo];

float fsmFilt1B0_servo[2][FSM_PATA_NUB_servo],fsmFilt1B1_servo[2][FSM_PATA_NUB_servo],fsmFilt1A1_servo[2][FSM_PATA_NUB_servo];
float fsmFilt2B0_servo[2][FSM_PATA_NUB_servo],fsmFilt2B1_servo[2][FSM_PATA_NUB_servo],fsmFilt2B2_servo[2][FSM_PATA_NUB_servo],fsmFilt2A1_servo[2][FSM_PATA_NUB_servo],fsmFilt2A2_servo[2][FSM_PATA_NUB_servo];

float fsmTao1_wn_servo[2][FSM_PATA_NUB_servo]={0};
float fsmTao2_kesi_servo[2][FSM_PATA_NUB_servo]={0};
float fsmT1_servo[2][FSM_PATA_NUB_servo]={0};
float fsmT2_servo[2][FSM_PATA_NUB_servo]={0};
float saveData_servo[2][FSM_SAVE_DATA_LENTH_servo]={0};

void AskfsmFilterPara_servo(void)
{
		//0：FSM_X_servo = 0；X0_fsm_wn_servo=24.5 ; X0_fsm_kesi_servo=0.16 ; X0_fsm_T1_servo=1	 ;  X0_fsm_T2_servo=400;
		//1：									X1_fsmTao1_servo=0	 ；X1_fsmTao2_servo=0		  ;	X1_fsmT1_servo=80	 ;  X1_fsmT2_servo=0	 ;	
		//2：									X2_fsmTao1_servo=1	 ;	X2_fsmTao2_servo=0		;	X2_fsmT1_servo=0.2 ;	X2_fsmT2_servo=0	 ;
		//3：									X3_fsmTao1_servo=1.4 ;	X3_fsmTao2_servo=1.4	;	X3_fsmT1_servo=0.2 ;	X3_fsmT2_servo=0.2 ;
		//4：									X4_fsmTao1_servo=0	 ;	X4_fsmTao2_servo=0		;	X4_fsmT1_servo=7   ;	X4_fsmT2_servo=0	 ;
		//5：									X5_fsmTao1_servo=60	 ;	X5_fsmTao2_servo=0.5	;	X5_fsmT1_servo=0   ;	X5_fsmT2_servo=0	 ;
		//6：									X6_fsmTao1_servo=482 ;	X6_fsmTao2_servo=0.8	;	X6_fsmT1_servo=0   ;	X6_fsmT2_servo=0	 ;
		//7：									X7_fsmTao1_servo=0.5 ;	X7_fsmTao2_servo=0.5	;	X7_fsmT1_servo=0.08;	X7_fsmT2_servo=0.08;
		fsmTao1_wn_servo[FSM_X_servo][0]=X0_fsm_wn_servo;  fsmTao2_kesi_servo[FSM_X_servo][0]=X0_fsm_kesi_servo;  fsmT1_servo[FSM_X_servo][0]=X0_fsm_T1_servo;  fsmT2_servo[FSM_X_servo][0]=X0_fsm_T2_servo;
		fsmTao1_wn_servo[FSM_X_servo][1]=X1_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][1]=X1_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][1]=X1_fsmT1_servo;   fsmT2_servo[FSM_X_servo][1]=X1_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][2]=X2_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][2]=X2_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][2]=X2_fsmT1_servo;   fsmT2_servo[FSM_X_servo][2]=X2_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][3]=X3_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][3]=X3_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][3]=X3_fsmT1_servo;   fsmT2_servo[FSM_X_servo][3]=X3_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][4]=X4_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][4]=X4_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][4]=X4_fsmT1_servo;   fsmT2_servo[FSM_X_servo][4]=X4_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][5]=X5_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][5]=X5_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][5]=X5_fsmT1_servo;   fsmT2_servo[FSM_X_servo][5]=X5_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][6]=X6_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][6]=X6_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][6]=X6_fsmT1_servo;   fsmT2_servo[FSM_X_servo][6]=X6_fsmT2_servo;
		fsmTao1_wn_servo[FSM_X_servo][7]=X7_fsmTao1_servo; fsmTao2_kesi_servo[FSM_X_servo][7]=X7_fsmTao2_servo;   fsmT1_servo[FSM_X_servo][7]=X7_fsmT1_servo;   fsmT2_servo[FSM_X_servo][7]=X7_fsmT2_servo;
		
	  //0:FSM_Y_servo = 1； Y0_fsm_wn_servo=24.5 ;  Y0_fsm_kesi_servo=0.16 ;  Y0_fsm_T1_servo=1  ;  Y0_fsm_T2_servo=400;
		//1：									Y1_fsmTao1_servo=0	 ； Y1_fsmTao2_servo=0		 ;  Y1_fsmT1_servo=80	 ;  Y1_fsmT2_servo=0	 ;	
		//2：									Y2_fsmTao1_servo=1	 ;	Y2_fsmTao2_servo=0		 ;	Y2_fsmT1_servo=0.2 ;	Y2_fsmT2_servo=0	 ;
		//3：									Y3_fsmTao1_servo=1.4 ;	Y3_fsmTao2_servo=1.4	 ;	Y3_fsmT1_servo=0.2 ;	Y3_fsmT2_servo=0.2 ;
		//4：									Y4_fsmTao1_servo=0	 ;	Y4_fsmTao2_servo=0		 ;	Y4_fsmT1_servo=7   ;	Y4_fsmT2_servo=0	 ;
		//5：									Y5_fsmTao1_servo=60	 ;	Y5_fsmTao2_servo=0.5	 ;	Y5_fsmT1_servo=0   ;	Y5_fsmT2_servo=0	 ;
		//6：									Y6_fsmTao1_servo=482 ;	Y6_fsmTao2_servo=0.8   ;	Y6_fsmT1_servo=0   ;	Y6_fsmT2_servo=0	 ;
		//7：									Y7_fsmTao1_servo=0.5 ;	Y7_fsmTao2_servo=0.5	 ;	Y7_fsmT1_servo=0.08;	Y7_fsmT2_servo=0.08;
		fsmTao1_wn_servo[FSM_Y_servo][0]=Y0_fsm_wn_servo;  fsmTao2_kesi_servo[FSM_Y_servo][0]=Y0_fsm_kesi_servo;  fsmT1_servo[FSM_Y_servo][0]=Y0_fsm_T1_servo;  fsmT2_servo[FSM_Y_servo][0]=Y0_fsm_T2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][1]=Y1_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][1]=Y1_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][1]=Y1_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][1]=Y1_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][2]=Y2_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][2]=Y2_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][2]=Y2_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][2]=Y2_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][3]=Y3_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][3]=Y3_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][3]=Y3_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][3]=Y3_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][4]=Y4_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][4]=Y4_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][4]=Y4_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][4]=Y4_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][5]=Y5_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][5]=Y5_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][5]=Y5_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][5]=Y5_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][6]=Y6_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][6]=Y6_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][6]=Y6_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][6]=Y6_fsmT2_servo;
		fsmTao1_wn_servo[FSM_Y_servo][7]=Y7_fsmTao1_servo; fsmTao2_kesi_servo[FSM_Y_servo][7]=Y7_fsmTao2_servo;   fsmT1_servo[FSM_Y_servo][7]=Y7_fsmT1_servo;   fsmT2_servo[FSM_Y_servo][7]=Y7_fsmT2_servo;
		//fsmTao1_wn[FSM_Y][0]=1;fsmTao2_kesi[FSM_Y][0]=1;fsmT1[FSM_Y][0]=1;fsmT2[FSM_Y][0]=1;

	//	AskFsmLeadLag2Para(FSM_X,3);
	//	AskFsmLeadLag2Para(FSM_X,0);
	//	AskFsmLeadLag1Para(FSM_X,2);
		
		AskFsmLeadLag2Para_servo(FSM_X_servo,2);	//光闭环一阶超前滞后校正2022/2/24:15:45
		
		AskFsmNotchPara_servo(FSM_X_servo,0);       //震荡环节用（反震荡） 上震荡，下(T1s+1)(T2s+1)
		AskFsmLeadLag1Para_servo(FSM_X_servo,1);    //求解一阶超前滞后，数字是将第几行参数带入，先将频率转换为弧度单位，然后进行双线性变换，求得系数。
		AskFsmLeadLag1Para_servo(FSM_X_servo,2);    //最后调用FsmLeadLag1（）函数进行求解输出
		AskFsmLeadLag2Para_servo(FSM_X_servo,3);    //一阶只有fsmTao1_wn 和 fsmT1 参与双线性变换计算。
		AskFsmLeadLag1Para_servo(FSM_X_servo,4);    //二阶四个参数都参与双线性变换计算。
		Ask2ParaShake_servo(FSM_X_servo,5);    
		Ask2ParaNotchfilter_servo(FSM_X_servo,6);    //AskFsmLeadLag1Para = 上（τs+1）下（Ts+1）
		AskFsmLeadLag2Para_servo(FSM_X_servo,7);    //AskFsmLeadLag2Para = 上（τ1s+1）（τ2s+1）下（T1s+1）（T2s+1）
		
		AskFsmNotchPara_servo(FSM_Y_servo,0);       //震荡环节用（反震荡） 上震荡，下(T1s+1)(T2s+1)
		AskFsmLeadLag1Para_servo(FSM_Y_servo,1);    //求解一阶超前滞后，数字是将第几行参数带入，先将频率转换为弧度单位，然后进行双线性变换，求得系数。
		AskFsmLeadLag1Para_servo(FSM_Y_servo,2);    //最后调用FsmLeadLag1（）函数进行求解输出
		AskFsmLeadLag2Para_servo(FSM_Y_servo,3);    //一阶只有fsmTao1_wn 和 fsmT1 参与双线性变换计算。
		AskFsmLeadLag1Para_servo(FSM_Y_servo,4);    //二阶四个参数都参与双线性变换计算。
		Ask2ParaShake_servo(FSM_Y_servo,5);    
		Ask2ParaNotchfilter_servo(FSM_Y_servo,6);    //AskFsmLeadLag1Para = 上（τs+1）下（Ts+1）
		AskFsmLeadLag2Para_servo(FSM_Y_servo,7);    //AskFsmLeadLag2Para = 上（τ1s+1）（τ2s+1）下（T1s+1）（T2s+1）
	//	AskFsmLeadLag2Para(FSM_Y);
	//	AskFsmLeadLag1Para(FSM_X);
	//	AskFsmLeadLag1Para(FSM_Y);
}


//miss_distance_X_float_LowpassBefore = x
//FSM_X = axis = 0：
//SN = 1
//一阶超前滞后
//					tao*s + 1
//一阶超前滞后:G = -----------
//					 T*s + 1
float FsmLeadLag1_servo(float x,int axis,int SN)  //一阶调用时，调用初始化函数的第几行，SN就等于几
{
	float x0,x1,y1;
	float y;   
    float out;
	
	erFsm1_servo[axis][0][SN] = x;//erFsm1_servo[0][0][1] = miss_distance_X_float_LowpassBefore

	x0 = erFsm1_servo[axis][0][SN];//erFsm1_servo[0][0][1]，x0 = miss_distance_X_float_LowpassBefore
	x1 = erFsm1_servo[axis][1][SN];//erFsm1_servo[0][1][1]
	y1 = euFsm1_servo[axis][1][SN];//erFsm1_servo[0][1][1]

	/*********************************************************************************************	
			Tao1 = 0
			T1 = 1 /（3.14*2*80）			
									 200*Tao1 + 1																	  200*Tao1 + 1																    200*T1 + 1	
			fsmFilt1B0_servo[0][1] = ------------		;	 fsmFilt1B1_servo[0][1] = -  --------------  ;  fsmFilt1A1_servo[0][1] = -  -------------  
		                             200*T1 + 1                                     200*T1 + 1                                     200*T1 + 1	
	
			fsmFilt1B0_servo[0][1]*x0+fsmFilt1B1_servo[0][1]*x1-fsmFilt1A1_servo[0][1]*y1
			
			x0 = miss_distance_X_float_LowpassBefore
					200*Tao1 + 1						  200*Tao1 + 1							    200*T1 + 1		
			y = ------------ * x0 + ( -  -------------- ) * x1 - ( -  ------------- ) * y1
					 200*T1 + 1                200*T1 + 1                   200*T1 + 1		
			x1 = x0		 
			y1 = y
	***********************************************************************************************/ 
	
	y=fsmFilt1B0_servo[axis][SN]*x0+fsmFilt1B1_servo[axis][SN]*x1-fsmFilt1A1_servo[axis][SN]*y1;
	out = y;

	euFsm1_servo[axis][1][SN] = out;												//euFsm1_servo[0][1][1] = out ; y1 = y
	erFsm1_servo[axis][1][SN] = erFsm1_servo[axis][0][SN]; 	//erFsm1_servo[0][1][1] = erFsm1_servo[0][0][1]; x1 = x0

	return out;
}


//x    = FW_miss_distance_error
//axis = FSM_X = 0
//SN   = 7
//二阶超前滞后
//							 (tao1*s + 1)*(tao2*s + 1)
//	二阶超前滞后传递函数：G = ---------------------------
//							  (T1*s + 1) * (T2*s + 1)
float FsmLeadLag2_servo(float x,int axis,int SN)
{
		float x0,x1,x2,y1,y2;
		float y;
		float out;

		erFsm2_servo[axis][0][SN] = x;		//erFsm2_servo[0][0][7] = FW_miss_distance_error

		x0 = erFsm2_servo[axis][0][SN];		//x0 = erFsm2_servo[0][0][7] = x
		x1 = erFsm2_servo[axis][1][SN];		//x1 = erFsm2_servo[0][1][7]
		x2 = erFsm2_servo[axis][2][SN];		//x2 = erFsm2_servo[0][2][7]
		y1 = euFsm2_servo[axis][1][SN];		//y1 = euFsm2_servo[0][1][7]
		y2 = euFsm2_servo[axis][2][SN];		//y2 = euFsm2_servo[0][2][7]
	
		/**************************************************************************************************************************
											200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1	
		fsmFilt2B0[0][7] = fsmTustinBz[0] = ---------------------------------------
										       200^2*T1*T2 + 200*(T1+T2) + 1
			
												2*1-2*200^2*Tao1*Tao2	
		fsmFilt2B1[0][7] = fsmTustinBz[1] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
                                          200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1
		fsmFilt2B2[0][7] = fsmTustinBz[2] = ----------------------------------------
		                                          200^2*T1*T2 + 200*(T1+T2) + 1
		
												2*1-2*200^2*T1*T2	
		fsmFilt2A1[0][7] = fsmTustinAz[1] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
											 200^2*T1*T2 - 200*(T1+T2) + 1
		fsmFilt2A2[0][7] = fsmTustinAz[2] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
	
				 200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1			2*1-2*200^2*Tao1*Tao2							  200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1				       2*1-2*200^2*T1*T2								  200^2*T1*T2 - 200*(T1+T2) + 1
		y = --------------------------------------- * x0 + ------------------------------- * x1 + ---------------------------------------- * x2 - ------------------------------- * y1 - ------------------------------- * y2
	           200^2*T1*T2 + 200*(T1+T2) + 1              200^2*T1*T2 + 200*(T1+T2) + 1                200^2*T1*T2 + 200*(T1+T2) + 1             200^2*T1*T2 + 200*(T1+T2) + 1          200^2*T1*T2 + 200*(T1+T2) + 1
		
		out = y
		y2 = y1  ============  euFsm2[0][2][7] = euFsm2[0][1][7]
		y1 = y	 ============  euFsm2[0][1][7] = out = y
		x2 = x1  ============  erFsm2[0][2][7] = erFsm2[0][1][7]
		x1 = x   ============  erFsm2[0][1][7] = erFsm2[0][0][7] = x
	***************************************************************************************************************************/
	
		y=fsmFilt2B0_servo[axis][SN]*x0+fsmFilt2B1_servo[axis][SN]*x1+fsmFilt2B2_servo[axis][SN]*x2-fsmFilt2A1_servo[axis][SN]*y1-fsmFilt2A2_servo[axis][SN]*y2;
		out = y;

		euFsm2_servo[axis][2][SN]= euFsm2_servo[axis][1][SN];	//y2 = y1  ============  euFsm2[0][2][7] = euFsm2[0][1][7]
		euFsm2_servo[axis][1][SN]= out;												//y1 = y	 ============  euFsm2[0][1][7] = out = y
		erFsm2_servo[axis][2][SN] = erFsm2_servo[axis][1][SN];//x2 = x1  ============  erFsm2[0][2][7] = erFsm2[0][1][7]
		erFsm2_servo[axis][1][SN] = erFsm2_servo[axis][0][SN];//x1 = x   ============  erFsm2[0][1][7] = erFsm2[0][0][7] = x

		return out;
}

//二阶双线性变换公式
void FsmTustin2_servo(void)
{
	
    fsmTustinT_servo=2/FSM_TS_servo;   // FSM_TS_servo = 0.01; fsmTustinT_servo = 200
		
		//fsmTustinAs_servo[0] = T1*T2 = 1/(2*3.14*0.08) * 1/(2*3.14*0.08) ; fsmTustinAs_servo[1] = T1+T2 = 2/(2*3.14*0.08) ; fsmTustinAs_servo[2] = 1
		//fsmTustinAz_servo[0] = 200^2*T1*T2 + 200*(T1+T2) + 1
		//fsmTustinAz_servo[1] = 2*1-2*200^2*T1*T2
		//fsmTustinAz_servo[2] = 200^2*T1*T2 - 200*(T1+T2) + 1
		fsmTustinAz_servo[0]=(fsmTustinAs_servo[0]*fsmTustinT_servo * fsmTustinT_servo) + (fsmTustinAs_servo[1] * fsmTustinT_servo) + fsmTustinAs_servo[2];
		fsmTustinAz_servo[1]=((2*fsmTustinAs_servo[2]-2*fsmTustinAs_servo[0]*fsmTustinT_servo*fsmTustinT_servo) ) ;
    fsmTustinAz_servo[2]=(fsmTustinAs_servo[0]*fsmTustinT_servo * fsmTustinT_servo) - (fsmTustinAs_servo[1] * fsmTustinT_servo) + fsmTustinAs_servo[2];
		
		//fsmTustinBs_servo[0] = Tao1*Tao2 = 1/(2*3.14*0.5) * 1/(2*3.14*0.5) ; fsmTustinBs_servo[1] = Tao1+Tao2 = 2/(2*3.14*0.5) ; fsmTustinBs_servo[2] = 1 ;
		//fsmTustinBz_servo[0] = 200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1
		//fsmTustinBz_servo[1] = 2*1-2*200^2*Tao1*Tao2
		//fsmTustinBz_servo[2] = 200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1
		fsmTustinBz_servo[0]=(fsmTustinBs_servo[0]*fsmTustinT_servo * fsmTustinT_servo) + (fsmTustinBs_servo[1]*fsmTustinT_servo) + fsmTustinBs_servo[2];
		fsmTustinBz_servo[1]=(2*fsmTustinBs_servo[2]-2*fsmTustinBs_servo[0]*fsmTustinT_servo * fsmTustinT_servo) ;
    fsmTustinBz_servo[2]=fsmTustinBs_servo[0] * fsmTustinT_servo * fsmTustinT_servo - (fsmTustinBs_servo[1]*fsmTustinT_servo) + fsmTustinBs_servo[2];
	
//==========================================================================================	
		/*******************************************************************************
														200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1	
		fsmTustinBz_servo[0] = --------------------------------------
														   200^2*T1*T2 + 200*(T1+T2) + 1
			
													     2*1-2*200^2*Tao1*Tao2	
		fsmTustinBz_servo[1] = -------------------------------
		                        200^2*T1*T2 + 200*(T1+T2) + 1
		
														200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1	
		fsmTustinBz_servo[2] = -------------------------------------
		                        200^2*T1*T2 + 200*(T1+T2) + 1
		
		*******************************************************************************/
		fsmTustinBz_servo[0]=fsmTustinBz_servo[0] / fsmTustinAz_servo[0];
		fsmTustinBz_servo[1]=fsmTustinBz_servo[1] / fsmTustinAz_servo[0];
    fsmTustinBz_servo[2]=fsmTustinBz_servo[2] / fsmTustinAz_servo[0];

	
		/*******************************************************************************
													       2*1-2*200^2*T1*T2	
		fsmTustinAz_servo[1] = -------------------------------
		                        200^2*T1*T2 + 200*(T1+T2) + 1
		
														200^2*T1*T2 - 200*(T1+T2) + 1
		fsmTustinAz_servo[2] = -------------------------------
		                        200^2*T1*T2 + 200*(T1+T2) + 1
		
		*******************************************************************************/
		fsmTustinAz_servo[1]=fsmTustinAz_servo[1] / fsmTustinAz_servo[0] ;
    fsmTustinAz_servo[2]=fsmTustinAz_servo[2] /fsmTustinAz_servo[0];
		fsmTustinAz_servo[0]=1.0;

}

/*
					2		1 - z^(-1)
双线性变换公式：S = ---- * ------------		,Ts是周期，采样时间
					Ts	    1 + z^(-1)
*/

//一阶双线性变换公式
void FsmTustin1_servo(void)
 {
		//TustinT=2/0.000064;
		//TustinT=2/0.000256;
		fsmTustinT_servo = 2/FSM_TS_servo;			//FSM_TS_servo = 0.01；
		 
		//fsmTustinAs_servo[0] = T1 = 1 /（3.14*2*80）；fsmTustinT_servo = 200；fsmTustinAs_servo[1] = 1	 
		//fsmTustinAz_servo[0] =  200*T1 + 1
		//fsmTustinAz_servo[1] = -200*T1 + 1
		fsmTustinAz_servo[0] = (fsmTustinAs_servo[0]*fsmTustinT_servo) + (fsmTustinAs_servo[1]) ;//
		fsmTustinAz_servo[1] = -(fsmTustinAs_servo[0]*fsmTustinT_servo) + (fsmTustinAs_servo[1]) ;
		
		//fsmTustinBs_servo[0] = Tao1 = 0 ；fsmTustinT_servo = 200 ；fsmTustinBs_servo[1] = 1
		//fsmTustinBz_servo[0] =   200*Tao1 + 1 = 1
		//fsmTustinBz_servo[1] = - 200*Tao1 + 1 = 1
		fsmTustinBz_servo[0] = (fsmTustinBs_servo[0]*fsmTustinT_servo) + (fsmTustinBs_servo[1]) ;
		fsmTustinBz_servo[1] = -(fsmTustinBs_servo[0]*fsmTustinT_servo) + (fsmTustinBs_servo[1]) ;
		
//==========================================================================================	
		
		 /*
																200*Tao1 + 1				 1
				fsmTustinBz_servo[0] = -------------- = ------------  	
																 200*T1 + 1			 200*T1 + 1
				
																-200*Tao1 + 1				-1
				fsmTustinBz_servo[1] = -------------- = ------------
																	200*T1 + 1		 200*T1 + 1
				
																-200*T1 + 1				
				fsmTustinAz_servo[1] = --------------
																 200*T1 + 1		     		
				
				fsmTustinAz_servo[0]=1.0;
		*/
		fsmTustinBz_servo[0]=fsmTustinBz_servo[0] / fsmTustinAz_servo[0];
		fsmTustinBz_servo[1]=fsmTustinBz_servo[1] / fsmTustinAz_servo[0];
			
		fsmTustinAz_servo[1]=fsmTustinAz_servo[1] / fsmTustinAz_servo[0] ;
		fsmTustinAz_servo[0]=1.0;
}


//
void AskFsmLeadLag2Para_servo(int axis,int SN)
{
		double T1,T2;
		double Tao1,Tao2;
		float fsmLeadLag2Tao1Hz,fsmLeadLag2Tao2Hz;
		float fsmLeadLag2T1Hz,fsmLeadLag2T2Hz;
		
		//fsmLeadLag2Tao1Hz = 0.5 (hz)
		//fsmLeadLag2Tao2Hz = 0.5 (hz)
		//fsmLeadLag2T1Hz = 0.08 (hz)
		//fsmLeadLag2T2Hz = 0.08 (hz)
		fsmLeadLag2Tao1Hz=fsmTao1_wn_servo[axis][SN]; 	//fsmTao1_wn_servo[0][7]  = X7_fsmTao1_servo = 0.5 (hz)
		fsmLeadLag2Tao2Hz=fsmTao2_kesi_servo[axis][SN];	//fsmTao2_kesi_servo[0][7]= X7_fsmTao2_servo = 0.5 (hz)
		fsmLeadLag2T1Hz=fsmT1_servo[axis][SN];					//fsmT1_servo[0][7] = X7_fsmT1_servo = 0.08
		fsmLeadLag2T2Hz=fsmT2_servo[axis][SN];					//fsmT2_servo[0][7] = X7_fsmT2_servo = 0.08
		
		if(fsmLeadLag2Tao1Hz == 0)
		{
			Tao1 = 0;
		}
		else
		{
			Tao1=1/(FSM_PI_servo*2*fsmLeadLag2Tao1Hz);      //Tao1 = 1/(2*3.14*0.5)
		}
		if(fsmLeadLag2Tao2Hz == 0)
		{
			Tao2 = 0;
		}
		else
		{
			Tao2=1/(FSM_PI_servo*2*fsmLeadLag2Tao2Hz);			//Tao2 = 1/(2*3.14*0.5)
		}
		fsmTustinBs_servo[0] = Tao1*Tao2 ;							//fsmTustinBs_servo[0] = Tao1*Tao2 = 1/(2*3.14*0.5) * 1/(2*3.14*0.5)
		fsmTustinBs_servo[1] = Tao1+Tao2 ;							//fsmTustinBs_servo[1] = Tao1+Tao2 = 2/(2*3.14*0.5)
		fsmTustinBs_servo[2] = 1 ;
		
		if(fsmLeadLag2T1Hz == 0)
		{
			T1 = 0;
		}
		else
		{
			T1=1/(FSM_PI_servo*2*fsmLeadLag2T1Hz);					//T1=1/(2*3.14*0.08)
		}
		if(fsmLeadLag2T2Hz == 0)
		{
			T2 = 0;
		}
		else
		{
			T2=1/(FSM_PI_servo*2*fsmLeadLag2T2Hz);					//T2=1/(2*3.14*0.08)
		}
		
		fsmTustinAs_servo[0] = T1*T2 ;									//fsmTustinAs_servo[0] = T1*T2 = 1/(2*3.14*0.08) * 1/(2*3.14*0.08) 
		fsmTustinAs_servo[1] = T1+T2 ;									//fsmTustinAs_servo[1] = T1+T2 = 2/(2*3.14*0.08)
		fsmTustinAs_servo[2] = 1 ;

		FsmTustin2_servo();
	
		/*********************************************************************************************
														200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1	
		fsmFilt2B0_servo[0][7] = fsmTustinBz_servo[0] =  --------------------------------------
		                                                	   200^2*T1*T2 + 200*(T1+T2) + 1
		
															2*1-2*200^2*Tao1*Tao2		
		fsmFilt2B1_servo[0][7] = fsmTustinBz_servo[1] = -------------------------------
		                                                 200^2*T1*T2 + 200*(T1+T2) + 1
		
														200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1	
		fsmFilt2B2_servo[0][7] = fsmTustinBz_servo[2] =  ---------------------------------------
		                                                      200^2*T1*T2 + 200*(T1+T2) + 1
		
															2*1-2*200^2*T1*T2			
		fsmFilt2A1_servo[0][7] = fsmTustinAz_servo[1] = -------------------------------
		                                                 200^2*T1*T2 + 200*(T1+T2) + 1
		
														  200^2*T1*T2 - 200*(T1+T2) + 1
		fsmFilt2A2_servo[0][7] = fsmTustinAz_servo[2] =  -------------------------------
		                                                  200^2*T1*T2 + 200*(T1+T2) + 1
		
		*********************************************************************************************/
		fsmFilt2B0_servo[axis][SN]=fsmTustinBz_servo[0];	//双线性变化的系数存在不同轴的不同列
		fsmFilt2B1_servo[axis][SN]=fsmTustinBz_servo[1];
		fsmFilt2B2_servo[axis][SN]=fsmTustinBz_servo[2]; 
		fsmFilt2A1_servo[axis][SN]=fsmTustinAz_servo[1];	
		fsmFilt2A2_servo[axis][SN]=fsmTustinAz_servo[2];

}

//axis = FSM_X_servo = 0
//SN = 1
void AskFsmLeadLag1Para_servo(int axis,int SN)   
{
		double T1Rad,T1;
		double Tao1Rad,Tao1;
		float fsmLeadLag1Tao1Hz;
    float fsmLeadLag1T1Hz;
	
		fsmLeadLag1Tao1Hz=fsmTao1_wn_servo[axis][SN];   //fsmLeadLag1Tao1Hz = fsmTao1_wn_servo[0][1] = 0
		fsmLeadLag1T1Hz=fsmT1_servo[axis][SN];					//fsmLeadLag1T1Hz = fsmT1_servo[0][1]	=	80 （hz）

		T1Rad=FSM_PI_servo*2*fsmLeadLag1T1Hz;						//T1Rad = 3.14*2*fsmLeadLag1T1Hz = 3.14*2*80
		T1=1/T1Rad;																			//T1 = 1/T1Rad	=	1/（3.14*2*80）
		
		Tao1Rad=FSM_PI_servo*2*fsmLeadLag1Tao1Hz;				//Tao1Rad = 3.14*2*fsmLeadLag1Tao1Hz = 0
		if(fsmLeadLag1Tao1Hz == 0)
		{
			Tao1 = 0;
		}
		else
		{
			Tao1=1/Tao1Rad;
		}
		
		fsmTustinBs_servo[0] = Tao1 ;										  //Tao1 = 0
		fsmTustinBs_servo[1] = 1 ;
		fsmTustinAs_servo[0] = T1 ;											  //T1 = 1 /（3.14*2*80）
		fsmTustinAs_servo[1] = 1 ;

		FsmTustin1_servo();															  //fsmTustinAs_servo

		/*
															 200*Tao1 + 1
			fsmFilt1B0_servo[0][1] = ------------
		                            200*T1 + 1
			
														       200*Tao1 + 1
			fsmFilt1B1_servo[0][1] = -  --------------
		                                200*T1 + 1
		
																	 200*T1 + 1				-1
			fsmFilt1A1_servo[0][1] = -  ------------- = ------ = -1
															     200*T1 + 1		     1						
			
		*/
		
		fsmFilt1B0_servo[axis][SN]=fsmTustinBz_servo[0];	// fsmFilt1B0_servo
		fsmFilt1B1_servo[axis][SN]=fsmTustinBz_servo[1];	//

		fsmFilt1A1_servo[axis][SN]=fsmTustinAz_servo[1];	//	
}
void Ask2ParaShake_servo(int axis,int SN)
{
	double KesiXieZhen1;
	double OmigaXieZhen1;
	double omigaRad;
	
	OmigaXieZhen1=fsmTao1_wn_servo[axis][SN];
	KesiXieZhen1=fsmTao2_kesi_servo[axis][SN];
	
	omigaRad=FSM_PI_servo*2*OmigaXieZhen1;
	
	//As是分母
	fsmTustinBs_servo[0] =  0;
	fsmTustinBs_servo[1] =  0;
	fsmTustinBs_servo[2] =  1;
    fsmTustinAs_servo[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinAs_servo[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinAs_servo[2] = 1 ;
	
	FsmTustin2_servo();

	fsmFilt2B0_servo[axis][SN]=fsmTustinBz_servo[0];	
	fsmFilt2B1_servo[axis][SN]=fsmTustinBz_servo[1];
	fsmFilt2B2_servo[axis][SN]=fsmTustinBz_servo[2]; 
	fsmFilt2A1_servo[axis][SN]=fsmTustinAz_servo[1];	
	fsmFilt2A2_servo[axis][SN]=fsmTustinAz_servo[2];

}
void Ask2ParaNotchfilter_servo(int axis,int SN)
{
	double KesiXieZhen1;
	double OmigaXieZhen1;
	double omigaRad;
	
	OmigaXieZhen1=fsmTao1_wn_servo[axis][SN];
	KesiXieZhen1=fsmTao2_kesi_servo[axis][SN];
	
	omigaRad=FSM_PI_servo*2*OmigaXieZhen1;
	
	//As是分母
	fsmTustinBs_servo[0] =  1/(omigaRad*omigaRad);
	fsmTustinBs_servo[1] =  0;
	fsmTustinBs_servo[2] =  1;
    fsmTustinAs_servo[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinAs_servo[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinAs_servo[2] = 1 ;
	
	FsmTustin2_servo();

	fsmFilt2B0_servo[axis][SN]=fsmTustinBz_servo[0];	
	fsmFilt2B1_servo[axis][SN]=fsmTustinBz_servo[1];
	fsmFilt2B2_servo[axis][SN]=fsmTustinBz_servo[2]; 
	fsmFilt2A1_servo[axis][SN]=fsmTustinAz_servo[1];	
	fsmFilt2A2_servo[axis][SN]=fsmTustinAz_servo[2];

}
void AskFsmNotchPara_servo(int axis,int SN)     //？
{
	double T1,T2;
	double Tao1,Tao2;
	float fsmLeadLag2Tao1Hz,fsmLeadLag2Tao2Hz;
	float fsmLeadLag2T1Hz,fsmLeadLag2T2Hz;
	double omigaRad;
	double KesiXieZhen1;
	double OmigaXieZhen1;
	
	OmigaXieZhen1=fsmTao1_wn_servo[axis][SN];
	KesiXieZhen1=fsmTao2_kesi_servo[axis][SN];
	fsmLeadLag2T1Hz=fsmT1_servo[axis][SN];
	fsmLeadLag2T2Hz=fsmT2_servo[axis][SN];
	
	omigaRad=FSM_PI_servo*2*OmigaXieZhen1;

	fsmTustinBs_servo[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinBs_servo[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinBs_servo[2] = 1 ;
	T1=1/(FSM_PI_servo*2*fsmLeadLag2T1Hz);
	T2=1/(FSM_PI_servo*2*fsmLeadLag2T2Hz);
	fsmTustinAs_servo[0] = T1*T2 ;
	fsmTustinAs_servo[1] = T1+T2 ;
	fsmTustinAs_servo[2] = 1 ;

	FsmTustin2_servo();

	fsmFilt2B0_servo[axis][SN]=fsmTustinBz_servo[0];	
	fsmFilt2B1_servo[axis][SN]=fsmTustinBz_servo[1];
	fsmFilt2B2_servo[axis][SN]=fsmTustinBz_servo[2]; 
	fsmFilt2A1_servo[axis][SN]=fsmTustinAz_servo[1];	
	fsmFilt2A2_servo[axis][SN]=fsmTustinAz_servo[2];
}



