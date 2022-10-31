#include "FsmCompensator.h"
#include "FsmCompensatorPara.h"

#define FSM_PI 3.14159265358979323846
#define FSM_TS 0.001
#define FSM_Qs 0.001
#define FSM_Guanceqi 0.001

#define FSM_PATA_NUB 12
#define FSM_SAVE_DATA_LENTH 100


float X8_fsmTao1 = 0.001;		//自己用的滤波器参数
float Y8_fsmTao1 = 0.001;		//自己用的滤波器参数

float X9_fsmTao1 = 0.001;		//自己用的滤波器参数
float Y9_fsmTao1 = 0.001;		//自己用的滤波器参数

float X10_fsmTao1 = 0.001;		//自己用的滤波器参数:w2调节抗干扰能力
float X10_fsmT1 = 0.001;			//自己用的滤波器参数:w1调节响应时间
float Y10_fsmTao1 = 0.001;		//自己用的滤波器参数:w2调节抗干扰能力
float Y10_fsmT1 = 0.001;			//自己用的滤波器参数:w1调节响应时间

//加二阶滞后程序
//加二阶超前程序

//========================================================
float fsmLeadLag1T1Hz=0;
float fsmLeadLag1Tao1Hz=0;

float fsmLeadLag2T1Hz=0;
float fsmLeadLag2Tao1Hz=0;
float fsmLeadLag2T2Hz=0;  
float fsmLeadLag2Tao2Hz=0;


double fsmTustinT;
double fsmTustinBz[4],fsmTustinBs[4],fsmTustinAz[4],fsmTustinAs[4];

float erFsm1[2][3][FSM_PATA_NUB];
float euFsm1[2][3][FSM_PATA_NUB];
float erFsm2[2][3][FSM_PATA_NUB];
float euFsm2[2][3][FSM_PATA_NUB];

float fsmFilt1B0[2][FSM_PATA_NUB],fsmFilt1B1[2][FSM_PATA_NUB],fsmFilt1A1[2][FSM_PATA_NUB];
float fsmFilt2B0[2][FSM_PATA_NUB],fsmFilt2B1[2][FSM_PATA_NUB],fsmFilt2B2[2][FSM_PATA_NUB],fsmFilt2A1[2][FSM_PATA_NUB],fsmFilt2A2[2][FSM_PATA_NUB];
float fsmFilt3B0[2][FSM_PATA_NUB],fsmFilt3B1[2][FSM_PATA_NUB],fsmFilt3B2[2][FSM_PATA_NUB],fsmFilt3B3[2][FSM_PATA_NUB],fsmFilt3A1[2][FSM_PATA_NUB],fsmFilt3A2[2][FSM_PATA_NUB],fsmFilt3A3[2][FSM_PATA_NUB];

float fsmTao1_wn[2][FSM_PATA_NUB]={0};
float fsmTao2_kesi[2][FSM_PATA_NUB]={0};
float fsmT1[2][FSM_PATA_NUB]={0};
float fsmT2[2][FSM_PATA_NUB]={0};
float saveData[2][FSM_SAVE_DATA_LENTH]={0};

void AskfsmFilterPara(void)
{
		//0：FSM_X = 0；X0_fsmTao1=0   ;  X0_fsmTao2=0    ; X0_fsmT1=20	 ;  X0_fsmT2=0   ;
		//1：						X1_fsmTao1=0	 ； X1_fsmTao2=0		;	X1_fsmT1=80	 ;  X1_fsmT2=0	 ;	
		//2：						X2_fsmTao1=1	 ;	X2_fsmTao2=0		;	X2_fsmT1=2   ;	X2_fsmT2=0	 ;
		//3：						X3_fsmTao1=0.4 ;	X3_fsmTao2=0.4	;	X3_fsmT1=0.2 ;	X3_fsmT2=0.2 ;
		//4：						X4_fsmTao1=0	 ;	X4_fsmTao2=0		;	X4_fsmT1=2   ;	X4_fsmT2=0	 ;
		//5：						X5_fsmTao1=1	 ;	X5_fsmTao2=0		;	X5_fsmT1=0.2 ;	X5_fsmT2=0	 ;
		//6：						X6_fsmTao1=0   ;	X6_fsmTao2=0		;	X6_fsmT1=100 ;	X6_fsmT2=0	 ;
		//7：						X7_fsmTao1=1.6 ;	X7_fsmTao2=1.6	;	X7_fsmT1=0.2 ;	X7_fsmT2=0.2 ;
		
		fsmTao1_wn[FSM_X][0]=X0_fsmTao1; fsmTao2_kesi[FSM_X][0]=X0_fsmTao2;   fsmT1[FSM_X][0]=X0_fsmT1;   fsmT2[FSM_X][0]=X0_fsmT2;
		fsmTao1_wn[FSM_X][1]=X1_fsmTao1; fsmTao2_kesi[FSM_X][1]=X1_fsmTao2;   fsmT1[FSM_X][1]=X1_fsmT1;   fsmT2[FSM_X][1]=X1_fsmT2;
		fsmTao1_wn[FSM_X][2]=X2_fsmTao1; fsmTao2_kesi[FSM_X][2]=X2_fsmTao2;   fsmT1[FSM_X][2]=X2_fsmT1;   fsmT2[FSM_X][2]=X2_fsmT2;
		fsmTao1_wn[FSM_X][3]=X3_fsmTao1; fsmTao2_kesi[FSM_X][3]=X3_fsmTao2;   fsmT1[FSM_X][3]=X3_fsmT1;   fsmT2[FSM_X][3]=X3_fsmT2;
		fsmTao1_wn[FSM_X][4]=X4_fsmTao1; fsmTao2_kesi[FSM_X][4]=X4_fsmTao2;   fsmT1[FSM_X][4]=X4_fsmT1;   fsmT2[FSM_X][4]=X4_fsmT2;
		fsmTao1_wn[FSM_X][5]=X5_fsmTao1; fsmTao2_kesi[FSM_X][5]=X5_fsmTao2;   fsmT1[FSM_X][5]=X5_fsmT1;   fsmT2[FSM_X][5]=X5_fsmT2;
		fsmTao1_wn[FSM_X][6]=X6_fsmTao1; fsmTao2_kesi[FSM_X][6]=X6_fsmTao2;   fsmT1[FSM_X][6]=X6_fsmT1;   fsmT2[FSM_X][6]=X6_fsmT2;
		fsmTao1_wn[FSM_X][7]=X7_fsmTao1; fsmTao2_kesi[FSM_X][7]=X7_fsmTao2;   fsmT1[FSM_X][7]=X7_fsmT1;   fsmT2[FSM_X][7]=X7_fsmT2;
		fsmTao1_wn[FSM_X][8]=X8_fsmTao1;//自己用的滤波器
		fsmTao1_wn[FSM_X][9]=X9_fsmTao1;//自己用的滤波器
		fsmTao1_wn[FSM_X][10]=X10_fsmTao1;fsmT1[FSM_X][10]=X10_fsmT1;//w2调节抗干扰能力\w1调节响应时间
	
		//0:FSM_Y = 1； Y0_fsmTao1=0   ;  Y0_fsmTao2=0     ;  Y0_fsmT1=20  ;  Y0_fsmT2=0   ;
		//1：						Y1_fsmTao1=0	 ； Y1_fsmTao2=0		 ;  Y1_fsmT1=80	 ;  Y1_fsmT2=0	 ;	
		//2：						Y2_fsmTao1=0	 ;	Y2_fsmTao2=0		 ;	Y2_fsmT1=2   ;	Y2_fsmT2=0	 ;
		//3：						Y3_fsmTao1=0.4 ;	Y3_fsmTao2=0.4	 ;	Y3_fsmT1=0.2 ;	Y3_fsmT2=0.2 ;
		//4：						Y4_fsmTao1=0	 ;	Y4_fsmTao2=0		 ;	Y4_fsmT1=2   ;	Y4_fsmT2=0	 ;
		//5：						Y5_fsmTao1=1	 ;	Y5_fsmTao2=0	   ;	Y5_fsmT1=0.2 ;	Y5_fsmT2=0	 ;
		//6：						Y6_fsmTao1=0   ;	Y6_fsmTao2=0     ;	Y6_fsmT1=100 ;	Y6_fsmT2=0	 ;
		//7：						Y7_fsmTao1=1.6 ;	Y7_fsmTao2=1.6	 ;	Y7_fsmT1=0.2 ;	Y7_fsmT2=0.2 ;
		fsmTao1_wn[FSM_Y][0]=Y0_fsmTao1; fsmTao2_kesi[FSM_Y][0]=Y0_fsmTao2;   fsmT1[FSM_Y][0]=Y0_fsmT1;   fsmT2[FSM_Y][0]=Y0_fsmT2;
		fsmTao1_wn[FSM_Y][1]=Y1_fsmTao1; fsmTao2_kesi[FSM_Y][1]=Y1_fsmTao2;   fsmT1[FSM_Y][1]=Y1_fsmT1;   fsmT2[FSM_Y][1]=Y1_fsmT2;
		fsmTao1_wn[FSM_Y][2]=Y2_fsmTao1; fsmTao2_kesi[FSM_Y][2]=Y2_fsmTao2;   fsmT1[FSM_Y][2]=Y2_fsmT1;   fsmT2[FSM_Y][2]=Y2_fsmT2;
		fsmTao1_wn[FSM_Y][3]=Y3_fsmTao1; fsmTao2_kesi[FSM_Y][3]=Y3_fsmTao2;   fsmT1[FSM_Y][3]=Y3_fsmT1;   fsmT2[FSM_Y][3]=Y3_fsmT2;
		fsmTao1_wn[FSM_Y][4]=Y4_fsmTao1; fsmTao2_kesi[FSM_Y][4]=Y4_fsmTao2;   fsmT1[FSM_Y][4]=Y4_fsmT1;   fsmT2[FSM_Y][4]=Y4_fsmT2;
		fsmTao1_wn[FSM_Y][5]=Y5_fsmTao1; fsmTao2_kesi[FSM_Y][5]=Y5_fsmTao2;   fsmT1[FSM_Y][5]=Y5_fsmT1;   fsmT2[FSM_Y][5]=Y5_fsmT2;
		fsmTao1_wn[FSM_Y][6]=Y6_fsmTao1; fsmTao2_kesi[FSM_Y][6]=Y6_fsmTao2;   fsmT1[FSM_Y][6]=Y6_fsmT1;   fsmT2[FSM_Y][6]=Y6_fsmT2;
		fsmTao1_wn[FSM_Y][7]=Y7_fsmTao1; fsmTao2_kesi[FSM_Y][7]=Y7_fsmTao2;   fsmT1[FSM_Y][7]=Y7_fsmT1;   fsmT2[FSM_Y][7]=Y7_fsmT2;
		fsmTao1_wn[FSM_Y][8]=Y8_fsmTao1;		//自己用的滤波器
		fsmTao1_wn[FSM_Y][9]=Y9_fsmTao1;		//自己用的滤波器
		fsmTao1_wn[FSM_Y][10]=Y10_fsmTao1;fsmT1[FSM_Y][10]=Y10_fsmT1;//w2调节抗干扰能力\w1调节响应时间
		//fsmTao1_wn[FSM_Y][0]=1;fsmTao2_kesi[FSM_Y][0]=1;fsmT1[FSM_Y][0]=1;fsmT2[FSM_Y][0]=1;

		AskFsmLeadLag3Para(FSM_X,8);	//Q滤波器初始化
		AskFsmLeadLag3Para(FSM_Y,8);	//Q滤波器初始化
		
		AskFsmLeadLag4Para(FSM_X,9);	//过程对象初始化
		AskFsmLeadLag4Para(FSM_Y,9);	//过程对象初始化
		
		AskFsmLeadLag1Para(FSM_X,10);
		AskFsmLeadLag1Para(FSM_Y,10);
		
		AskFsmLeadLag1Para(FSM_X,0);    //震荡环节用（反震荡） 上震荡，下(T1s+1)(T2s+1)
		AskFsmLeadLag1Para(FSM_X,1);    //求解一阶超前滞后，数字是将第几行参数带入，先将频率转换为弧度单位，然后进行双线性变换，求得系数。
		AskFsmLeadLag1Para(FSM_X,2);    //最后调用FsmLeadLag1（）函数进行求解输出
		AskFsmLeadLag2Para(FSM_X,3);    //一阶只有fsmTao1_wn 和 fsmT1 参与双线性变换计算。
		AskFsmLeadLag1Para(FSM_X,4);    //二阶四个参数都参与双线性变换计算。
		AskFsmLeadLag1Para(FSM_X,5);
		//Ask2ParaShake(FSM_X,5);    
		AskFsmLeadLag1Para(FSM_X,6);    //AskFsmLeadLag1Para = 上（τs+1）下（Ts+1）
		AskFsmLeadLag2Para(FSM_X,7);    //AskFsmLeadLag2Para = 上（τ1s+1）（τ2s+1）下（T1s+1）（T2s+1）
		
		AskFsmLeadLag1Para(FSM_Y,0);
			AskFsmLeadLag1Para(FSM_Y,1);    //求解一阶超前滞后，数字是将第几行参数带入，先将频率转换为弧度单位，然后进行双线性变换，求得系数。
		AskFsmLeadLag1Para(FSM_Y,2);    //最后调用FsmLeadLag1（）函数进行求解输出
		AskFsmLeadLag2Para(FSM_Y,3);    //一阶只有fsmTao1_wn 和 fsmT1 参与双线性变换计算。
		AskFsmLeadLag1Para(FSM_Y,4);    //二阶四个参数都参与双线性变换计算。
		AskFsmLeadLag1Para(FSM_Y,5);
		//Ask2ParaShake(FSM_X,5);    
		AskFsmLeadLag1Para(FSM_Y,6);    //AskFsmLeadLag1Para = 上（τs+1）下（Ts+1）
		AskFsmLeadLag2Para(FSM_Y,7);    //AskFsmLeadLag2Para = 上（τ1s+1）（τ2s+1）下（T1s+1）（T2s+1）
	
}

//FW_encoder_degrees = x
//FSM_X = axis = 0：
//SN = 1
//									tao*s + 1
//一阶超前滞后:G = -----------
//									 T*s + 1
float FsmLeadLag1(float x,int axis,int SN)  //一阶调用时，调用初始化函数的第几行，SN就等于几
{
		float x0,x1,y1;
		float y;   
		float out;
		
		erFsm1[axis][0][SN] = x;		//x=FY_encoder_degrees,axis=FSM_Y=1,SN=1;

		x0 = erFsm1[axis][0][SN];	 //erFsm1_servo[0][0][1]，x0 = x = FY_encoder_degrees
		x1 = erFsm1[axis][1][SN];  //erFsm1_servo[0][1][1]
		y1 = euFsm1[axis][1][SN];  //erFsm1_servo[0][1][1]
		
		/*********************************************************************************************	
			Tao1 = 0
			T1 = 1 /（3.14*2*80）			
												 200*Tao1 + 1													- 200*Tao1 + 1												-	200*T1 + 1	
			fsmFilt1B0[0][1] = ------------		;	 fsmFilt1B1[0][1] = --------------  ;  fsmFilt1A1[0][1] = -------------- 
		                      200*T1 + 1                            200*T1 + 1                            200*T1 + 1	
	
			y = fsmFilt1B0[0][1]*x0+fsmFilt1B1[0][1]*x1-fsmFilt1A1[0][1]*y1
			
			x0 = FY_encoder_degrees
					200*Tao1 + 1				- 200*Tao1 + 1						 - 200*T1 + 1		
			y = ------------ * x0 + ---------------  * x1 -  --------------  * y1
					 200*T1 + 1            200*T1 + 1               200*T1 + 1		
			
			x1 = x0	= x	 
			y1 = y
	***********************************************************************************************/ 
	
		y=fsmFilt1B0[axis][SN]*x0+fsmFilt1B1[axis][SN]*x1-fsmFilt1A1[axis][SN]*y1;
		out = y;

		euFsm1[axis][1][SN] = out;									//euFsm1_servo[0][1][1] = out ; y1 = y
		erFsm1[axis][1][SN] = erFsm1[axis][0][SN];  //erFsm1_servo[0][1][1] = erFsm1_servo[0][0][1]; x1 = x0

		return out;
}

//x = FW_location_low_pass
//axis = FSM_X = 0
//SN = 7
//														 (tao1*s + 1)*(tao2*s + 1)
//	二阶超前滞后传递函数：G = ---------------------------
//													    (T1*s + 1) * (T2*s + 1)
float FsmLeadLag2(float x,int axis,int SN)
{
	float x0,x1,x2,y1,y2;
	float y;
	float out;

	erFsm2[axis][0][SN] = x;   //erFsm2[0][0][7] x = FW_location_low_pass

	x0 = erFsm2[axis][0][SN];	 //x0 = erFsm2[0][0][7] = x
	x1 = erFsm2[axis][1][SN];	 //x1 = erFsm2[0][1][7]
	x2 = erFsm2[axis][2][SN];	 //x2 = erFsm2[0][2][7]
	y1 = euFsm2[axis][1][SN];	 //y1 = euFsm2[0][1][7]
	y2 = euFsm2[axis][2][SN];  //y2 = euFsm2[0][2][7]

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
	
				 200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1				     2*1-2*200^2*Tao1*Tao2							  200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1				       2*1-2*200^2*T1*T2								  200^2*T1*T2 - 200*(T1+T2) + 1
		y = --------------------------------------- * x0 + ------------------------------- * x1 + ---------------------------------------- * x2 - ------------------------------- * y1 - ------------------------------- * y2
	           200^2*T1*T2 + 200*(T1+T2) + 1              200^2*T1*T2 + 200*(T1+T2) + 1                200^2*T1*T2 + 200*(T1+T2) + 1             200^2*T1*T2 + 200*(T1+T2) + 1          200^2*T1*T2 + 200*(T1+T2) + 1
		
		out = y
		y2 = y1  ============  euFsm2[0][2][7] = euFsm2[0][1][7]
		y1 = y	 ============  euFsm2[0][1][7] = out = y
		x2 = x1  ============  erFsm2[0][2][7] = erFsm2[0][1][7]
		x1 = x   ============  erFsm2[0][1][7] = erFsm2[0][0][7] = x
	***************************************************************************************************************************/
	
	y=fsmFilt2B0[axis][SN]*x0+fsmFilt2B1[axis][SN]*x1+fsmFilt2B2[axis][SN]*x2-fsmFilt2A1[axis][SN]*y1-fsmFilt2A2[axis][SN]*y2;
	out = y;

	euFsm2[axis][2][SN]= euFsm2[axis][1][SN];
	euFsm2[axis][1][SN]= out;
	erFsm2[axis][2][SN] = erFsm2[axis][1][SN];
	erFsm2[axis][1][SN] = erFsm2[axis][0][SN]; 

	return out;
}

//低通滤波器:Q(s)
float FsmLeadLag3(float x,int axis,int SN)
{
	float x0,x1,x2,x3,y1,y2,y3;
	float y;
	float out;

	erFsm2[axis][0][SN] = x;   //

	x0 = erFsm2[axis][0][SN];	 //x0 = erFsm2[0][0][8] = x
	x1 = erFsm2[axis][1][SN];	 //x1 = erFsm2[0][1][8]
	x2 = erFsm2[axis][2][SN];	 //x2 = erFsm2[0][2][8]
	x3 = erFsm2[axis][3][SN];	 //x3 = erFsm2[0][3][8]
	
	y1 = euFsm2[axis][1][SN];	 //y1 = euFsm2[0][1][8]
	y2 = euFsm2[axis][2][SN];  //y2 = euFsm2[0][2][8]
	y3 = euFsm2[axis][3][SN];  //y2 = euFsm2[0][3][8]

	/**************************************************************************************************************************
												                							3*Tao*N + 1
		fsmFilt3B0[0][8] = fsmTustinBz[0] =  ---------------------------------------
													              	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
			
												                							3*Tao*N + 3
		fsmFilt3B1[0][8] = fsmTustinBz[1] =  ---------------------------------------
		                                    	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
                                        						-	3*Tao*N + 3
		fsmFilt3B2[0][8] = fsmTustinBz[2] =  ---------------------------------------
		                                    	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
																					
		                                    						-	3*Tao*N + 1
		fsmFilt3B3[0][8] = fsmTustinBz[3] =  ---------------------------------------
		                                    	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1																			
		
													              	 -3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1
		fsmFilt3A1[0][8] = fsmTustinAz[1] =  --------------------------------------------
		                                         Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
															           	 3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3
		fsmFilt3A2[0][8] = fsmTustinAz[2] =  --------------------------------------------
		                                         Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
																						
															           	 - Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1
		fsmFilt3A3[0][8] = fsmTustinAz[3] =  --------------------------------------------
		                                         Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1																		
	
											3*Tao*N + 1			                  							3*Tao*N + 3			                  						-	3*Tao*N + 3			                  						-	3*Tao*N + 1                       	 -3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1		       	   3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3             - Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1
		y =  --------------------------------------- * x0 +  --------------------------------------- * x1 +  --------------------------------------- * x2 +  --------------------------------------- * x3 -  -------------------------------------------- * y1 -  -------------------------------------------- * y2 -  -------------------------------------------- * y3 ;
	      	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1         	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1         	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1         	Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1              Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1                Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1               Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
		out = y
		y3 = y3
		y2 = y1  ============  euFsm2[0][2][8] = euFsm2[0][1][8]
		y1 = y	 ============  euFsm2[0][1][8] = out = y
		x3 = x2
		x2 = x1  ============  erFsm2[0][2][8] = erFsm2[0][1][8]
		x1 = x   ============  erFsm2[0][1][8] = erFsm2[0][0][8] = x
	***************************************************************************************************************************/
	
	y=fsmFilt3B0[axis][SN]*x0+fsmFilt3B1[axis][SN]*x1+fsmFilt3B2[axis][SN]*x2+fsmFilt3B3[axis][SN]*x3-fsmFilt3A1[axis][SN]*y1-fsmFilt3A2[axis][SN]*y2-fsmFilt3A3[axis][SN]*y3;
	out = y;

	euFsm2[axis][3][SN]= euFsm2[axis][2][SN];
	euFsm2[axis][2][SN]= euFsm2[axis][1][SN];
	euFsm2[axis][1][SN]= out;
	erFsm2[axis][3][SN] = erFsm2[axis][2][SN];
	erFsm2[axis][2][SN] = erFsm2[axis][1][SN];
	erFsm2[axis][1][SN] = erFsm2[axis][0][SN]; 

	return out;
}

//二阶双线性变换公式
void FsmTustin2(void)
{
	
    fsmTustinT=2/FSM_TS;
		
		//fsmTustinAz[0] = T1*T2 = 1/(2*3.14*0.08) * 1/(2*3.14*0.08) ; fsmTustinAs_servo[1] = T1+T2 = 2/(2*3.14*0.08) ; fsmTustinAs_servo[2] = 1
		//fsmTustinAz[0] = 200^2*T1*T2 + 200*(T1+T2) + 1
		//fsmTustinAz[1] = 2*1-2*200^2*T1*T2
		//fsmTustinAz[2] = 200^2*T1*T2 - 200*(T1+T2) + 1
		fsmTustinAz[0]=(fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (fsmTustinAs[1]*fsmTustinT) + fsmTustinAs[2];
		fsmTustinAz[1]=((2*fsmTustinAs[2]-2*fsmTustinAs[0]*fsmTustinT*fsmTustinT) ) ;
    fsmTustinAz[2]=(fsmTustinAs[0]*fsmTustinT*fsmTustinT)- (fsmTustinAs[1]*fsmTustinT) + fsmTustinAs[2];
		
		//fsmTustinBz[0] = Tao1*Tao2 = 1/(2*3.14*0.5) * 1/(2*3.14*0.5) ; fsmTustinBs_servo[1] = Tao1+Tao2 = 2/(2*3.14*0.5) ; fsmTustinBs_servo[2] = 1 ;
		//fsmTustinBz[0] = 200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1
		//fsmTustinBz[1] = 2*1-2*200^2*Tao1*Tao2
		//fsmTustinBz[2] = 200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1
		fsmTustinBz[0]=(fsmTustinBs[0]*fsmTustinT*fsmTustinT) + (fsmTustinBs[1]*fsmTustinT) + fsmTustinBs[2];
		fsmTustinBz[1]=(2*fsmTustinBs[2]-2*fsmTustinBs[0]*fsmTustinT*fsmTustinT) ;
    fsmTustinBz[2]=fsmTustinBs[0]*fsmTustinT*fsmTustinT- (fsmTustinBs[1]*fsmTustinT) + fsmTustinBs[2];
	
//==========================================================================================	
		/*******************************************************************************
											200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1	
		fsmTustinBz[0] = --------------------------------------
													200^2*T1*T2 + 200*(T1+T2) + 1
			
												 2*1-2*200^2*Tao1*Tao2	
		fsmTustinBz[1] = -------------------------------
		                  200^2*T1*T2 + 200*(T1+T2) + 1
		
											200^2*Tao1*Tao2 - 200*(Tao1+Tao2) + 1
		fsmTustinBz[2] = ---------------------------------------
		                     200^2*T1*T2 + 200*(T1+T2) + 1
		
		*******************************************************************************/
	
		fsmTustinBz[0]=fsmTustinBz[0] / fsmTustinAz[0];
		fsmTustinBz[1]=fsmTustinBz[1] / fsmTustinAz[0];
    fsmTustinBz[2]=fsmTustinBz[2] / fsmTustinAz[0];

		/*******************************************************************************
													 2*1-2*200^2*T1*T2	
		fsmTustinAz[1] = -------------------------------
		                  200^2*T1*T2 + 200*(T1+T2) + 1
		
											200^2*T1*T2 - 200*(T1+T2) + 1
		fsmTustinAz[2] = -------------------------------
		                  200^2*T1*T2 + 200*(T1+T2) + 1
		
		*******************************************************************************/
		
		fsmTustinAz[1]=fsmTustinAz[1] / fsmTustinAz[0] ;
    fsmTustinAz[2]=fsmTustinAz[2] /fsmTustinAz[0];
		fsmTustinAz[0]=1.0;
}

/*
										 2			1 - z^(-1)
双线性变换公式：S = ---- * ------------		,Ts是周期，采样时间
										 Ts			1 + z^(-1)
*/

//一阶双线性变换公式
void FsmTustin1(void)
 {
		//TustinT=2/0.000064;
		//TustinT=2/0.000256;
		fsmTustinT=2/FSM_TS;  //fsmTustinT = 2000
		 
		//fsmTustinAs[0] = T1 = 1 /（3.14*2*80）；fsmTustinT = 200；fsmTustinAs[1] = 1	 
		//fsmTustinAz[0] =  200*T1 + 1
		//fsmTustinAz[1] = -200*T1 + 1
		fsmTustinAz[0]= (fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		fsmTustinAz[1]=-(fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		
		//fsmTustinBs[0] = Tao1 = 0 ；fsmTustinT = 200 ；fsmTustinBs[1] = 1
		//fsmTustinBz[0] =   200*Tao1 + 1 = 1
		//fsmTustinBz[1] = - 200*Tao1 + 1 = 1
		fsmTustinBz[0]= (fsmTustinBs[0]*fsmTustinT) + (fsmTustinBs[1]) ;
		fsmTustinBz[1]=-(fsmTustinBs[0]*fsmTustinT) + (fsmTustinBs[1]) ;
		
	//==========================================================================================	
	
		/*
													200*Tao1 + 1				 1
				fsmTustinBz[0] = -------------- = ------------  	
													200*T1 + 1			 200*T1 + 1
				
													- 200*Tao1 + 1				 1
				fsmTustinBz[1] = ---------------- = ------------
													  200*T1 + 1		   200*T1 + 1
				
													- 200*T1 + 1
				fsmTustinAz[1] = -------------- 
													  200*T1 + 1			
				
				fsmTustinAz[0]=1.0;
		*/
		
		fsmTustinBz[0]=fsmTustinBz[0] / fsmTustinAz[0];
		fsmTustinBz[1]=fsmTustinBz[1] / fsmTustinAz[0];
			
		fsmTustinAz[1]=fsmTustinAz[1] / fsmTustinAz[0] ;
		fsmTustinAz[0]=1.0;
}

void AskFsmLeadLag2Para(int axis,int SN)
{
		double T1,T2;
		double Tao1,Tao2;
		float fsmLeadLag2Tao1Hz,fsmLeadLag2Tao2Hz;
		float fsmLeadLag2T1Hz,fsmLeadLag2T2Hz;
	
		//fsmLeadLag2Tao1Hz = fsmTao1_wn[0][7] = X7_fsmTao1 = 1.6 (hz)
		//fsmLeadLag2Tao2Hz = fsmTao2_kesi[0][7] = X7_fsmTao2 = 1.6 (hz)
		//fsmLeadLag2T1Hz = fsmT1[0][7] = X7_fsmT1 = 0.2 (hz)
		//fsmLeadLag2T2Hz = fsmT1[0][7] = X7_fsmT2 = 0.2 (hz)	
		fsmLeadLag2Tao1Hz=fsmTao1_wn[axis][SN];
		fsmLeadLag2Tao2Hz=fsmTao2_kesi[axis][SN];
		fsmLeadLag2T1Hz=fsmT1[axis][SN];
		fsmLeadLag2T2Hz=fsmT2[axis][SN];
		
		
    Tao1=1/(FSM_PI*2*fsmLeadLag2Tao1Hz);	//Tao1 = 1/(2*3.14*fsmLeadLag2Tao1Hz) = 1/(2*3.14*1.6)
		Tao2=1/(FSM_PI*2*fsmLeadLag2Tao2Hz);	//Tao2 = 1/(2*3.14*fsmLeadLag2Tao2Hz) = 1/(2*3.14*1.6)
		fsmTustinBs[0] = Tao1*Tao2 ;					//fsmTustinBs[0] = Tao1*Tao2 = 1/(2*3.14*1.6) * 1/(2*3.14*1.6)
		fsmTustinBs[1] = Tao1+Tao2 ;					//fsmTustinBs[1] = Tao1+Tao2 = 2/(2*3.14*1.6)
		fsmTustinBs[2] = 1 ;
		T1=1/(FSM_PI*2*fsmLeadLag2T1Hz);			//T1 = 1/(2*3.14*0.2)
		T2=1/(FSM_PI*2*fsmLeadLag2T2Hz);			//T2 = 1/(2*3.14*0.2)
		fsmTustinAs[0] = T1*T2 ;							//fsmTustinAs[0] = T1*T2 = 1/(2*3.14*0.2) * 1/(2*3.14*0.2)
		fsmTustinAs[1] = T1+T2 ;							//fsmTustinAs[1] = T1+T2 = 2/(2*3.14*0.2)
		fsmTustinAs[2] = 1 ;	

		FsmTustin2();
		
		/*******************************************************************************
											                   200^2*Tao1*Tao2 + 200*(Tao1+Tao2) + 1	
		fsmFilt2B0[0][7] = fsmTustinBz[0] = --------------------------------------
													                  200^2*T1*T2 + 200*(T1+T2) + 1
			
												                    2*1-2*200^2*Tao1*Tao2	
		fsmFilt2B1[0][7] = fsmTustinBz[1] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
                                                     1	
		fsmFilt2B2[0][7] = fsmTustinBz[2] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
													                    2*1-2*200^2*T1*T2	
		fsmFilt2A1[0][7] = fsmTustinAz[1] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
															                         1	
		fsmFilt2A2[0][7] = fsmTustinAz[2] = -------------------------------
		                                     200^2*T1*T2 + 200*(T1+T2) + 1
		
		*******************************************************************************/
		
		fsmFilt2B0[axis][SN]=fsmTustinBz[0];	//双线性变化的系数存在不同轴的不同列
		fsmFilt2B1[axis][SN]=fsmTustinBz[1];
		fsmFilt2B2[axis][SN]=fsmTustinBz[2]; 
		fsmFilt2A1[axis][SN]=fsmTustinAz[1];	
		fsmFilt2A2[axis][SN]=fsmTustinAz[2];
}

float fsmLeadLag1Tao1Hz;
float fsmLeadLag1T1Hz;
void AskFsmLeadLag1Para(int axis,int SN)   //当SN = 1；axis = 0时
{
	double T1Rad,T1;
	double Tao1Rad,Tao1;
	
	//fsmTao1_wn[0][1] = X1_fsmTao1 = 0；fsmT1[0][1] = X1_fsmT1 = 80
	fsmLeadLag1Tao1Hz=fsmTao1_wn[axis][SN];   //fsmLeadLag1Tao1Hz = 0
	fsmLeadLag1T1Hz=fsmT1[axis][SN];				  //fsmLeadLag1T1Hz = 80 （hz）

	if(SN == 10)
	{
			Tao1 = fsmLeadLag1Tao1Hz;
			T1 = fsmLeadLag1T1Hz;
	}
	else
	{
			T1Rad=FSM_PI*2*fsmLeadLag1T1Hz;				//T1Rad = 2*3.14*80
			T1=1/T1Rad;														//T1 = 1/(2*3.14*80)
			
			Tao1Rad=FSM_PI*2*fsmLeadLag1Tao1Hz;		//Tao1Rad = 2*3.14*0 = 0
			if(fsmLeadLag1Tao1Hz == 0)
			{
				Tao1 = 0;														//Tao1 = 0
			}
			else
			{
				Tao1=1/Tao1Rad;											//Tao1 = 1/(2*3.14*fsmLeadLag1Tao1Hz)
			}
	}
	fsmTustinBs[0] = Tao1 ;								//fsmTustinBs[0] = Tao1 = 0
	fsmTustinBs[1] = 1 ;									//fsmTustinBs[1] = 1 
	fsmTustinAs[0] = T1 ;									//fsmTustinAs[0] = T1 = 1/(2*3.14*80)
	fsmTustinAs[1] = 1 ;									//fsmTustinAs[1] = 1 

	FsmTustin1();
	
	/*
													               200*Tao1 + 1				   1
	  fsmFilt1B0[0][1] = fsmTustinBz[0] = -------------- = ------------ 	
													                200*T1 + 1			 200*T1 + 1
				
													               - 200*Tao1 + 1				  1
		fsmFilt1B1[0][1] = fsmTustinBz[1] = ---------------- = ------------
													                  200*T1 + 1		  200*T1 + 1
				
													               - 200*T1 + 1			
		fsmFilt1A1[0][1] = fsmTustinAz[1] = -------------- 
													                 200*T1 + 1		     		
			
	*/
	
	fsmFilt1B0[axis][SN]=fsmTustinBz[0];	
	fsmFilt1B1[axis][SN]=fsmTustinBz[1];

	fsmFilt1A1[axis][SN]=fsmTustinAz[1];	
}
void Ask2ParaShake(int axis,int SN)
{
	double KesiXieZhen1;
	double OmigaXieZhen1;
	double omigaRad;
	
	OmigaXieZhen1=fsmTao1_wn[axis][SN];
	KesiXieZhen1=fsmTao2_kesi[axis][SN];
	
	omigaRad=FSM_PI*2*OmigaXieZhen1;
	
	//As是分母
	fsmTustinBs[0] =  0;
	fsmTustinBs[1] =  0;
	fsmTustinBs[2] =  1;
    fsmTustinAs[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinAs[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinAs[2] = 1 ;
	
	FsmTustin2();

	fsmFilt2B0[axis][SN]=fsmTustinBz[0];	
	fsmFilt2B1[axis][SN]=fsmTustinBz[1];
	fsmFilt2B2[axis][SN]=fsmTustinBz[2]; 
	fsmFilt2A1[axis][SN]=fsmTustinAz[1];	
	fsmFilt2A2[axis][SN]=fsmTustinAz[2];

}
void Ask2ParaNotchfilter(int axis,int SN)
{
	double KesiXieZhen1;
	double OmigaXieZhen1;
	double omigaRad;
	
	OmigaXieZhen1=fsmTao1_wn[axis][SN];
	KesiXieZhen1=fsmTao2_kesi[axis][SN];
	
	omigaRad=FSM_PI*2*OmigaXieZhen1;
	
	//As是分母
	fsmTustinBs[0] =  1/(omigaRad*omigaRad);
	fsmTustinBs[1] =  0;
	fsmTustinBs[2] =  1;
    fsmTustinAs[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinAs[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinAs[2] = 1 ;
	
	FsmTustin2();

	fsmFilt2B0[axis][SN]=fsmTustinBz[0];	
	fsmFilt2B1[axis][SN]=fsmTustinBz[1];
	fsmFilt2B2[axis][SN]=fsmTustinBz[2]; 
	fsmFilt2A1[axis][SN]=fsmTustinAz[1];	
	fsmFilt2A2[axis][SN]=fsmTustinAz[2];

}
void AskFsmNotchPara(int axis,int SN)     //？
{
	double T1,T2;
	double Tao1,Tao2;
	float fsmLeadLag2Tao1Hz,fsmLeadLag2Tao2Hz;
	float fsmLeadLag2T1Hz,fsmLeadLag2T2Hz;
	double omigaRad;
	double KesiXieZhen1;
	double OmigaXieZhen1;
	
	OmigaXieZhen1=fsmTao1_wn[axis][SN];
	KesiXieZhen1=fsmTao2_kesi[axis][SN];
	fsmLeadLag2T1Hz=fsmT1[axis][SN];
	fsmLeadLag2T2Hz=fsmT2[axis][SN];
	
	omigaRad=FSM_PI*2*OmigaXieZhen1;

	fsmTustinBs[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinBs[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinBs[2] = 1 ;
	T1=1/(FSM_PI*2*fsmLeadLag2T1Hz);
	T2=1/(FSM_PI*2*fsmLeadLag2T2Hz);
	fsmTustinAs[0] = T1*T2 ;
	fsmTustinAs[1] = T1+T2 ;
	fsmTustinAs[2] = 1 ;

	FsmTustin2();

	fsmFilt2B0[axis][SN]=fsmTustinBz[0];	
	fsmFilt2B1[axis][SN]=fsmTustinBz[1];
	fsmFilt2B2[axis][SN]=fsmTustinBz[2]; 
	fsmFilt2A1[axis][SN]=fsmTustinAz[1];	
	fsmFilt2A2[axis][SN]=fsmTustinAz[2];
}
float MeanFilter(int axis,float x,int n)   //？
{
	float sum=0;
	float meanData=0;
	int i;
	for(i=0;i<n;i++)
	{
		saveData[axis][n-i]=saveData[axis][n-i-1];
	}
	saveData[axis][0]=x;
	for(i=0;i<n;i++)
	{
		sum=sum+saveData[axis][i];
	}
	meanData=sum/n;
	return meanData;
	
}

//									 						 3*Tao*S + 1
//低通滤波器:Q(s) = ---------------------------------------
//									 Tao^3*S^3 + 3*Tao^2*S^2 + 3*Tao*S + 1
//AskFsmLeadLag3Para(0,8)
float fsmLeadLag3Tao;
void AskFsmLeadLag3Para(int axis,int SN)
{
		double Tao;
		fsmTustinT = 2/FSM_Qs;                     //fsmTustinT = 2000
	
		//fsmTao1_wn[0][8] = X8_fsmTao1 = 0.001；
		fsmLeadLag3Tao=fsmTao1_wn[axis][SN];   		//fsmLeadLag3Tao = 0.001
		
		Tao = fsmLeadLag3Tao;
		
		fsmTustinAs[0] = Tao ;								//fsmTustinAs[0] = Tao = 0.001
		fsmTustinAs[1] = 1 ;									//fsmTustinAs[1] = 1 
		
		//fsmTustinAz[0] = Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		//fsmTustinAz[1] = -3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1
		//fsmTustinAz[2] = 3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3
		//fsmTustinAz[3] = - Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1
		fsmTustinAz[0] = (fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		fsmTustinAz[1] = (-3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1]) ;
		fsmTustinAz[2] = (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1]) ;
		fsmTustinAz[3] = (-fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		
		//fsmTustinBz[0] = 3*Tao*N + 1
		//fsmTustinBz[1] = 3*Tao*N + 3
		//fsmTustinBz[2] = - 3*Tao*N + 3
		//fsmTustinBz[3] = - 3*Tao*N + 1
		fsmTustinBz[0] = (3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		fsmTustinBz[1] = (3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1]) ;
		fsmTustinBz[2] = (-3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1]) ;
		fsmTustinBz[3] = (-3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
		
//==========================================================================================	
		/*******************************************************************************
																	3*Tao*N + 1
		fsmTustinBz[0] = ---------------------------------------
											Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
			
												          3*Tao*N + 3	
		fsmTustinBz[1] = ---------------------------------------
		                  Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
																- 3*Tao*N + 3
		fsmTustinBz[2] = ---------------------------------------
		                  Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
																- 3*Tao*N + 1
		fsmTustinBz[3] = ---------------------------------------
		                  Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
		*******************************************************************************/				
		fsmTustinBz[0] = fsmTustinBz[0] / fsmTustinAz[0];
		fsmTustinBz[1] = fsmTustinBz[1] / fsmTustinAz[0];
    fsmTustinBz[2] = fsmTustinBz[2] / fsmTustinAz[0];
		fsmTustinBz[3] = fsmTustinBz[3] / fsmTustinAz[0];
		
/*******************************************************************************
											 -3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1
		fsmTustinAz[1] = --------------------------------------------
		                     Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
											  3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3
		fsmTustinAz[2] = --------------------------------------------
		                     Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		
											 - Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1
		fsmTustinAz[3] = --------------------------------------------
		                     Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1
		*******************************************************************************/
		
		fsmTustinAz[1] = fsmTustinAz[1] / fsmTustinAz[0];
    fsmTustinAz[2] = fsmTustinAz[2] / fsmTustinAz[0];
		fsmTustinAz[3] = fsmTustinAz[3] / fsmTustinAz[0];
		fsmTustinAz[0] = 1.0;
		
		fsmFilt3B0[axis][SN] = fsmTustinBz[0];			//双线性变化的系数存在不同轴的不同列
		fsmFilt3B1[axis][SN] = fsmTustinBz[1];
		fsmFilt3B2[axis][SN] = fsmTustinBz[2]; 
		fsmFilt3B3[axis][SN] = fsmTustinBz[3]; 
		fsmFilt3A1[axis][SN] = fsmTustinAz[1];	
		fsmFilt3A2[axis][SN] = fsmTustinAz[2];
		fsmFilt3A3[axis][SN] = fsmTustinAz[3];
}


/*																			km
  电机及其负载的传递函数:Gn(S) =  -----------------
  																(Te*S+1)*(Tm*S+1)
  						J*Ra						1						 La
  其中：Tm = ------  ;  km = ---  ;  Te = ----
  						kb*ki						ke					 Ra
	ki:电机的转矩系数  La:电枢绕组的电感  Ra:电枢绕组的电阻
	Jm:电机的转动惯量  JL:负载的转动惯量
	L = Jm + JL
*/
float fsmLeadLag4Tao;
float Km ;
float Tm ;
float Te ;
float J  ;
float Jm = 2;			//Jm:电机的转动惯量 
float JL = 0;			//JL:负载的转动惯量
float Ra = 1;			//Ra:电枢绕组的电阻
float La = 0.01;	//La:电枢绕组的电感
float Kb = 2;			//
float Ki = 2;			//ki:电机的转矩系数 
float Ke = 1;			//

//AskFsmLeadLag4Para(0,9)
void AskFsmLeadLag4Para(int axis,int SN)
{
		double Tao;
		fsmTustinT = 2/FSM_Guanceqi;                     //fsmTustinT = 2000
		
		J = Jm + JL;					//2
		Tm = (J*Ra)/(Kb*Ki);	//0.5
		Km = 1/Ke;						//1
		Te = La/Ra;       		//0.01
		
		//fsmTao1_wn[0][9] = X8_fsmTao1 = 0.001；
		fsmLeadLag4Tao = fsmTao1_wn[axis][SN];   		//fsmLeadLag3Tao = 0.001
		
		Tao = fsmLeadLag4Tao;
	
		fsmTustinAs[0] = Tao ;								//fsmTustinAs[0] = Tao = 0.001
		fsmTustinAs[1] = 1 ;									//fsmTustinAs[1] = 1 
		
		//fsmTustinAz[0] = Km*(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		//fsmTustinAz[1] = Km*(-3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1)
		//fsmTustinAz[2] = Km*(3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3)
		//fsmTustinAz[3] = Km*(- Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1)
		fsmTustinAz[0] = Km*((fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1])) ;
		fsmTustinAz[1] = Km*((-3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1])) ;
		fsmTustinAz[2] = Km*((3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinT) + 3*(fsmTustinAs[1])) ;
		fsmTustinAz[3] = Km*((-fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*fsmTustinAs[0]*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1])) ;
		
		//fsmTustinBz[0] = 3*Tao*Te*Tm*N^3 + (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 + (3*Tao + Tm + Te)*N + 1
		//fsmTustinBz[1] = -9*Tao*Te*Tm*N^3 - (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 + (3*Tao + Tm + Te)*N + 3
		//fsmTustinBz[2] = 9*Tao*Te*Tm*N^3 - (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 - (3*Tao + Tm + Te)*N + 3
		//fsmTustinBz[3] = -3*Tao*Te*Tm*N^3 + (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 - (3*Tao + Tm + Te)*N + 1
		fsmTustinBz[0] = (3*fsmTustinAs[0]*Te*Tm*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*Te + 3*fsmTustinAs[0]*Tm + Te*Tm)*fsmTustinT*fsmTustinT + (3*fsmTustinAs[0] + Tm + Te)*fsmTustinT + 1 ;
		fsmTustinBz[1] = (-9*fsmTustinAs[0]*Te*Tm*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*Te + 3*fsmTustinAs[0]*Tm + Te*Tm)*fsmTustinT*fsmTustinT + (3*fsmTustinAs[0] + Tm + Te)*fsmTustinT + 3 ;
		fsmTustinBz[2] = (9*fsmTustinAs[0]*Te*Tm*fsmTustinT*fsmTustinT*fsmTustinT) - (3*fsmTustinAs[0]*Te + 3*fsmTustinAs[0]*Tm + Te*Tm)*fsmTustinT*fsmTustinT - (3*fsmTustinAs[0] + Tm + Te)*fsmTustinT + 3 ;
		fsmTustinBz[3] = (-3*fsmTustinAs[0]*Te*Tm*fsmTustinT*fsmTustinT*fsmTustinT) + (3*fsmTustinAs[0]*Te + 3*fsmTustinAs[0]*Tm + Te*Tm)*fsmTustinT*fsmTustinT - (3*fsmTustinAs[0] + Tm + Te)*fsmTustinT + 1 ;
		
//==========================================================================================	
		/*******************************************************************************
											3*Tao*Te*Tm*N^3 + (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 + (3*Tao + Tm + Te)*N + 1
		fsmTustinBz[0] = -------------------------------------------------------------------------------
																		Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
			
											-9*Tao*Te*Tm*N^3 - (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 + (3*Tao + Tm + Te)*N + 3
		fsmTustinBz[1] = -------------------------------------------------------------------------------
																		Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		
											9*Tao*Te*Tm*N^3 - (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 - (3*Tao + Tm + Te)*N + 3
		fsmTustinBz[2] = -------------------------------------------------------------------------------
																		Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		
											-3*Tao*Te*Tm*N^3 + (3*Tao*Te + 3*Tao*Tm + Te*Tm)*N^2 - (3*Tao + Tm + Te)*N + 1
		fsmTustinBz[3] = -------------------------------------------------------------------------------
																		Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		
		*******************************************************************************/				
		fsmTustinBz[0] = fsmTustinBz[0] / fsmTustinAz[0];
		fsmTustinBz[1] = fsmTustinBz[1] / fsmTustinAz[0];
    fsmTustinBz[2] = fsmTustinBz[2] / fsmTustinAz[0];
		fsmTustinBz[3] = fsmTustinBz[3] / fsmTustinAz[0];
		
/*******************************************************************************
											Km*(-3*Tao^3*N^3 - 3*Tao^2*N^2 + 3*Tao*N + 1)
		fsmTustinAz[1] = ----------------------------------------------
		                    Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		
											Km*(3*Tao^3*N^3 - 3*Tao^2*N^2 - 3*Tao*N + 3)
		fsmTustinAz[2] = ----------------------------------------------
		                    Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		
											Km*(- Tao^3*N^3 + 3*Tao^2*N^2 - 3*Tao*N + 1)
		fsmTustinAz[3] = ----------------------------------------------
		                    Km(Tao^3*N^3 + 3*Tao^2*N^2 + 3*Tao*N + 1)
		*******************************************************************************/
		
		fsmTustinAz[1] = fsmTustinAz[1] / fsmTustinAz[0];
    fsmTustinAz[2] = fsmTustinAz[2] / fsmTustinAz[0];
		fsmTustinAz[3] = fsmTustinAz[3] / fsmTustinAz[0];
		fsmTustinAz[0] = 1.0;
		
		fsmFilt3B0[axis][SN] = fsmTustinBz[0];			//双线性变化的系数存在不同轴的不同列
		fsmFilt3B1[axis][SN] = fsmTustinBz[1];
		fsmFilt3B2[axis][SN] = fsmTustinBz[2]; 
		fsmFilt3B3[axis][SN] = fsmTustinBz[3]; 
		fsmFilt3A1[axis][SN] = fsmTustinAz[1];	
		fsmFilt3A2[axis][SN] = fsmTustinAz[2];
		fsmFilt3A3[axis][SN] = fsmTustinAz[3];
}