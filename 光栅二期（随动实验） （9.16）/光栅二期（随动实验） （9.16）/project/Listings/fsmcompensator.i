#line 1 "..\\User\\FsmCompensator.c"
#line 1 "..\\User\\FsmCompensator.h"



void AskfsmFilterPara(void);
float FsmLeadLag1(float x,int axis,int SN);
float FsmLeadLag2(float x,int axis,int SN);
void FsmTustin2(void);
void FsmTustin1(void);
void AskFsmLeadLag2Para(int axis,int SN);
void AskFsmLeadLag1Para(int axis,int SN);
void AskFsmNotchPara(int axis,int SN);
float MeanFilter(int axis,float x,int n);
void Ask2ParaShake(int axis,int SN);
void Ask2ParaNotchfilter(int axis,int SN);
#line 2 "..\\User\\FsmCompensator.c"
#line 1 "..\\User\\FsmCompensatorPara.h"



float X0_fsm_wn=24.5;
float X0_fsm_kesi=0.16;
float X0_fsm_T1=1;
float X0_fsm_T2=400;

float X1_fsmTao1=10;
float X1_fsmTao2=0;
float X1_fsmT1=0.05;
float X1_fsmT2=0;

float X2_fsmTao1=0;
float X2_fsmTao2=0;
float X2_fsmT1=10;
float X2_fsmT2=0;

float X3_fsmTao1=0.001;
float X3_fsmTao2=0.001;
float X3_fsmT1=1;
float X3_fsmT2=1;

float X4_fsmTao1=0;
float X4_fsmTao2=0;
float X4_fsmT1=7;
float X4_fsmT2=0;


float X5_fsmTao1=60;     
float X5_fsmTao2=0.5;    
float X5_fsmT1=0;     
float X5_fsmT2=0;


float X6_fsmTao1=482;
float X6_fsmTao2=0.8;
float X6_fsmT1=0;
float X6_fsmT2=0;

float X7_fsmTao1=0;
float X7_fsmTao2=0;
float X7_fsmT1=1000;
float X7_fsmT2=1000;




#line 3 "..\\User\\FsmCompensator.c"

#line 10 "..\\User\\FsmCompensator.c"





float fsmLeadLag1T1Hz=0;
float fsmLeadLag1Tao1Hz=0;

float fsmLeadLag2T1Hz=0;
float fsmLeadLag2Tao1Hz=0;
float fsmLeadLag2T2Hz=0;  
float fsmLeadLag2Tao2Hz=0;


double fsmTustinT;
double fsmTustinBz[3],fsmTustinBs[3],fsmTustinAz[3],fsmTustinAs[3];

float erFsm1[2][3][10];
float euFsm1[2][3][10];
float erFsm2[2][3][10];
float euFsm2[2][3][10];

float fsmFilt1B0[2][10],fsmFilt1B1[2][10],fsmFilt1A1[2][10];
float fsmFilt2B0[2][10],fsmFilt2B1[2][10],fsmFilt2B2[2][10],fsmFilt2A1[2][10],fsmFilt2A2[2][10];

float fsmTao1_wn[2][10]={0};
float fsmTao2_kesi[2][10]={0};
float fsmT1[2][10]={0};
float fsmT2[2][10]={0};
float saveData[2][100]={0};

void AskfsmFilterPara(void)
{
	fsmTao1_wn[0][0]=X0_fsm_wn;  fsmTao2_kesi[0][0]=X0_fsm_kesi;  fsmT1[0][0]=X0_fsm_T1;  fsmT2[0][0]=X0_fsm_T2;
	fsmTao1_wn[0][1]=X1_fsmTao1; fsmTao2_kesi[0][1]=X1_fsmTao2;   fsmT1[0][1]=X1_fsmT1;   fsmT2[0][1]=X1_fsmT2;
	fsmTao1_wn[0][2]=X2_fsmTao1; fsmTao2_kesi[0][2]=X2_fsmTao2;   fsmT1[0][2]=X2_fsmT1;   fsmT2[0][2]=X2_fsmT2;
	fsmTao1_wn[0][3]=X3_fsmTao1; fsmTao2_kesi[0][3]=X3_fsmTao2;   fsmT1[0][3]=X3_fsmT1;   fsmT2[0][3]=X3_fsmT2;
	fsmTao1_wn[0][4]=X4_fsmTao1; fsmTao2_kesi[0][4]=X4_fsmTao2;   fsmT1[0][4]=X4_fsmT1;   fsmT2[0][4]=X4_fsmT2;
	fsmTao1_wn[0][5]=X5_fsmTao1; fsmTao2_kesi[0][5]=X5_fsmTao2;   fsmT1[0][5]=X5_fsmT1;   fsmT2[0][5]=X5_fsmT2;
	fsmTao1_wn[0][6]=X6_fsmTao1; fsmTao2_kesi[0][6]=X6_fsmTao2;   fsmT1[0][6]=X6_fsmT1;   fsmT2[0][6]=X6_fsmT2;
	fsmTao1_wn[0][7]=X7_fsmTao1; fsmTao2_kesi[0][7]=X7_fsmTao2;   fsmT1[0][7]=X7_fsmT1;   fsmT2[0][7]=X7_fsmT2;
	




	
	AskFsmNotchPara(0,0);       
   	AskFsmLeadLag1Para(0,1);    
	AskFsmLeadLag1Para(0,2);    
	AskFsmLeadLag2Para(0,3);    
	AskFsmLeadLag1Para(0,4);    
	Ask2ParaShake(0,5);    
	Ask2ParaNotchfilter(0,6);    
	AskFsmLeadLag2Para(0,7);    
	



}

float FsmLeadLag1(float x,int axis,int SN)  
{
	float x0,x1,y1;
	float y;   
    float out;
	
	erFsm1[axis][0][SN] = x;

	x0 = erFsm1[axis][0][SN];
	x1 = erFsm1[axis][1][SN];
	y1 = euFsm1[axis][1][SN];

	y=fsmFilt1B0[axis][SN]*x0+fsmFilt1B1[axis][SN]*x1-fsmFilt1A1[axis][SN]*y1;
	out = y;

	euFsm1[axis][1][SN] = out;
	erFsm1[axis][1][SN] = erFsm1[axis][0][SN]; 

	return out;
}

float FsmLeadLag2(float x,int axis,int SN)
{
	float x0,x1,x2,y1,y2;
	float y;
	float out;

	erFsm2[axis][0][SN] = x;

	x0 = erFsm2[axis][0][SN];
	x1 = erFsm2[axis][1][SN];
	x2 = erFsm2[axis][2][SN];
	y1 = euFsm2[axis][1][SN];
	y2 = euFsm2[axis][2][SN];

	y=fsmFilt2B0[axis][SN]*x0+fsmFilt2B1[axis][SN]*x1+fsmFilt2B2[axis][SN]*x2-fsmFilt2A1[axis][SN]*y1-fsmFilt2A2[axis][SN]*y2;
	out = y;

	euFsm2[axis][2][SN]= euFsm2[axis][1][SN];
	euFsm2[axis][1][SN]= out;
	erFsm2[axis][2][SN] = erFsm2[axis][1][SN];
	erFsm2[axis][1][SN] = erFsm2[axis][0][SN]; 

	return out;
}


void FsmTustin2(void)
{
	
    fsmTustinT=2/0.0005;
		
	fsmTustinAz[0]=(fsmTustinAs[0]*fsmTustinT*fsmTustinT) + (fsmTustinAs[1]*fsmTustinT) + fsmTustinAs[2];
	fsmTustinAz[1]=((2*fsmTustinAs[2]-2*fsmTustinAs[0]*fsmTustinT*fsmTustinT) ) ;
    fsmTustinAz[2]=(fsmTustinAs[0]*fsmTustinT*fsmTustinT)- (fsmTustinAs[1]*fsmTustinT) + fsmTustinAs[2];
	
	fsmTustinBz[0]=(fsmTustinBs[0]*fsmTustinT*fsmTustinT) + (fsmTustinBs[1]*fsmTustinT) + fsmTustinBs[2];
	fsmTustinBz[1]=(2*fsmTustinBs[2]-2*fsmTustinBs[0]*fsmTustinT*fsmTustinT) ;
    fsmTustinBz[2]=fsmTustinBs[0]*fsmTustinT*fsmTustinT- (fsmTustinBs[1]*fsmTustinT) + fsmTustinBs[2];
	

	
	fsmTustinBz[0]=fsmTustinBz[0] / fsmTustinAz[0];
	fsmTustinBz[1]=fsmTustinBz[1] / fsmTustinAz[0];
    fsmTustinBz[2]=fsmTustinBz[2] / fsmTustinAz[0];

	
	fsmTustinAz[1]=fsmTustinAz[1] / fsmTustinAz[0] ;
    fsmTustinAz[2]=fsmTustinAz[2] /fsmTustinAz[0];
	fsmTustinAz[0]=1.0;

}

void FsmTustin1(void)
 {
	
	
	fsmTustinT=2/0.0005;
	fsmTustinAz[0]= (fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
	fsmTustinAz[1]=-(fsmTustinAs[0]*fsmTustinT) + (fsmTustinAs[1]) ;
  
	fsmTustinBz[0]= (fsmTustinBs[0]*fsmTustinT) + (fsmTustinBs[1]) ;
	fsmTustinBz[1]=-(fsmTustinBs[0]*fsmTustinT) + (fsmTustinBs[1]) ;
 	

	
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
	
	fsmLeadLag2Tao1Hz=fsmTao1_wn[axis][SN];
	fsmLeadLag2Tao2Hz=fsmTao2_kesi[axis][SN];
	fsmLeadLag2T1Hz=fsmT1[axis][SN];
	fsmLeadLag2T2Hz=fsmT2[axis][SN];
	
    Tao1=1/(3.14159265358979323846*2*fsmLeadLag2Tao1Hz);
	Tao2=1/(3.14159265358979323846*2*fsmLeadLag2Tao2Hz);
	fsmTustinBs[0] = Tao1*Tao2 ;
	fsmTustinBs[1] = Tao1+Tao2 ;
	fsmTustinBs[2] = 1 ;
	T1=1/(3.14159265358979323846*2*fsmLeadLag2T1Hz);
	T2=1/(3.14159265358979323846*2*fsmLeadLag2T2Hz);
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

float fsmLeadLag1Tao1Hz;
float fsmLeadLag1T1Hz;
void AskFsmLeadLag1Para(int axis,int SN)   
{
	double T1Rad,T1;
	double Tao1Rad,Tao1;
	
	fsmLeadLag1Tao1Hz=fsmTao1_wn[axis][SN];   
	fsmLeadLag1T1Hz=fsmT1[axis][SN];

	
	T1Rad=3.14159265358979323846*2*fsmLeadLag1T1Hz;
	T1=1/T1Rad;
	
	Tao1Rad=3.14159265358979323846*2*fsmLeadLag1Tao1Hz;
	if(fsmLeadLag1Tao1Hz == 0)
	{
		Tao1 = 0;
	}
	else
	{
		Tao1=1/Tao1Rad;
	}
	
	
	fsmTustinBs[0] = Tao1 ;
	fsmTustinBs[1] = 1 ;
	fsmTustinAs[0] = T1 ;
	fsmTustinAs[1] = 1 ;

	FsmTustin1();

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
	
	omigaRad=3.14159265358979323846*2*OmigaXieZhen1;
	
	
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
	
	omigaRad=3.14159265358979323846*2*OmigaXieZhen1;
	
	
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
void AskFsmNotchPara(int axis,int SN)     
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
	
	omigaRad=3.14159265358979323846*2*OmigaXieZhen1;

	fsmTustinBs[0] = 1/(omigaRad*omigaRad) ;
	fsmTustinBs[1] = 2*KesiXieZhen1/(omigaRad) ;
	fsmTustinBs[2] = 1 ;
	T1=1/(3.14159265358979323846*2*fsmLeadLag2T1Hz);
	T2=1/(3.14159265358979323846*2*fsmLeadLag2T2Hz);
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
float MeanFilter(int axis,float x,int n)   
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


