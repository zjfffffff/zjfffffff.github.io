#ifndef __FSMCOMPENSATOR_h
#define __FSMCOMPENSATOR_h

#define FSM_X 0
#define FSM_Y 1

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

void AskFsmLeadLag3Para(int axis,int SN);
float FsmLeadLag3(float x,int axis,int SN);

void AskFsmLeadLag4Para(int axis,int SN);
#endif 
