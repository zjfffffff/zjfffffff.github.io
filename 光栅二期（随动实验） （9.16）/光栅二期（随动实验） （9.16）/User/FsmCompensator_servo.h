#ifndef __FSMCOMPENSATOR_servo_h
#define __FSMCOMPENSATOR_servo_h

void AskfsmFilterPara_servo(void);
float FsmLeadLag1_servo(float x,int axis,int SN);
float FsmLeadLag2_servo(float x,int axis,int SN);
void FsmTustin2_servo_servo(void);
void FsmTustin1_servo_servo(void);
void AskFsmLeadLag2Para_servo(int axis,int SN);
void AskFsmLeadLag1Para_servo(int axis,int SN);
void AskFsmNotchPara_servo(int axis,int SN);
void Ask2ParaShake_servo(int axis,int SN);
void Ask2ParaNotchfilter_servo(int axis,int SN);
#endif 
