#ifndef _Deg_InverseSolution_h
#define _Deg_InverseSolution_h


extern double *Deg_1_1,*Deg_2_1;
extern double FW_Degree_GS;
extern double FY_Degree_GS;
extern double jiao1,jiao2;
void Deg_CloseLoop(double x);//Ω«∂»ø€»¶≈–∂œ
void Deg_InverseSolution(double FW_Degree_GS,double FY_Degree_GS);
float SinBode(float amp, float frequency, float offest);


#endif