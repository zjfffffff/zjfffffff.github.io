#include "AngelCalculation.h"
#include "math.h"
#include "CalculatesLocation.h"
#include "stm32f4xx.h"	
#define pi 3.1415926  
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float SpaceSetValueFY;
int8_t K_FY = 1;
int8_t K_HG = -1;
float kr = 1;
struct AxisFW_AxisFY AxisFW_AxisFY_T1;

void MatrixMutiply3_New(int m, int n, int p, float lMatrix1[3][3],float lMatrix2[3][3], float lMatrixResult[3][3])
{
    int i, j, k;
    float lSum = 0;

    /*嵌套循环计算结果矩阵（m*p）的每个元素*/
    for(i = 0; i < m; i++)
        for(j = 0; j < p; j++)
        {
            /*按照矩阵乘法的规则计算结果矩阵的i*j元素*/
            lSum = 0;
            for(k = 0; k < n; k++)
                lSum += lMatrix1[i][k] * lMatrix2[k][j];
            lMatrixResult[i][j] = lSum;
        }
}

AxisFW_AxisFY AngelCalculation(float Azimuth, float Roll, float Pitch, float PointToFW, float PointToFY, float NewAngle)
{
	float theta = 0.0;
	float HG_Out = 0.0;
	float FY_Out = 0.0;
	float angle1 = 0.0;
	float angle2 = 0.0;
	float PH1 = 0.0;
	float FY1 = 0.0;
	float q1[3][3] = {0.0};
	float q3[3][3] = {0.0};
	float q4[3][3] = {0.0};
	float q5[3][3] = {0.0};
	float q_result[3][3] = {0.0};
	float x1 = 0.0;
	float y1 = 0.0;
	float z1 = 0.0;
	float Angle = 0.0;
	float Alpha = 0.0;
	float FW_Out = 0.0;
	Angle = 90.0f + NewAngle;
	AxisFW_AxisFY AxisFW_AxisFY_Temp;
	theta = PointToFW - Azimuth;
	theta = theta * 3.1415926f / 180.0f;
	Alpha = SpaceSetValueFY * kr * pi / 180;
	HG_Out = cosf(theta) * Roll - K_FY * sinf(theta) * Pitch;
	FY_Out = cosf(theta) * Pitch + sinf(theta) * Roll;
	FW_Out = Azimuth * cosf(Alpha) + K_HG * HG_Out * sin(Alpha);
	
	PH1 = PointToFW - FW_Out;
//	PH1 = PointToFW - Azimuth;
	FY1 = PointToFY - FY_Out;
	
	
	angle1 = - PH1 * 3.1415926f / 180.0f;
	angle2 = - FY1 * 3.1415926f / 180.0f;
	
	q1[0][0] = cosf(angle2) * sinf(angle1);
	q1[1][0] = cosf(angle2) * cosf(angle1);
	q1[2][0] = sinf(angle2);
	q3[0][0] = cosf(pi / 2.0f);
	q3[0][1] = sinf(pi / 2.0f);
	q3[0][2] = 0;
	q3[1][0] = -sinf(pi / 2.0f);
	q3[1][1] = cosf(pi / 2.0f);
	q3[1][2] = 0;
	q3[2][0] = 0;
	q3[2][1] = 0;
	q3[2][2] = 1;
	
	q4[0][0] = cosf(-Angle * pi / 180.0f);
	q4[0][1] = 0;
	q4[0][2] = -sinf(-Angle * pi / 180.0f);
	q4[1][0] = 0;
	q4[1][1] = 1;
	q4[1][2] = 0;
	q4[2][0] = sinf(-Angle * pi / 180.0f);
	q4[2][1] = 0;
	q4[2][2] = cosf(-Angle * pi / 180.0f);
	
	MatrixMutiply3_New(3, 3, 3, q3, q1, q_result);
	MatrixMutiply3_New(3, 3, 3, q4, q_result, q5);
	
	x1 = q5[0][0];
	y1 = q5[1][0];
	z1 = q5[2][0];
	AxisFW_AxisFY_Temp.FW = atan(y1 / z1) * 180.0f / pi;
	AxisFW_AxisFY_Temp.FY = (atan(x1 / sqrt(y1 * y1 + z1 * z1)) * 180.0f / pi) / 2.0f + NewAngle / 2.0f;
	FW_buchang_degree = AxisFW_AxisFY_Temp.FW;
	FY_buchang_degree = AxisFW_AxisFY_Temp.FY;
	return AxisFW_AxisFY_Temp;
	
}
