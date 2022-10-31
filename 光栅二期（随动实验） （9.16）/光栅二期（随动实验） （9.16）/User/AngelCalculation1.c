#include "AngelCalculation1.h"
#include "math.h"
#include "CalculatesLocation.h"
#include "stm32f4xx.h"	
#define pi 3.1415926  
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float SpaceSetValueFY;
extern int8_t K_FY;
extern int8_t K_HG;
extern float kr;
extern unsigned int diaoyong_cnt;
float x2 = 0;
float y2 = 0;
float z2 = 0;

	float angle1 = 0;
	float angle2 = 0;

void MatrixMutiply3_New1(int m, int n, int p, float lMatrix1[3][3],float lMatrix2[3][3], float lMatrixResult[3][3])
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

void AngelCalculation1(float Azimuth, float Roll, float Pitch, float PointToFW, float PointToFY, float NewAngle)
{
	float q1[3][3] = {0.0};
	float q2[3][3] = {0.0};
	
	float q2_0[3][3] = {0.0};
	float q2_1[3][3] = {0.0};
	float q2_2[3][3] = {0.0};
	
	float q3[3][3] = {0.0};
	float q4[3][3] = {0.0};
	float q5[3][3] = {0.0};
	float q_result[3][3] = {0.0};
	float q_result1[3][3] = {0.0};
	float x1 = 0.0;
	float y1 = 0.0;
	float z1 = 0.0;
	float Angle = 0.0;
	float r = 0;
	float p = 0;
	float y = 0;


	
	
	
	
//	Angle = (90.0f + NewAngle)*(-1);
	r = Roll * 3.1415926f / 180.0f;
	p = Pitch * 3.1415926f / 180.0f;
	p = p * (-1);
	y = Azimuth * 3.1415926f / 180.0f;
	angle2 = PointToFY * 3.1415926f / 180.0f;
	angle1 = PointToFW  * 3.1415926f / 180.0f;
	
	q1[0][0] = cosf(angle2) * sinf(angle1);
	q1[1][0] = cosf(angle2) * cosf(angle1);
	q1[2][0] = sinf(angle2);
	
	
	q2_0[0][0] = cosf(r);
	q2_0[0][1] = 0;
	q2_0[0][2] = sinf(r);
	q2_0[1][0] = 0;
	q2_0[1][1] = 1;
	q2_0[1][2] = 0;
	q2_0[2][0] = -sinf(r);
	q2_0[2][1] = 0;
	q2_0[2][2] = cosf(r);
	
	q2_1[0][0] = 1;
	q2_1[0][1] = 0;
	q2_1[0][2] = 0;
	q2_1[1][0] = 0;
	q2_1[1][1] = cosf(p);
	q2_1[1][2] = -sinf(p);
	q2_1[2][0] = 0;
	q2_1[2][1] = sinf(p);
	q2_1[2][2] = cosf(p);
	
	q2_2[0][0] = cosf(y);
	q2_2[0][1] = -sinf(y);
	q2_2[0][2] = 0;
	q2_2[1][0] = sinf(y);
	q2_2[1][1] = cosf(y);
	q2_2[1][2] = 0;
	q2_2[2][0] = 0;
	q2_2[2][1] = 0;
	q2_2[2][2] = 1;
	
	MatrixMutiply3_New1(3, 3, 3, q2_0, q2_1, q_result);
	MatrixMutiply3_New1(3, 3, 3, q_result, q2_2, q2);
	
	q3[0][0] = 0;
	q3[0][1] = 1;
	q3[0][2] = 0;
	q3[1][0] = -1;
	q3[1][1] = 0;
	q3[1][2] = 0;
	q3[2][0] = 0;
	q3[2][1] = 0;
	q3[2][2] = 1;
	
	q4[0][0] = cosf(-15 * pi / 180.0f);
	q4[0][1] = 0;
	q4[0][2] = sinf(-15 * pi / 180.0f);
	q4[1][0] = 0;
	q4[1][1] = 1;
	q4[1][2] = 0;
	q4[2][0] = -sinf(-15 * pi / 180.0f);
	q4[2][1] = 0;
	q4[2][2] = cosf(-15 * pi / 180.0f);
	
	MatrixMutiply3_New1(3, 3, 3, q4, q3, q_result);
	MatrixMutiply3_New1(3, 3, 3, q_result, q2, q_result1);
	MatrixMutiply3_New1(3, 3, 3, q_result1, q1, q5);
	
	x2 = q5[0][0];
	y2 = q5[1][0];
	z2 = q5[2][0];
	FW_buchang_degree =  atanf(y2 / x2) * 180.0f / pi;
	FY_buchang_degree =  atanf(z2 / sqrt((x2)*(x2)+(y2)*(y2)))*180.0f / pi / 2.0f - 7.5;
//	FY_buchang_degree = (atanf(x2 / sqrt(y2 * y2 + z2 * z2)) * 180.0f / pi) / 2.0f + NewAngle / 2.0f - 15;
//	FY_buchang_degree = (-1) * FY_buchang_degree;
}
