#include "stm32f4xx.h"
#include "GuanDaoZhuanHuan.h"
#include "math.h"
#include <CalculatesLocation.h>

#define pi 3.1415926  


float y3 = 0;
float p3 = 0;
float r3 = 0;
extern float yaw_attitude_float;
extern float pitch_attitude_float;
extern float roll_attitude_float;

extern float yaw_attitude_float_ly;
extern float pitch_attitude_float_ly;
extern float roll_attitude_float_ly;

float y4 = 0;
float p4 = 0;
float r4 = 0;
float r_change_later = 0;
float y_change_later = 0;
float p_change_later = 0;

float r_change_before = 0;
float y_change_before = 0;
float p_change_before = 0;

float FW_PianCha = 0.75; 
float FY_PianCha = -0.095;
float HG_PianCha = -0.07;

float r4_test = 5;              //横滚
float p4_test = 0;             //俯仰
float y4_test = 0;              //方位


void ly_MatrixMutiply3(int m, int n, int p, float lMatrix1[3][3],float lMatrix2[3][3], float lMatrixResult[3][3])
{
    int i, j, k;
    float lSum = 0;

    
    for(i = 0; i < m; i++)
        for(j = 0; j < p; j++)
        {
            
            lSum = 0;
            for(k = 0; k < n; k++)
                lSum += lMatrix1[i][k] * lMatrix2[k][j];
            lMatrixResult[i][j] = lSum;
        }
}



void GuanDaoChang(void)
{
	y3 = FW_PianCha * pi / 180;
	p3 = FY_PianCha * pi / 180;
	r3 = HG_PianCha * pi / 180;
	
	r4 = roll_attitude_float_ly * pi /180.0;             //方位
	p4 = pitch_attitude_float_ly * pi /180.0;           //俯仰
	y4 = yaw_attitude_float_ly * pi /180.0;            //横滚
	
//	r4 = r4_test * pi /180.0;             //方位
//	p4 = p4_test * pi /180.0;           //俯仰
//	y4 = y4_test * pi /180.0;            //横滚
	
	
    float q1_1[3][3] =
    {
        {cosf(r4), 0, -sinf(r4)},
        {0, 1, 0},
        {sinf(r4), 0, cosf(r4)}
    };
	float q1_2[3][3] =
    {
        {1, 0, 0},
        {0, cosf(p4), sinf(p4)},
        {0, -sinf(p4), cosf(p4)}
    };
	float q1_3[3][3] =
    {
        {cosf(y4), sinf(y4), 0},
        {-sinf(y4), cosf(y4), 0},
        {0, 0, 1}
    };
	
	float q2[3][3] =
    {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    };
	
	float q3_1[3][3] =
    {
        {cosf(y3), sinf(y3), 0},
        {-sinf(y3), cosf(y3), 0},
        {0, 0, 1}
    };
	float q3_2[3][3] =
    {
        {1, 0, 0},
        {0, cosf(p3), sinf(p3)},
        {0, -sinf(p3), cosf(p3)}
    };
	float q3_3[3][3] =
    {
        {cosf(r3), 0, -sinf(r3)},
        {0, 1, 0},
        {sinf(r3), 0, cosf(r3)}
    };
	
	float q1_result[3][3];
	float q1[3][3];
	float q3_result[3][3];
	float q3[3][3];
	float q4_result[3][3];
	float q4[3][3];
	ly_MatrixMutiply3(3, 3, 3, q1_1, q1_2, q1_result);
	ly_MatrixMutiply3(3, 3, 3, q1_result, q1_3, q1);	
	ly_MatrixMutiply3(3, 3, 3, q3_1, q3_2, q3_result);
	ly_MatrixMutiply3(3, 3, 3, q3_result, q3_3, q3);
	ly_MatrixMutiply3(3, 3, 3, q3, q2, q4_result);
	ly_MatrixMutiply3(3, 3, 3, q4_result, q1, q4);
	
	y_change_before = atan(q4[0][1]/q4[1][1]);
	p_change_before = asin(q4[2][1]);
	r_change_before = -atan(q4[2][0]/q4[2][2]);
	
	r_change_later = r_change_before / pi * 180;        //横滚   单位度
	y_change_later = y_change_before / pi * 180;        //方位   单位度
	p_change_later = p_change_before / pi * 180;	       //俯仰   单位度
	
}

