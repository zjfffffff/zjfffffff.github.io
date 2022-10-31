#include "stm32f4xx.h"	
#include <math.h>
#include <CalculatesLocation.h>
#define M_PI  (3.14159265358979323846)

float Angle_ly = 0;
extern float yaw_attitude_float;
extern float pitch_attitude_float;
extern float roll_attitude_float;
int K1_ly = 1;
int K2_ly = 1;
float Space_FY = 0;
float Space_FW = 0;
//float x1_z, y1_z, z1_z;
float x1_1_z, y1_1_z, z1_1_z;
void MatrixMutiply3(int m, int n, int p, float lMatrix1[3][3],float lMatrix2[3][3], float lMatrixResult[3][3])
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
float ppp, yyy;
float q5[3][3];	
/////////////////////azimuth = yaw_attitude_float, pitch = pitch_attitude_float, roll = roll_attitude_float, AxisPh = FW_encoder_degrees，AxisFy = FY_encoder_degrees
CalculatesData CalculatesLocation(float azimuth, float pitch, float roll, float AxisPh, float AxisFy, float NewAngle)
{
    CalculatesData cd;
    float y, p, r;
		float RealAngle = 90.0 + NewAngle;
//	azimuth = 0;
//	pitch = 0;
//	roll = 0;
	
    y = azimuth * M_PI / (float)180;
    p = pitch * M_PI / (float)180;
    r = roll * M_PI / (float)180;
		p = - p;
    float angle1= AxisPh * M_PI / 180;
    float angle3= AxisFy;
    float angle2= (2 * angle3 + 15) * M_PI / 180;
    
//	  float x1, y1, z1;
		float x1_1, y1_1, z1_1;
	
    float q1[3][3] = 
		{
				{cosf(angle2) * cosf(angle1),0,0}, 
				{cosf(angle2) * sinf(angle1),0,0}, 
				{sinf(angle2),0,0}
		};
    float q2[3][3] =
    {
        {cosf((float)15 * M_PI / (float)180), 0, sinf((float)(15) * M_PI /(float) 180)},
        {0, 1, 0},
        {-sinf((float)(15) * M_PI / (float)180), 0, cosf((float)15 * M_PI / (float)180)}
    };
    float q3[3][3] =
    {
        {0, -1, 0},
        {1, 0, 0},
        {0, 0, 1}
    };
		float q4[3][3] =
		{
				{cosf(y)*cosf(r)+sinf(y)*sinf(p)*sinf(r),sinf(y)*cosf(p),-cosf(y)*sinf(r)+sinf(y)*sinf(p)*cosf(r)},
				{-sinf(y)*cosf(r)+cosf(y)*sinf(p)*sinf(r),cosf(y)*cosf(p),sinf(y)*sinf(r)+cosf(y)*sinf(p)*cosf(r)},	
				{cosf(p)*sinf(r),- sinf(p),cosf(p)*cosf(r)}		
		};

//    float tmp1[3][3] =
//    {
//        {cosf(y), -sinf(y), 0},
//        {sinf(y), cosf(y), 0},
//        {0, 0, 1}
//    };
//    float tmp2[3][3] =
//    {
//        {1, 0, 0},
//        {0, cosf(p), -sinf(p)},
//        {0, sinf(p), cosf(p)}
//    };
//    float tmp3[3][3] =
//    {
//        {cosf(r),0, sinf(r)},
//        {0, 1, 0},
//        {-sinf(r), 0, cosf(r)}
//    };
    float tmp[3][3];
    float tmp_t[3][3];
		float tmp_t_t[3][3];
//    MatrixMutiply3(3, 3, 3, tmp1, tmp2, tmp);
//    MatrixMutiply3(3, 3, 3, tmp, tmp3, q4);
    MatrixMutiply3(3, 3, 3, q4, q3, tmp);
		MatrixMutiply3(3, 3, 3, tmp, q2, tmp_t);
		
//    q5[0][0] = tmp_t[0][0] * q1[0][0] + tmp_t[0][1] * q1[1][0] + tmp_t[0][2] * q1[2][0];
//    q5[1][0] = tmp_t[1][0] * q1[0][0] + tmp_t[1][1] * q1[1][0] + tmp_t[1][2] * q1[2][0];
//    q5[2][0] = tmp_t[2][0] * q1[0][0] + tmp_t[2][1] * q1[1][0] + tmp_t[2][2] * q1[2][0];

//    x1 = q5[0][0];
//    y1 = q5[1][0];
//    z1 = q5[2][0];
		
		MatrixMutiply3(3, 3, 3, tmp_t, q1, tmp_t_t);

    x1_1 = tmp_t_t[0][0];
    y1_1 = tmp_t_t[1][0];
    z1_1 = tmp_t_t[2][0];
   
	 
	  x1_1_z = x1_1;
    y1_1_z = y1_1;
    z1_1_z = z1_1;
//	  x1_z = x1;
//    y1_z = y1;
//    z1_z = z1;
	 
	 	ppp = atanf(x1_1_z / y1_1_z) * (float)180 / M_PI;
    yyy = atanf(z1_1_z / sqrtf(y1_1_z * y1_1_z + x1_1_z * x1_1_z)) * (float)180 / M_PI;
	 

//	  ppp = atanf(x1 / y1) * (float)180 / M_PI;
//    yyy = atanf(z1 / sqrtf(y1 * y1 + x1 * x1)) * (float)180 / M_PI;
    
		Space_FW = ppp;
		Space_FY = yyy;
		
//		ppp = -ppp;
//    cd.x = ppp;
//    cd.y = yyy;
//    cd.z = 0;
//	
//	Angle_ly = ppp; //PPP 方位   yyy 俯仰
//	
//	Space_FY = yyy + cosf(Angle_ly * 3.1415 / 180) * pitch_attitude_float + K1_ly * sinf(Angle_ly * 3.1415 / 180) * roll_attitude_float ;
////	Space_FW = ppp +  K2_ly * yaw_attitude_float;
//	Space_FW = yaw_attitude_float * cosf(Space_FY * 3.1415 / 180) - roll_attitude_float * sinf(Space_FY * 3.1415 / 180) + ppp;
//	
	
	
	
    return cd;
	
	
	
	
	
	
	
}