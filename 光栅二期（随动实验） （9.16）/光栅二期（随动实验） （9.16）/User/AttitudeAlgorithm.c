/*文件包含伺服输出函数，
输入全局变量：CarrierFW、CarrierFY、CarrierHG，        载体方位 载体俯仰 载体横滚 单位：度
              SpaceSetValueFW、SpaceSetValueFY，       空间设定方位角 空间设定俯仰角 单位：度

输出全局变量：FW_location_set、FY_location_set         电机走的角度  单位：度
2020.03.02*/
#include "AttitudeAlgorithm.h"
#include "stm32f4xx.h"	
#include "math.h"

#define pi 3.1415926        
float r = 0;
float p = 0;
float y = 0;
float angle_fy = 0;
float angle_fw = 0;
float angle_qj = 0;
extern float NewAngle;
float x1 = 0;
float y1 = 0;
float z1 = 0;
float FW_buchang_degree = 0;
float FY_buchang_degree = 0;
float FW_location_actual_set = 0;    //方位设定值

float CarrierFW = 0;
float CarrierFY = 0;
float CarrierHG = 0;
	
float SpaceSetValueFW = 0;
float SpaceSetValueFY = 0;

extern unsigned int diaoyong_cnt;
extern float FW_zero_degree_zheng;    
extern float FY_location_set ;             //编码器闭环设定值
extern float FW_location_set;    //方位设定值
extern float FY_zero_degree;


void SpaceToAxiseAttitude(void)
{
	//	//赵老师公式起   step_design_arry_prepass[diaoyong_cnt]
//	//陀螺的FW_relative_rad = (Z_slave_1ms_distance_sum + step_design_arry_prepass[diaoyong_cnt]) * 3.1415926 / 180.0;  
//	FW_relative_rad = (Z_slave_1ms_distance_sum + step_design_arry_prepass[diaoyong_cnt]) * 3.1415926 / 180.0;
//	FY_relative_rad = 0 * 3.1415926 / 180.0;
//		
//	matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad) + 0.965925*sin(FY_relative_rad);
//	matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
//	matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
//		
//	FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 ;   //轴1.2189 --> 空1.1774
//	FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926 + 15.0) * 0.5;
//	//赵老师公式终
	
//	GPIO_SetBits(GPIOE,GPIO_Pin_2);
		
	
	float RealAngle = 90.0 + NewAngle;
	r = CarrierHG * pi /180.0;
	p = CarrierFY * pi /180.0;
//	y = (CarrierFW + step_design_arry_prepass[diaoyong_cnt]) * pi /180.0;
	y = CarrierFW * pi /180.0;
	angle_fy = SpaceSetValueFY * pi /180.0;
	angle_fw = SpaceSetValueFW * pi /180.0;
	angle_qj = -RealAngle * pi /180.0;

	x1 = (-cosf(p)*sinf(y)*cosf(angle_qj)+(sinf(r)*cosf(y)+cosf(r)*sinf(p)*sinf(y))*-sinf(angle_qj))*cosf(-angle_fy)*sinf(-angle_fw) + (cosf(p)*cosf(y)*cosf(angle_qj)+(sinf(r)*sinf(y)-cosf(r)*sinf(p)*cosf(y))*-sinf(angle_qj))*cosf(-angle_fy)*cosf(-angle_fw) + (sin(p)*cos(angle_qj)+cos(p)*cos(r)*-sin(angle_qj))*sin(-angle_fy);
	y1 = -(cosf(r)*cosf(y)-sinf(r)*sinf(p)*sinf(y))*cosf(-angle_fy)*sinf(-angle_fw) + -(cosf(r)*sinf(y)+sinf(r)*sinf(p)*cosf(y))*cosf(-angle_fy)*cosf(-angle_fw) + sinf(r)*cosf(p)*sinf(-angle_fy);
	z1 = (-cosf(p)*sinf(y)*sinf(angle_qj)+(sinf(r)*cosf(y)+cosf(r)*sinf(p)*sinf(y))*cosf(angle_qj))*cosf(-angle_fy)*sinf(-angle_fw) + (cosf(p)*cosf(y)*sinf(angle_qj)+(sinf(r)*sinf(y)-cosf(r)*sinf(p)*cosf(y))*cosf(angle_qj))*cosf(-angle_fy)*cosf(-angle_fw) + (sinf(p)*sinf(angle_qj)+cos(p)*cos(r)*cos(angle_qj))*sin(-angle_fy);

	FW_buchang_degree = atan(y1/z1)*180.0/pi;
	FY_buchang_degree = (atan(x1/sqrt(y1*y1+z1*z1))*180/pi+NewAngle)/2.0;
	//	GPIO_ResetBits(GPIOE,GPIO_Pin_2);//GPIOF9,F10设置高，灯灭

	//FW_location_set = FW_zero_degree_zheng - Z_slave_1ms_distance_sum;	
	//FW_location_set = FW_zero_degree_zheng - FW_buchang_degree;	
	//FY_location_set = FY_zero_degree - FY_buchang_degree;
	//	FW_buchang_degree = step_design_set; //FW_buchang_degree内含阶跃				
//	FW_location_set = FW_zero_degree_zheng - FW_buchang_degree; //FW_buchang_degree内含阶跃
//	//FW_location_set = 201 + step_design_arry_prepass[diaoyong_cnt] + step_sin_value;  //step_sin_value
//	FW_location_actual_set = 201 + step_design_arry[diaoyong_cnt] - FW_buchang_degree; 
//	FY_location_set = FY_zero_degree - FY_buchang_degree;
	//FY_location_set = FY_zero_degree - zhengfu * FY_buchang_degree + X_slave_1ms_distance_sum;

}
	