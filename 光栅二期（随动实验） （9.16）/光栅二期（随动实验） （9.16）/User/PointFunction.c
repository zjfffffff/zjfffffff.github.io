/*�ļ������ŷ����������
����ȫ�ֱ�����CarrierFW��CarrierFY��CarrierHG��        ���巽λ ���帩�� ������ ��λ����
              SpaceSetValueFW��SpaceSetValueFY��       �ռ��趨��λ�� �ռ��趨������ ��λ����

���ȫ�ֱ�����FW_location_set��FY_location_set         ����ߵĽǶ�  ��λ����
2020.03.02*/
#include "PointFunction.h"
#include "stm32f4xx.h"	
#include "AttitudeAlgorithm.h"
#include "FsmCompensator.h"
#include "math.h"
#include "timer.h"
#include "ioinit.h"
#include "AngelCalculation.h"
#include "AngelCalculation1.h"
#include "Deg_InverseSolution.h"
#include "DyConvertCoordinate1.h"

void point_to(void);

float v_zhixiang_num=60;
float FW_pianzhi = -14.14;		//ָ��λƫ�ã�2022.5.7��դ��56.2
float FY_pianzhi = -14.14;		//ָ56.2����ƫ�ã�2022.5.7��դ��56.2
extern float FY_Target_Position;

float Point_FW_set_prepass = 0;
float Point_FY_set_prepass = 0;
float FW_pointing_degree_LY = 0;
float FY_pointing_degree_LY = 0;

float FW_pointing_degree_set = 0;    //ָ����趨ֵ
float FW_pointing_degree_out = 0;    //ָ���+���ֵ
float FY_pointing_degree_set = 0;    //ָ����趨ֵ
float FY_pointing_degree_out = 0;    //ָ���+���ֵ
float FY_location_set_last = 0;
float FY_target_speed = 0;
float FW_target_speed = 0;
float FW_location_set_last = 0;
u8 ZhiXiangFlag=0;
u8 acc_vel_flag=0;

extern float FW_location_set;    //��λ�趨ֵ
extern float FY_location_set;    //�������ջ��趨ֵ
extern u8 FY_location_loop_open_flag;
extern u8 FW_location_loop_open_flag;
extern u8 FW_light_loop_open_flag;
extern u8 FY_light_loop_open_flag;
extern u8 FY_location_para;
extern u8 FW_location_para;
extern u8 FY_location_para_set;
extern u8 FW_location_para_set;
extern float velocity_limit_value_set;
extern float FW_zero_degree_zheng;     //0.02 ��- 201.2021
extern float FY_zero_degree;         //0.06 ��- 323.02
extern float FW_zero_degree_fu;
extern u8 FW_zero_flag;
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float X_slave_1ms_distance_sum1_2;
extern float X_slave_1ms_distance_sum;
extern float FW_step_chafen_speed;
extern float FY_step_chafen_speed;
extern float global_FW_step_piror;
extern float global_FY_step_piror;
extern float FY_wending_qiankui_speed_lowpass;   //�ȶ�ǰ���ٶ��˲�
extern float FW_wending_qiankui_speed_lowpass;
extern float Z_slave_1ms_distance_sum;
extern float Z_slave_1ms_distance;
extern float X_slave_1ms_distance;
extern u8 TuoLuo_Flag;
extern float CarrierFW;
extern float CarrierFY;
extern float CarrierHG;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
extern float pitch_attitude_float;
extern float yaw_attitude_float;
extern float x_axis_velocity_float;
extern float z_axis_velocity_float;
extern float step_sin_value;
extern u32 step_sin_cnt; 
extern float X_slave_1ms_distance_PT;
extern float accelerate_limit;

float FW_KongJian_offset = 0.019;
float FY_KongJian_offset = -0.001;
float KongJian_Set = -29.777;
float GuiLing_Set = 0;
float FY_KongJian_Set = -0.05;     //0.345
float FY_GuiLing_Set = 0;

u16 KongJian_CNT = 0;
u8 KongJianZhiXiang_flag = 0;
u8 GuiLing_flag = 0;
u16 GuiLin_CNT = 0;
u8 KongJianPoin_flag = 0;

float KongJianPoint_FW = 0;
u8 KongJianPoint_Complete_Flag = 0;
float KongJianPoint_FY = 0;
float KongJianPoint_FW_Zero = 0;
float KongJianPoint_FY_Zero = 0;
unsigned int Eqution_Flag = 3;
extern float roll_attitude_float;
//extern float accelerate_limit;
//4ָ��	
extern unsigned int diaoyong_cnt;
extern float step_design_arry_prepass[300];
extern float NewAngle;

float FW_New_BM = 0;
float FY_New_BM = 0;
extern float FW_encoder_degrees;  
extern float FY_encoder_degrees;

TwoAngle axisAngle;
TwoAngle spaceAngle;

extern INAngle inAngle;

extern u8 system_mode; 
extern float FW_tuoluo_para ;
extern float FY_tuoluo_para ;

void SpacePoint(void)
{
	if(system_mode == 17)
	{
		FY_location_loop_open_flag = 0;
		FW_location_loop_open_flag = 0;
		FW_light_loop_open_flag = 0;
		FY_light_loop_open_flag = 0;
		FW_tuoluo_para = 4;
		FW_tuoluo_para =4;
	}
	else
	{
		FY_location_loop_open_flag = 1;
		FW_location_loop_open_flag = 1;
		FW_light_loop_open_flag = 0;
		FY_light_loop_open_flag = 0;
		FW_location_para = 4;
		FY_location_para =4;
	}
		velocity_limit_value_set = v_zhixiang_num;   //60
		accelerate_limit = 2000;
	
		FW_pointing_degree_LY=*Deg_2_1;//ָ��ʽ��ֵ��2��5.7��
		FW_pointing_degree_set = FW_pointing_degree_LY;			//��դ2���������
		Point_FY_set_prepass=*Deg_1_1;//ָ��ʽ��ֵ��1��5.7��
		FY_pointing_degree_set = Point_FY_set_prepass;			//��դ1��������Դ
	
		point_to();   //������ָ��
}
	

void point_to(void)
{
	//FW_pointing_degree_out�Ǿ��ԽǶ�
	FY_pointing_degree_out = FY_zero_degree + FY_pianzhi - FY_pointing_degree_set;            //�����¸�
	FW_pointing_degree_out = FW_zero_degree_fu + FW_pianzhi - FW_pointing_degree_set;

	FW_location_set = FW_pointing_degree_out;
	FY_location_set = FY_pointing_degree_out;
}