#include "timer.h"
#include "PositionLoop.h"
#include "MotorOutputPosse.h"
#include "stm32f4xx.h"	
#include "FSMC.h"	
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"
#include "can.h"
#include "elmo.h"
#include "ioinit.h"
#include "math.h"
#include "aaa.h"
#include "stmflash.h"
#include "AttitudeAlgorithm.h"
#include "PointFunction.h"
#include "ServoFunction.h"
#include "TestFunction.h"
#include "ScanFunction.h"
#include "TestScan.h"
#include "YunSuFunction.h"
#include "EXIT.h"
#include <CalculatesLocation.h>
#include "GuanDaoZhuanHuan.h"
#include "AngelCalculation.h"
#include "DyConvertCoordinate1.h"
#include "FY_485.h"
#include "AHRS_100A.h"
#include "Camera_Config.h"
#include "PC_Communication.h"
#include "Comm1_Communication.h"
#include "RMS2.h"
#include "SaoMiao.h"
#include "temperature.h"
#include "usart.h"	
#include "StepSingle.h"
#include "Receive_Send_Data.h"
#include "Deg_InverseSolution.h"
#include "WriteGuanDao.h"
float GS_1,GS_2;
u16 GuanDaoWR_JiShu = 0;
int GS_Deadscale_Flag;
float gs_ZHIXIANG1=1,gs_ZHIXIANG2=0.01;
float gs_x_k=-1;
int wMO_count = 0;
extern float FW_location_loop_error_sum ;
extern float FY_location_loop_error_sum ;
float GS_Trace_FW,GS_Trace_FY;
float GS_FW_DEG,GS_FY_DEG,GS_HG_DEG;
float GS_TRACE_FW_DEG,GS_TRACE_FY_DEG;
extern float Shangweiji_Pitch_Angle_Value_prepass;
extern float Shangweiji_Direction_Angle_prepass;
extern float GS_Suidong_offset;
float Speed_encode_FY = 0;
float Speed_encode_FW = 0;
float miss_distance_X_float_sum = 0;
float miss_distance_Y_float_sum = 0;
float kp_guangshan = 0.01;
float kp_guangshan_Y = 0.012; //0.019
extern u8 miss_distance_X_flag;
int SF_Jixia = 0;

extern float FY_location_kp;
extern float FY_location_ki;
extern float FY_location_kd;

extern float FW_location_ki;
extern float FW_location_kd;
float FW_location_JiTing_kp = 20;
float FW_location_JiTing_ki = 0;
float FW_location_JiTing_kd = 0.1;

float FY_location_JiTing_kp = 20;
float FY_location_JiTing_ki = 0.1;
float FY_location_JiTing_kd = 0.1;

extern unsigned int Eqution_Flag;

//////光纤陀螺的变量定义
float Tuoluo_bit_float_k_2 = 1;
float k_11 = 1;
float GOY_K2 = -0.05;
float GOY_K1 = -0.4;//0
u8 GOY_K = 3;
extern float Tuoluo_bit_float12;
extern float Tuoluo_bit_float11;
extern float Tuoluo_bit_float1;
extern float Tuoluo_bit_float_Y1;
extern float FY_Degree_GS_suidong,FW_Degree_GS_suidong;
float siqu_gs_flag=0;
float tuoluo_c = 0;
float tuoluo_wending_sudu_X1 = 0;
float tuoluo_wending_sudu_error_X = 0;
float tuoluo_wending_sudu_error_Y = 0;
float tuoluo_wending_sudu_X = 0;
float tuoluo_wending_sudu_Y = 0;
float tuoluo_wending_last_X = 0;
float tuoluo_wending_last_Y = 0;

float tuoluo_wending_ceshing_X = 0; //陀螺正弦测试
float tuoluo_wending_ceshing_Y = 0; //陀螺正弦测试
float tuoluo_A_X = 6.28;
float tuoluo_A_Y = 6.28;
float tuoluo_C = 0;
float tuoluo_hz_X = 0.2;
float tuoluo_hz_Y = 0.2;
float tuoluo_count = 0;
float tuoluo_count_Y = 0;

float FW_location_tuoluo_kp = 20;
float FW_location_tuoluo_ki = 0;
float FW_location_tuoluo_kd = 0.1;

float FY_location_tuoluo_kp = 115;
float FY_location_tuoluo_ki = 0;
float FY_location_tuoluo_kd = 0.1;

float FW_tuoluo_para = 0;
float FY_tuoluo_para = 0;
extern float FW_tuoluo_out;
extern float FY_tuoluo_out;
float FW_Tuoluo_first = 0;
float FY_Tuoluo_first = 0;


float FW_Tuoluo_set = 0;          //方位光纤陀螺角速度设定值为：0
float FY_Tuoluo_set = 0;          //俯仰光纤陀螺角速度设定值为：0

float FW_Tuoluo_last = 0;
float FY_Tuoluo_last = 0;

float FW_Tuoluo_bit_float = 0;		//方位光纤陀螺角速度
float FY_Tuoluo_bit_float = 0;		//俯仰光纤陀螺角速度

u8 FW_tuoluo_loop_open_flag = 0; 	//打开方位稳定环
u8 FY_tuoluo_loop_open_flag = 0;	//打开方位稳定环

extern float FY_location_kp;
extern float FW_location_kp;
extern float FY_location_ki;
extern float FW_location_ki;
				
#define SF_RX_touluo_Len_Y 10
#define SF_TX_touluo_Len_Y 10

u8 SF_TX_touluo_Data_Y[SF_TX_touluo_Len_Y],SF_RX_touluo_Data_Y[SF_RX_touluo_Len_Y];

float touluo_biaoduyingsu_Y = 154000;

u32 Tuoluo_bit_u32_Y = 0;
u32 Tuoluo_bit1_Y = 0;
u32 Tuoluo_bit2_Y = 0;
u32 Tuoluo_bit3_Y = 0;
u32 Tuoluo_bit4_Y = 0;
u32 Tuoluo_bit5_Y = 0;
int Tuoluo_bit_int_Y = 0;
extern float Tuoluo_bit_float_Y;							//光纤陀螺传回角速度

u16 Tuoluo_temperature_u16_Y = 0;
u16 Tuoluo_temperature1_Y = 0;
u16 Tuoluo_temperature2_Y = 0;
int Tuoluo_temperature_int_Y = 0;
float Tuoluo_temperature_float_Y = 0;			//光纤陀螺传回温度

#define SF_RX_touluo_Len 10
#define SF_TX_touluo_Len 10

float touluo_biaoduyingsu = 154000;				//陀螺标度因数

u32 Tuoluo_bit_u32 = 0;
u32 Tuoluo_bit1 = 0;
u32 Tuoluo_bit2 = 0;
u32 Tuoluo_bit3 = 0;
u32 Tuoluo_bit4 = 0;
u32 Tuoluo_bit5 = 0;
int Tuoluo_bit_int = 0;
extern float Tuoluo_bit_float;							//光纤陀螺传回角速度

u16 Tuoluo_temperature_u16 = 0;
u16 Tuoluo_temperature1 = 0;
u16 Tuoluo_temperature2 = 0;
int Tuoluo_temperature_int = 0;
float Tuoluo_temperature_float = 0;			//光纤陀螺传回温度

uint8_t SF_touluo_i = 0;
u8 SF_TX_touluo_Data[SF_TX_touluo_Len],SF_RX_touluo_Data[SF_RX_touluo_Len];
extern u16 SF_COMFalg;//读串口的标志位

///////////////////////

//u8 count_flag = 1;
u8 count_f = 1;
//u8 count_1 = 1;
//extern float FW_power_on_degrees;
float FY_Difference = 0;
int16_t k_count_Y = 0;
float FY_encoder_degrees_First = 0.0;
float FW_encoder_degrees_First = 0.0;
float FW_encoder_degrees_Second = 0.0;
float FW_Difference = 0.0;
int16_t k_count = 0;


u8 FW=1;
u8 FY=2;

struct
{
	unsigned int EncodeLow;
	unsigned int EncodeHigh;
	float Degrees;
	unsigned int Encode;
	unsigned int EncoderFlag;
	unsigned int Low2bits;
}EncoderData;

typedef union
{
	struct
	{
		unsigned int EncodeLow;
		unsigned int EncodeHigh;
		float Degrees;
		unsigned int Encode;
		unsigned int EncoderFlag;
		unsigned int Low2bits;
	}EncoderData_FW;
	struct
	{
		unsigned int EncodeLow;
		unsigned int EncodeHigh;
		float Degrees;
		unsigned int Encode;
		unsigned int EncoderFlag;
		unsigned int Low2bits;
	}EncoderData_FY;
}Encoder__Data;
Encoder__Data Encoder_Data; 

typedef union
{
	struct
	{
		float TI;
		float IQ;
		float EC;
		float MF;
		float SR;
		float EE_1;
		float AN_6;
		float AN_1;
		float AN_3;
		float AN_4;
		float AN_5;
		float PX;
	}Data_byte;
}Data_value;
extern Data_value Elmo_FW; 

typedef union
{
	struct
	{
		float TI;
		float IQ;
		float EC;
		float MF;
		float SR;
		float EE_1;
		float AN_6;
		float AN_1;
		float AN_3;
		float AN_4;
		float AN_5;
		float PX;
	}Data_byte;
}Data_value1;
extern Data_value1 Elmo_FY;


float temperature_liyanjun1=0;
float temperature_liyanjun2=0;


float du_read_Currente=0;
int du_read_temperature=0;
u8 biao_zhi=0;


//环形走位变量设置
#define n_LYJ 285
u16 counter1=0;
u16 counter2=0;
u16 counter_data=1;
u16 counter_j=0;	
float ab[n_LYJ][2]={{0,0}};
float FW_zero_degree1_fu=0 ;
float FY_zero_degree1_zheng=0;




//自己定义求RMS2微弧度
float RMS2_weihudu_X=0.0;
float RMS2_weihudu_Y=0.0;

float RMS2_bianmaqi_X=0.0;
float RMS2_bianmaqi_Y=0.0;

#define RETURN_ZERO_MODE 3  								 //回零模式
#define POINT_MODE 4         								//指向模式
#define Time_Di_16         			    ((u32)(0x6400E100))
#define Time_Gao_16         				((u32)(0x6400e200))
#define FPGADataUpdateFlag              ((u32)(0x6400F000))//数据更新状态使能

/*********************************************************************↓全局变量↓**************************************************************/
u8 JingGenZong_Err = 0;                           //与精跟踪板通信变量;
u8 JingGenZong_Flag = 0;                          //与精跟踪板通信变量;
u8 FW_light_para = 0;                             //方位光闭环校正选择模式;
u8 FY_light_para = 0;                             //俯仰光闭环校正选择模式;
u8 stabilize_type_select = 0;                     // = 1目标为惯导， = 2目标为数引;
u8 stabilize_axis_select = 0;                     // = 1 俯仰稳定，  = 2 方位稳定， = 3 共同稳定;
u8 FY_location_para = 0;
u8 FW_location_para = 0;
u8 FY_location_para_set = 1;
u8 FW_location_para_set = 1;
float FW_pre_lvbo = 20;    						  //阶跃前置滤波
float FY_pre_lvbo = 20;
float FW_lowpass = 80;     						  //1ms位置闭环——速度环低通滤波☆
float FY_lowpass = 80;

/*********************************************************************↓全局变量END↓**************************************************************/

/*********************************************************************↓外部调用变量↓**************************************************************/
extern int32_t MF_fy,EE1_fy,MF_fw,EE1_fw;
extern float NewAngle;

/*********************************************************************↓外部调用变量END↓**************************************************************/
float FW_wending_qiankui_lvbo = 2;   //位置闭环前馈滤波☆
float FY_wending_qiankui_lvbo = 2;

float FW_track_qiankui_lvbo = 2;     //光闭环跟踪前馈滤波★
float FY_track_qiankui_lvbo = 2;

float FW_wending_yijie_tao = 1;      //位置闭环的一阶校正 
float FW_wending_yijie_T = 0.2;	
float FY_wending_yijie_tao = 1;
float FY_wending_yijie_T = 0.2;	

float FW_wending_erjie_tao = 0.4;    //位置闭环的一阶校正 
float FW_wending_erjie_T = 0.2;
float FY_wending_erjie_tao = 0.4;
float FY_wending_erjie_T = 0.2;

float FW_lowpass_servo = 80;           //10ms光闭环——速度环低通滤波★
float FY_lowpass_servo = 80;

float FW_track_yijie_tao_servo = 1;    //10ms光闭环一阶★ 
float FW_track_yijie_T_servo = 0.2;
float FY_track_yijie_tao_servo = 1;
float FY_track_yijie_T_servo = 0.2;
	
float FW_track_erjie_tao_servo = 1;    //10ms光闭环二阶★ 
float FW_track_erjie_T_servo = 0.2;
float FY_track_erjie_tao_servo = 1;
float FY_track_erjie_T_servo = 0.2;


float accelerate_limit = 1000;
float velocity_limit_value_set = 40;


//校正参数
extern float X0_fsmT1;     //阶跃前置滤波
extern float Y0_fsmT1;

extern float X1_fsmT1;     //速度环低通滤波
extern float Y1_fsmT1;

extern float X2_fsmT1;     //稳定前馈滤波
extern float Y2_fsmT1;

extern float X4_fsmT1;     //跟踪前馈滤波
extern float Y4_fsmT1;

extern float X5_fsmTao1;   //稳定的一阶校正
extern float X5_fsmT1;
extern float Y5_fsmTao1;      
extern float Y5_fsmT1;     

extern float X3_fsmTao1;   //稳定的二阶校正
extern float X3_fsmTao2;
extern float X3_fsmT1;
extern float X3_fsmT2;
extern float Y3_fsmTao1;
extern float Y3_fsmTao2;
extern float Y3_fsmT1;
extern float Y3_fsmT2;

extern float X1_fsmT1_servo;     //10MS速度环低通滤波
extern float Y1_fsmT1_servo;

extern float X2_fsmTao1_servo;   //10MS光闭环一阶校正
extern float X2_fsmT1_servo;
extern float Y2_fsmTao1_servo;
extern float Y2_fsmT1_servo;

extern float X7_fsmTao1_servo;   //10MS光闭环二阶校正
extern float X7_fsmTao2_servo;
extern float X7_fsmT1_servo;
extern float X7_fsmT2_servo;
extern float Y7_fsmTao1_servo;
extern float Y7_fsmTao2_servo;
extern float Y7_fsmT1_servo;
extern float Y7_fsmT2_servo;
/*********************************************************************↑外部引用PositionLoop参数↑**************************************************************/
extern float FY_revise_out ;               //校正输出乘K
extern float FW_revise_out;               //方位校正值（速度）
extern float FW_location_loop_error;    //设定值与编码器值误差
extern float FY_location_loop_error;         //设定值 - 编码器值 
extern float FW_location_kp;
extern float FW_location_loop_actual_error;    //设定值与编码器值误差

/*********************************************************************↑外部引用MotorOutputPosse参数↑**************************************************************/
extern float FW_output_value;
extern float FY_output_value;
extern float accelerate_limit_1ms;
extern float velocity_limit_value_set;
extern float FY_limit_acc_value;
extern float FY_output_value_last;
extern float FY_output_value_da;
extern float FW_revise_out_speed;         //方位校正值加前馈速度
extern float FY_wending_qiankui_speed_lowpass;   //稳定前馈速度滤波
extern float FY_track_qiankui_speed_k;
extern float FY_track_qiankui_speed_lowpass;
extern float FW_wending_qiankui_speed_lowpass;  //方位前馈速度低通滤波
extern float FW_track_qiankui_speed_k;   //光闭环前馈系数Gain★
extern float FW_track_qiankui_speed_lowpass;
/*********************************************************************↑外部引用AttitudeAlgorithm参数↑**************************************************************/
extern float FW_buchang_degree;
extern float FY_buchang_degree;
extern float CarrierFW;
extern float CarrierFY;
extern float CarrierHG;
extern float SpaceSetValueFW;
extern float SpaceSetValueFY;
/*********************************************************************↑外部引用Function参数↑**************************************************************/
extern float FW_pointing_degree_set;    //指向角设定值
extern float FY_pointing_degree_set;    //指向角设定值
extern float FY_target_speed;

/*********************************************************************↑外部引用ServoFunction参数↑**************************************************************/
extern float FW_light_loop_k;       //理想2000
extern float miss_distance_X_float ;
extern float miss_distance_X_float_to_angle;
extern float miss_distance_Y_float;
extern float miss_distance_Y_float_to_angle;
extern u8 track_axis_select;      //跟踪轴选择，0：跟方位俯仰 1;跟方位 2：跟俯仰 
extern float FW_light_loop_k_2;     //默认二阶校正
extern float FW_light_loop_revise;                 //方位校正输出
extern float FY_light_loop_k;       //光闭环开环增益K★
extern float FY_light_loop_revise;                //俯仰校正输出
extern float FY_light_loop_k_2;      //默认二阶校正※
extern float FW_pi_K;
extern u8 FW_light_para_set;      //光闭环校正参数选择 1：K  2:一阶校正  3：二阶校正   4：PI校正
extern u8 FY_light_para_set;
/*********************************************************************↑设置参数区↑**************************************************************/
float FW_location_set = 397.6837;    //方位设定值
float FY_location_set = 217.1357;    //编码器闭环设定值

float FW_location_last = 0;
float FY_location_last = 0;

//俯仰编码器用
#define FY_encoder_Shangxianwei 360		//俯仰编码器上限位
#define FY_encoder_Xiaxianwei  -360		//俯仰编码器下限位

//读FSMC状态标志位
u16 TEST_ReadDataFlag=0;         //FPGA F000标志位
u16 TuoBa_fLAG=0;
u32 FPGA_READ_TEST_ADDR = 0;

//俯仰编码器用
u16 FY_encoder_l = 0;       
u16 FY_encoder_h = 0;
u32 FY_encoder_u32 = 0;        //俯仰绝对式编码器值
float FY_encoder_degrees = 0;  //俯仰编码器转换为角度
float FY_encoder_degrees_lowpass = 0;  //俯仰编码器转换为角度
u8 low_2bit_FY = 0;       

//


//方位编码器用
u16 FW_encoder_l = 0;
u16 FW_encoder_h = 0;
u32 FW_encoder_u32 = 0;        //俯仰绝对式编码器值
float FW_encoder_degrees = 0;  //俯仰编码器转换为角度
u8 low_2bit_FW = 0;  


//光纤惯导变量
u8 read_usart2[100];
u32 x_axis_velocity = 0;  
u32 x_axis_velocity_supplement = 0;
float x_axis_velocity_float = 0;
float x_axis_velocity_1msdistance = 0;  
float x_axis_velocity_1msdistance_sum = 0; 

u32 y_axis_velocity = 0;  
u32 y_axis_velocity_supplement = 0; 
float y_axis_velocity_float = 0;
float y_axis_velocity_1msdistance = 0;
float y_axis_velocity_1msdistance_sum = 0;

float x_axis_velocity_float_ly = 0;
float y_axis_velocity_float_ly = 0;

u32 z_axis_velocity = 0; 
u32 z_axis_velocity_supplement = 0;
float z_axis_velocity_float = 0;
float z_axis_velocity_1msdistance = 0;
float z_axis_velocity_1msdistance_sum = 0;

u32 x_axis_accelerated = 0;   
u32 x_axis_accelerated_supplement = 0;  
float x_axis_accelerated_float = 0; 

u32 y_axis_accelerated = 0;    
u32 y_axis_accelerated_supplement = 0;   
float y_axis_accelerated_float = 0;

u32 z_axis_accelerated = 0;   
u32 z_axis_accelerated_supplement = 0;  
float z_axis_accelerated_float = 0;

u32 roll_attitude = 0;
u32 roll_attitude_supplement = 0;
float roll_attitude_float = 0;
float roll_attitude_float_ly = 0;

u32 pitch_attitude = 0;
u32 pitch_attitude_supplement = 0;
float pitch_attitude_float = 0;
float pitch_attitude_float_ly = 0;

float roll_attitude_before = 0;
float pitch_attitude_before = 0;


u32 yaw_attitude = 0;
u32 yaw_attitude_supplement = 0;
float yaw_attitude_float = 0;
float yaw_attitude_float_ly = 0;

//MEMS惯导变量
u8 read_usart1[50];
int yaw_angle_slave = 0;           //从机偏航
float yaw_angle_float_slave = 0;

int pitch_angle_slave = 0;         //从机俯仰
float pitch_angle_float_slave = 0;

int roll_angle_slave = 0;          //从机横滚
float roll_angle_float_slave = 0;

int yaw_angle_velocity_slave = 0;       //从机偏航角速度
int yaw_angle_velocity_float_slave = 0;

int pitch_angle_velocity_slave = 0;     //从机俯仰角速度
int pitch_angle_velocity_float_slave = 0;

int roll_angle_velocity_slave = 0;      //从机横滚角速度
int roll_angle_velocity_float_slave = 0;

u32 X_axis_palstance = 0;
float X_axis_palstance_float = 0;
float X_axis_palstance_float_off_set = 0;
float X_axis_palstance_float_lowpass = 0;
float X_axis_palstance_float_lowpass_out = 0;

float low_pss_out_sanjie = 0;

u32 Y_axis_palstance = 0;
float Y_axis_palstance_float = 0;

u32 Z_axis_palstance = 0;
float Z_axis_palstance_float = 0;

float X_slave_1ms_distance = 0;
float X_slave_1ms_distance_PT = 0;
float X_slave_1ms_distance_SV = 0;
float X_slave_1ms_distance_sum = 0;

float Y_slave_1ms_distance = 0;
float Y_slave_1ms_distance_sum = 0;

float Z_slave_1ms_distance = 0;
float Z_slave_1ms_distance_sum = 0;

//M1陀螺
u8 read_usart3[10];
u8 read_usart4[200];
//M1惯导
u32 IMU_bit_u32 = 0;
u32 IMU_bit1 = 0;
u32 IMU_bit2 = 0;
u32 IMU_bit3 = 0;
u32 IMU_bit4 = 0;
u32 IMU_bit5 = 0;
int IMU_bit_int = 0;
float IMU_bit_float = 0;
float IMU_bit_float_lowpass = 0;

//LVDS
u8 lvds_array1[110];
u8 lvds_array2[110];

//温度
u16 temp1_u16 = 0;
u16 temp2_u16 = 0;
float temp1_f = 0;
float temp2_f = 0;

//脱靶量
u8 read_usart5[20];
int miss_distance_judge_X = 0;
int miss_distance_judge_Y = 0;
int miss_distance_X = 0;
int miss_distance_Y = 0;



extern float FW_Miss_distance;//方位脱靶量
extern float FY_Miss_distance;//俯仰脱靶量
extern float WD_FW,WD_FY;



//求帧间误差
float miss_distance_Y_float_last = 0;  //上一次脱靶量计数
float miss_distance_X_float_last = 0;  //上一次脱靶量计数

float FY_miss_distance_sub = 0;
float FW_miss_distance_sub = 0;
////////////////////////////////////////////////////////////////////////////////////

//系统模式
u8 system_mode = 0; 

//模式参数设置区
u8 FY_location_loop_open_flag = 0;
u8 FW_location_loop_open_flag = 0;
u8 FW_light_loop_open_flag = 0;			//方位光闭环跟踪标志位
u8 FY_light_loop_open_flag = 0;			//俯仰光闭环跟踪标志位

//3回零
u8 huiling_cnt = 0;
u8 huiling_once_flag = 1;
extern u8 FW_zero_flag;    //方位上电在哪个区域标志位

//5扫描
u32 timer_126ms_cnt = 0;           //126ms定时计数
u8 scan_parameter_set_flag = 1;   //扫描速度加速度参数设定标志位，只进一次
extern float FW_scan_degree;        
extern float FY_scan_degree;
extern int FW_scan_degree_out_int;
extern int FY_scan_degree_out_int;
extern u8 scan_cnt;               //扫描次数计数
extern u8 scan_cnt_derection;     //扫描角度个数计数
u8 yushu = 0;                     //1°阶跃时用
float FY_sin_value = 0;                                                                                                                                                                                                                                                                                                                            
float FW_relative_degree = 0;
float FY_off_set = 0;
float FY_sin_value_fabs = 0;

float FW_relative_rad = 0;
float FY_relative_rad = 0;

float matrix_a = 0;
float matrix_b = 0;
float matrix_c = 0;


//模拟阶跃
float FW_simulated_target_motion = 200;    //模拟阶跃
float FY_simulated_target_motion = 300;    //模拟阶跃
float FW_pre_lowpss_filter = 0;            //模拟阶跃前置滤波输出
float FY_pre_lowpss_filter = 0;            //模拟阶跃前置滤波输出
//float pre_lowpss_filter = 0;            //模拟阶跃前置滤波输出
u32 step_cnt = 0;                       //模拟阶跃计数
u32 FY_step_cnt = 0;                       //模拟阶跃计数
float FW_set_off = 3;                      //模拟阶跃角度设置
float FY_set_off = 3;                      //模拟阶跃角度设置

//6稳定
float X_slave_1ms_distance_sum_last = 0;
float X_slave_1ms_distance_sub = 0;

float Z_slave_1ms_distance_sum_last = 0;
float Z_slave_1ms_distance_sub = 0;

float X_speed_off_set = 0.025;         //MEMS陀螺俯仰稳定速度偏置，越大越往下
float Z_speed_off_set = 0.019;         //MEMS陀螺方位稳定速度偏置，越大越往左

u32 sin_cnt = 0;              //sin计数
float sin_value = 0;          //sin计算值
float sin_value_sum = 0;      //sin计算值
float sin_T_f = 5000;
u32 sin_T_u = 5000;

float FY_qiankui_speed = 0;                   //稳定差分前馈速度


float FW_revise_out_speed_acc = 0;     //限制加速度后的速度
float FW_revise_out_speed_last = 0;    //上一个周期的速度
float FW_revise_out_volatge = 0;       //速度校正转换为电压


float FW_qiankui_speed = 0;


float FY_light_loop_revise_speed = 0;                //俯仰校正输出


float FW_light_loop_revise_speed = 0;                 //方位校正输出

float FW_track_qiankui_speed = 0;
float FY_track_qiankui_speed = 0;

u32 step_jishi = 300;


//单输出口
u8 output_open_flag = 0;

int FW_output_value_can  = 0;
float FW_output_angle_pa_last  = 301.5;     //FW_zero_degree_zheng   
float FY_output_angle_pa_last  = 88.5;     //FY_zero_degree
float FW_output_angle_pa_now  = 0;
float FY_output_angle_pa_now  = 0;
float FW_output_value_can_sub  = 0;

float velocity_limit_value = 0;

float FW_limit_vel_value = 0;
float FY_limit_vel_value = 0;


//求差分速度
float global_FW_position_piror = 0;
float global_FY_position_piror = 0;
float FW_chafen_speed = 0;
float FY_chafen_speed = 0;

float FW_send_array[10];
float FY_send_array[10];
u8 FW_send_array_select = 0;
u8 FY_send_array_select = 0;

float shiyanzhengfu = 1;

float da_out = 0;


//自检标志位
int FY_encoder_Flag = 0;
int FW_encoder_Flag = 0;
int GuangxianGuandao_Flag = 0;
extern u32 Time;

float FW_zero_degree_fu = 313.11757;        //400.077698;
//float FW_zero_degree_fu = 268.65387;        //400.077698;
float FW_zero_degree_zheng = 313.11757;       //0.02 《- 201.2021
//float FW_zero_degree_zheng = 268.65387;       //0.02 《- 201.2021 
float FY_zero_degree = 213.45628;              //217.105453;
//float FY_zero_degree = 229.969833;              //217.105453;

uint16_t fpgaUpdateFlag = 0;
extern u8 AA_TXData[100];     //发送数据
extern float TX_Coarse_Azimuth;//粗跟踪伺服方位
extern float TX_Coarse_Pitch;//粗跟踪伺服俯仰
extern union FloatToArray Temp_Data;//
extern float TX_Ins_Attitude_FW;//惯导姿态方位
extern float TX_Ins_Attitude_FY;//惯导姿态俯仰
extern float TX_Ins_Attitude_Roll;//惯导姿态横滚
extern float TX_Ins_Speed_FW;//惯导角速度方位
extern float TX_Ins_Speed_FY;//惯导角速度俯仰
extern float TX_Ins_Speed_Roll;//惯导角速度横滚
extern float TX_Encoder_FW;//编码器方位
extern float TX_Encoder_FY;//编码器俯仰
extern float TX_Miss_FY;//脱靶量方位
extern float TX_Miss_FW;//脱靶量俯仰
extern float TX_Fast_Reflection_Mirror_FW;//快反镜方位
extern float TX_Fast_Reflection_Mirror_FY;//快反镜俯仰
extern int Read_JiaoYan_Flag;
extern uint8_t  Read_JiaoYanHouShuJu[46];//校验后存储数据
extern u8 CommandNew,CommandOld,CO_ID,ST_ID,Work_Mode;//命令 设备ID 工作模式
extern u8 TX_Servo_Steady_State;//伺服稳态
extern u8 AA_TX_Len,AA_RX_Len;//数据长度
extern float DianLiu_FW;
extern float DianLiu_FY;
extern s8 DianLiu_2;
extern s8 DianLiu_1;
extern u8 TX_Coarse_State;  ///粗跟踪状态字
extern u8 TX_Precise_State; ///精跟踪状态字
extern float TX_Ins_Yaw;//惯性空间偏航
extern float TX_Ins_RollAngle;//惯性空间横滚
extern float TX_Ins_PitchAngle;//惯性空间俯仰
extern u8 Fast_Mirror_Work_Mode;//快反镜工作模式
extern u8 Primary_Mirror_Work_Mode;//主反射镜工作模式
extern u8 Point_Mode; //指向模式
extern float FW_Huiling_last;
extern float FY_Huiling_last;

//步进模式变量
int FW_BuJin_Time_Count = 0;                            
int FW_BuJin_Flag = 0;                                //方位步进标志位
int FW_BuJin_LiangCheng_Degree = 0;                   //方位步进量程 无符号
int FW_BuJin_Time = 0;                                //方位步进 进入中断时间 整型
unsigned int FW_BuJin_SuDu = 0;                       //方位步进  扫描速度      无符号整型
unsigned int FW_BuJin_Count = 0;                      //方位步进  扫描次数      无符号整形
int FW_BuJin_degree_Start = 0;                        //方位步进  扫描起始角度  有符号整形
float FW_BuJin_degree_out = 0;                        //方位步进  输出  浮点型
unsigned int FW_BuJin_Direct = 0;                     //方位步进  扫描方向      无符号整形
int  FW_BuJin_limit = 0;                              //方位步进  扫描限位      有符号整形
int  FW_BuJin_degree_step = 0;                        //方位步进  扫描步长      有符号整形

int FY_BuJin_Flag = 0;                                //俯仰步进标志位
unsigned int FY_BuJin_Count = 0;                      //俯仰步进  扫描次数      无符号整形
int FY_BuJin_LiangCheng_Degree = 0;                   //俯仰步进量程 无符号
int FY_BuJin_Time = 0;                                //俯仰步进 进入中断时间 整型
unsigned int FY_BuJin_SuDu = 0;                       //俯仰步进  扫描速度      无符号整型
int FY_BuJin_degree_Start = 0;                        //俯仰步进  扫描起始角度  有符号整形
float FY_BuJin_degree_out = 0;                        //俯仰步进  输出  浮点型
unsigned int FY_BuJin_Direct = 0;                     //俯仰步进  扫描方向      无符号整形
int  FY_BuJin_limit = 0;                              //俯仰步进  扫描限位      有符号整形
int  FY_BuJin_degree_step = 0;                        //俯仰步进  扫描步长      有符号整形
extern int Send_Flag;
extern u8 Scan_FangShi;

//路径
u8 step_open_flag = 0;
unsigned int diaoyong_cnt = 0;

float step_design_set = 0;


float step_design_set_prepass = 0;

int cnt_Step = 0;
int Time_Long = 110;


int ScanAngleRang = 6 ;                        //扫描范围
float FY_ScanAngleRang = 2;                      //俯仰扫描范围
float FW_ScanStepLength = 2;                      //扫描步长
int Scan_StepCount = 0;                         
int	FW_Scan_StartAngle = 0;                    //方位扫描起始角度
int FY_Scan_StartAngle = 65;
int Scan_Direction = 0x01;                        //判断扫描方向
int FW_Scan_StopAngle = 0;                     //方位扫描结束角度
int FW_Direction = 0;           //右+1  左-1   //方位方向
int huanxiangzheng_flag = 0;
int FY_Direction = 0;          //上+1   下-1   //俯仰方向
int Flag_Com_Direction = 1;                    //方向只赋一次值
u8 Scan_Number = 0;
u8 Give_Scan_Number = 20;
float Cha = 0;
float FW_encoder_degrees_lowpass = 0;  //俯仰编码器转换为角度

extern float WD_FW;
extern float WD_FY;
int Send = 0;
//float FY_Scan_Zhixing = 0;

int jisuan_TIME = 0;
int Time_LY = 0;
float FW_yunsu_sudu = 0;
float FW_yunsu_sudu_1 = 0;
float FY_yunsu_sudu_1 = 0;
float Old_yunsu = 0;
int yunsu_cnt = 0;
float FY_yunsu_sudu = 0;
u8 CanShu_Read[38]={0};
extern u8	EXTI2_FLAG;	
u32 MaiChong_cnt=0;
int fasong_cnt = 65;
int once_flag = 0;
float FW_JueDui_StartAngle = 60;

int LY_Flag = 0;
int WaiTong_CNT = 0;

extern u32  EXTI2_CNT;
//int Send_MaiChong_CNT = 0;
float WuCha = 0;

float Watch_CaiJi[220]={0};
int watchi = 0;
int TX_watch = 10;
int Ma_watch = 1;	
u8 Ma_CaiJi[300]={0};

float Watch_MaiChong_Later[70]={0};
int watchi1 = 0;

u32 CaiJi_CNT=0;

int yunsu_zheng = 0;
int yunsu_fu = 0;

float FW_yunsu_StopAngle = 80;
float FW_yunsu_StartAngle = 70;
u8 TuoLuo_Flag = 1;

u32 camera_arry_cnt = 0;
float camera_arry[1000];
u8 miss_distance_flag = 0;
u8 miss_distance_cnt = 0;
u8 chakan1 = 0;
u8 chakan2 = 0;
float miss_distance_X_float_start1 = 0;
float miss_distance_X_float_start2 = 0;
float miss_distance_X_float_end1 = 0;
float miss_distance_X_float_sub1 = 0;
float miss_distance_X_float_end2 = 0;
float miss_distance_X_float_sub2 = 0;
float miss_distance_X_float_end3 = 0;
float miss_distance_X_float_sub3 = 0;
float miss_distance_X_float_end4 = 0;
float miss_distance_X_float_sub4 = 0;
float miss_distance_X_float_end5 = 0;
float miss_distance_X_float_sub5 = 0;
float miss_distance_X_float_end6 = 0;
float miss_distance_X_float_sub6 = 0;
float miss_distance_X_float_end7 = 0;
float miss_distance_X_float_sub7 = 0;
float miss_distance_X_float_end8 = 0;
float miss_distance_X_float_sub8 = 0;
float miss_distance_X_float_end9 = 0;
float miss_distance_X_float_sub9 = 0;
float miss_distance_X_float_end10 = 0;
float miss_distance_X_float_sub10 = 0;
float miss_distance_X_float_endsum1 = 0;
float miss_distance_X_float_ave = 0;
float miss_distance_X_float_endsum2 = 0;


float TEST_Change = 1234.8;

float step_sin_value = 0;
u32 step_sin_cnt = 0; 

extern float FY_Watch_shuyin_error;


float FW_step_chafen_speed = 0;
float FY_step_chafen_speed = 0;
float global_FW_step_piror = 0;
float global_FY_step_piror = 0;
extern float Watch_location_loop;

u8 TuoBa_Set_CNT = 0;
float FW_encoder_TuoBa_Set = 0;
float FY_encoder_TuoBa_Set = 0;

int PT_CNT = 0;
float PT_FY_miss_distance_sub = 0;
float PT_miss_distance_Y_float_last = 0;
				
float PT_FW_miss_distance_sub = 0;
float PT_miss_distance_X_float_last = 0;

float watch_slave_X[2000] = {0};
float watch_slave_Z[2000] = {0};
u16 slave_CNT = 0;
u16 slaveX_CNT = 0;
int z_int = 0;

u32 Scan_cnt_ly = 0;
u32 YunSu_cnt_ly = 0;

unsigned int JieYue_cnt = 0;
u8 Change_flag_ly = 0;

extern float step_design_set_ly;
extern float PanFangXiang;
extern int Flag_Com_Direction_ly;                    //方向只赋一次值
extern u8 Start_cnt;

extern float YunSu_step_design_set_ly;
extern u8 YunSu_Start_cnt;
extern float YunSu_PanFangXiang;
extern int Flag_YunSu_Direction_ly;                    //方向只赋一次值
extern u8 Test_Scan_Time;
float yaw_offset_val = -0.346;    //-4.009
float pitch_offset_val = 0.0;  //0.405
float roll_offset_val = 0.0;   //0.109

u8 ZhenJiShu_ly[1100] = {0};
u8 zjs[1000] = {0};
u16 ZJS_CNT = 0;
u16 ly_zjs = 0;
u16 CeShi_CNT = 0;

float miss_distance_X_float_LowpassBefore = 0;
float miss_distance_Y_float_LowpassBefore = 0;
float FW_miss_distance_lowpass_ly = 0;
float FY_miss_distance_lowpass_ly = 0;
u8 lowpass_flat = 1; 
float ave_k=0.3;
u16 miss_Watch_CNT = 0;
float miss_Watch[1500] = {0};

extern int huanXiang_flag;
extern u16 X1_ly_cnt;
extern u16 X2_ly_cnt;
extern float X1_ly[1500];
extern float X2_ly[1500];

extern u8 ZhiXiangFlag;
u8 JiTing_Set_CNT = 0;
u8 JiTing_ZhongDuan_CNT = 0;
float FW_encoder_JiTing_Set = 0;
float FY_encoder_JiTinga_Set = 0;

extern float WaiTongBu_step_design_set;
extern u32 WaiTongBu_MaiChong_cnt;
extern int WaiTongBu_Flag_Com_Direction;                    //方向只赋一次值
extern float WaiTongBu_PanFangXiang;
extern u8 WaiTongBu_Start_cnt;
extern u16 Chang_Kp_CNT;

u16 KongJianZhiXiang_CNT = 0;
float KongJianZhiXiang_FW = 0;
float KongJianZhiXiang_FY = 0;

u16 ZhiXiang_Complete_Flag = 0;
extern u8 EXIT_POINT_FLAG;
extern u8 KongJianPoin_flag;
extern u8 SCAN_POINT_FLAG;

u8 GuiLingFlag = 0;
extern u8 ZhiXiang_Complete_FlagLY1;
extern u8 YunSu_POINT_FLAG;
extern u8 AA_Data[];
extern u16 ScanFunction_Number;
extern u16 YunSuFunction_Number;

extern u8 ZhiXiang_Complete_FlagLY;
extern u8 ZhiXiang_Complete_FlagYunSu;

extern float FW_Miss_distance;//方位脱靶量
extern float FY_Miss_distance;//俯仰脱靶量

extern u8 EXIT_LianXu_FLAG;
extern u8 EXIT_Song;

u8 FW_Xian_Left = 1;
u8 FW_Xian_Right = 1;

extern float r_change_later;
extern float y_change_later;
extern float p_change_later;

int FW_LY_CNT = 0;
u8 last_position_flag = 0;

float GXKJ_FW = 0;
float GXKJ_FY = 0;

u8 Test_XiangWei = 0;

u8 start_flat = 0;
u8 start_CNT = 0;
extern float Point_FY_set_prepass;
float Position_begin = 0;
float Encoder_Start = 0;

extern float FY_miss_distance_error ;            
extern float FW_miss_distance_error ; 
extern float FW_miss_distance_error_lowpass;      //方位脱靶量80Hz低通滤波
extern float FY_miss_distance_error_lowpass;      //俯仰脱靶量80Hz低通滤波

extern u8 miss_cnt_10ms;

//float GD_angle = 0.022;
float GD_angle = 1.29;
INAngle inAngle;

extern float FW_New_BM;
extern float FY_New_BM;
float FW_Space = 0;
float FY_Space = 0;

extern float Space_FY;
extern float Space_FW;
float v_zhixiang_num_zero=60;
float FY_OFSET = 0.001;
float FW_OFSET = 0.0028;
extern float error_sum_limit;
extern float FW_location_ki;
extern float FY_location_ki;
float Change_FW_ki = 0.04;
float Change_limit = 50;
float yaw_attitude_change = 0;
u8 ly_GD_flag = 2;
int ZhenPin = 0;

extern float erFsm2_servo[2][3][10];
extern float euFsm2_servo[2][3][10];

extern u8 BeginMove_flag;
extern float FY_Target_Position;

u16 OutSpeedCnt = 0;
u16 CntFW = 0;
float SpeedNowFW = 0;
float MaxPosition = 0;
float MinPosition = 0;
float OutSpeedFrequency = 0.1;
u16 OutCount = 0;
u16 FrequencyCnt = 0;
float SpeedBode[50] = {0};
float OutSpeedAmp = 2;
extern float FW_pointing_degree_LY;
extern u8 Mode_Chose;
extern float FY_Speed_Position;
extern float FY_485_SpeedFloat;


/*相机控制相关*/
extern u8 CAM_Data[],CAM_Answer_Data[];
extern u16 Threshold;//阈值
extern u8 Windows_Switch;
extern u16 X_Length_Setting,Y_Length_Setting;
extern u32 Exposure_Time;//曝光时间
u8 Camera_Config_cnt=0;
//以下4行为测试用变量,测试后可删除
extern u32 Exposure_Time_temp;
extern u16 Threshold_temp;
extern u8 Windows_Switch_temp;
extern u16 X_Length_Setting_temp,Y_Length_Setting_temp;
/*************/
u8 Camer_Set_CNT = 0;
u8 FW_Choose_Mode = 0;
u8 FY_Choose_Mode = 0;
float FW_Speed_Position = 0;
u16 Test_ZhenPin_CNT = 0;
u16 ZhenPin_1 = 0;
u16 ZhenPin_2 = 0;
extern float X_Migration;
extern float Y_Migration;
////////////跟踪模式光斑判断标志位////////////
float GS_Trace_flag=0;
/////////////////////////////////////////////////
int Mod_Exechange_Flag=0;
extern float Lici_flag_GS;
//求RMS的和的平方值
float RMS2_vol(float Final,float Final2)
{
		float X=0.0,Y=0.0;
		X=CalculateRMS_1(Final)*29.5;
		Y=CalculateRMS_2(Final2)*29.5;
}

float RMS2_vol1(float Final)
{
		float X=0.0;
		X=CalculateRMS_1(Final)*29.5;
		return X;
}

float RMS2_vol2(float Final2)
{
		float Y=0.0;
		Y=CalculateRMS_2(Final2)*29.5;
		return Y;
}

void TIM10_Init(u16 arr,u16 psc)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);  

		TIM_TimeBaseInitStructure.TIM_Period = arr; 	  //自动重装载值
		TIM_TimeBaseInitStructure.TIM_Prescaler=psc;      //定时器分频
		TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
		TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

		TIM_TimeBaseInit(TIM10,&TIM_TimeBaseInitStructure);//初始化TIM3

		TIM_ITConfig(TIM10,TIM_IT_Update,ENABLE); //允许定时器3更新中断
		TIM_Cmd(TIM10,ENABLE); //使能定时器3

		NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //定时器3中断
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//void TIM2_config(void)
//{   
//	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

//	TIM_BaseInitStructure.TIM_Period = 1000-1;	  	//设置周期
//	TIM_BaseInitStructure.TIM_Prescaler = 84-1;	  	//预分频数，定时器主频位84M，预分频设置为83，分频84倍，一个时钟脉冲位1us
//	TIM_BaseInitStructure.TIM_ClockDivision = 0;  	//时钟分割
//	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//计数器向上计数
//	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);	//执行初始化函数    
////	TIM_ARRPreloadConfig(TIM2, ENABLE);				//启用ARR的影子寄存器（直到产生更新事件才更改设置）
//	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
//	TIM_Cmd(TIM2, ENABLE);											//使能定时器2
//	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);		
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//	
//}


//void TIM3_Int_Init(u16 arr,u16 psc)
//{
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
//	
//	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
//	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
//	
//	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//	TIM_Cmd(TIM3,ENABLE); //使能定时器3
//	
//	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
//	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//	
//}


/*
//void TIM2_IRQHandler(void)
//{
//  if(TIM_GetITStatus(TIM2, TIM_IT_Update)==1)
//  {
////		Time++;
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
//	}		
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
//}


//void TIM3_IRQHandler(void)
//{
////	if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //溢出中断1MS中断
//	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==1)
//	{
//		GPIO_ToggleBits(GPIOE, GPIO_Pin_2);
//		GPIO_ToggleBits(GPIOA, GPIO_Pin_6);	
//	}
//	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 
//}
*/
float saopin_fuzhi,saopin_pinlv,saopin_pianzhi;
extern float EXIT_FW_Zero,FY_WaiTongBu_ZhiXing,EXIT_FY_Zero;
//float DATA_1[300],DATA_2[300],DATA_3[300],DATA_4[300],DATA_5[300],DATA_6[300],DATA_7[300],DATA_8[300],DATA_9[300];
u8 EXTI_CNT,LY_flag_N=0,A;
u16 a_i;
u16 Write_485_CNT = 0;
//定时器3中断服务函数
void TIM1_UP_TIM10_IRQHandler(void)
{
		uint32_t i = 0;
		if(TIM_GetITStatus(TIM10,TIM_IT_Update)==SET) //溢出中断1MS中断
		{		
//			GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
//			GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
			ZhenPin++;
			TuoBa_fLAG=*(uint32_t*)(PC_TuoBaADDRFlag);	//0x6400F000
			TEST_ReadDataFlag=*(uint32_t*)(PC_ComReadADDRFlag);   //如果串口里有数，TEST_ReadDataFlag标志位会置位
//			Send_Tongbuxinhao_Write_High();
			read_miss_distance();	  
			read_FY_encoder();									//读取俯仰编码器
			read_FW_encoder();									//读取方位编码器
			//2022.5.8读取光纤惯导串口5
			read_inertial_navigation(); 
			if((TuoBa_fLAG & 0x0002)== 0x0002)			//RS422串口2：（921600）
			{
					Receive_JiShangData_Read()	;				//粗伺服接收机上总控板的数据（粗伺服板与机上总控板通信）
			}
			
			//双线性变换参数设置
			para_set();
			GPIO_SetBits(GPIOC,GPIO_Pin_10);
			
			Receive_DBJsuidong_Read();
			
			
//			GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
//			GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
//			Send_Tongbuxinhao_Write_Low();							
				
			/////////////////////////////跟踪模式光斑判断
			
			if(system_mode == 25&&read_usart5[2]== 0x20)
			{
			
			GS_Trace_flag=0;//无光斑
			GS_Deadscale_Flag=0;
			}
			else
			{
			
			GS_Trace_flag=1;//有光斑
				
			}
//			SinBode(saopin_fuzhi, saopin_pinlv, saopin_pianzhi);
	//1急停				
			if(system_mode == 1)    //急停就进入待机模式 
			{
					Mode_Chose = 1;					//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
					FW_output_angle_pa_last  = FW_encoder_degrees;//方位编码器转换为角度
					FY_output_angle_pa_last  = FY_encoder_degrees;//俯仰编码器转换为角度
					FW_light_loop_open_flag = 0;									//方位光闭环跟踪标志位
					FY_light_loop_open_flag = 0;									//俯仰光闭环跟踪标志位
					FW_tuoluo_loop_open_flag = 0;
					FY_tuoluo_loop_open_flag = 0;
					
					erFsm2_servo[0][0][7] = 0;
					erFsm2_servo[0][1][7] = 0;
					erFsm2_servo[0][2][7] = 0;
					
					euFsm2_servo[0][1][7] = 0;
					euFsm2_servo[0][2][7] = 0;
					
					erFsm2_servo[1][0][7] = 0;
					erFsm2_servo[1][1][7] = 0;
					erFsm2_servo[1][2][7] = 0;
					euFsm2_servo[1][1][7] = 0;
					euFsm2_servo[1][2][7] = 0;
				
				
				
	//		    output_open_flag = 0;
					DA12_out(0,0);	
					Flag_Com_Direction = 1;        //没用
				//清标志位
					huiling_once_flag = 1;					//清零志位
					Z_slave_1ms_distance_sum = 0;  //Z方位
					X_slave_1ms_distance_sum = 0;

					miss_distance_flag = 0;			//控制step_open_flag的标志位
		//			Send_MaiChong_CNT = 0;    //没用
					step_open_flag = 0;					//控制diaoyong_cnt的标志位
					step_design_set = 0;				//
					diaoyong_cnt = 0;						//
					MaiChong_cnt = 0;
		//			FY_Scan_Zhixing = 0;			//没用

					CaiJi_CNT = 0;					//没用

					JieYue_cnt = 0;					//没用
					Change_flag_ly = 0;			//没用
					
					FW_location_kp = FW_location_JiTing_kp;	//20
					FW_location_ki = FW_location_JiTing_ki; //0.1
					FW_location_kd = FW_location_JiTing_kd; //0.1
					
					Test_Scan_Time = 0;			//没用
					huanXiang_flag = 0;			//
						
					KongJianPoin_flag = 0;	//空间指向到定点标志位（SpacePoint()；㎏ongJianPoin_flag=1时，空间指向到定点
					GuiLingFlag = 0;


	//////////////////定位到当前点/////////////////////////////////////////////
					FY_location_loop_open_flag = 1;		//方位位置闭环标志位
					FW_location_loop_open_flag = 1; 	//俯仰位置闭环标志位
					FW_light_loop_open_flag = 0;			//方位光闭环跟踪标志位
					FY_light_loop_open_flag = 0;			//俯仰光闭环跟踪标志位
					FW_location_para = 4;//FW_location_para_set = 1,校正模式选择：为1时，只有K校正；为2时，一阶校正；为3时，二阶校正；为4时，PID校正
					FY_location_para = FY_location_para_set;//FY_location_para_set = 1
				
					JiTing_Set_CNT++;						//急停计数
					JiTing_ZhongDuan_CNT++;
					if(JiTing_Set_CNT == 1)
					{
							FW_encoder_JiTing_Set = FW_encoder_degrees;
							FY_encoder_JiTinga_Set = FY_encoder_degrees;
					}
					if(JiTing_Set_CNT == 6)                  //急停走到位后记录上次跟踪位置
					{
						if(last_position_flag == 0)			//
						{
							last_position_flag = 1;

							FW_output_angle_pa_last  = FW_encoder_degrees;     //FW_zero_degree_zheng   
							FY_output_angle_pa_last  = FY_encoder_degrees;     //FY_zero_degree
							
							FW_light_loop_revise = 0;
							FY_light_loop_revise = 0;
							
							FW_track_qiankui_speed = 0;
							FY_track_qiankui_speed = 0;
							
							
							FY_miss_distance_error = 0;
							FW_miss_distance_error = 0;
							FW_Miss_distance = 0;
							FY_Miss_distance = 0;
							miss_distance_X_float = 0;
							miss_distance_Y_float = 0;
							miss_distance_Y_float_to_angle = 0;
							miss_distance_X_float_to_angle = 0;
							FW_miss_distance_error_lowpass = 0;     
							FY_miss_distance_error_lowpass = 0;     
							
							FY_track_qiankui_speed_lowpass = 0;
							FW_track_qiankui_speed_lowpass = 0;
							
							miss_cnt_10ms = 0;
							Z_slave_1ms_distance = 0;
							X_slave_1ms_distance_SV = 0;
						}
					}
				
	//			if(JiTing_Set_CNT > 30)
	//			{
	//				FW_location_set = FW_encoder_JiTing_Set;			
	//				FY_location_set = FY_encoder_JiTinga_Set;
	//				velocity_limit_value_set = 5;	
	//				accelerate_limit = 1000;
	//				output_open_flag = 1;
	//			}
					if(JiTing_ZhongDuan_CNT == 3 || JiTing_Set_CNT == 60)
					{
							GPIO_SetBits(GPIOA,GPIO_Pin_6);
							Delay_us(110);
							GPIO_ResetBits(GPIOA,GPIO_Pin_6);
					}
					if(JiTing_Set_CNT > 60)
					{
						JiTing_Set_CNT = 60;
					}
					if(JiTing_ZhongDuan_CNT > 3)
					{
						JiTing_ZhongDuan_CNT = 3;	
					}
				
					FW_location_set = FW_encoder_JiTing_Set;			
					FY_location_set = FY_encoder_JiTinga_Set;
					velocity_limit_value_set = 5;	
					accelerate_limit = 1000;
					output_open_flag = 1;
				
					if(EXIT_Song == 1)
					{
						EXIT_LianXu_FLAG = 1;
					}			
			}
	//2掉电=失磁					
			if(system_mode == 2)    //急停就进入待机模式 
			{       
				  Lici_flag_GS=0;
					output_open_flag = 0; //关闭输出
					power_down();
			}
			FW_Xian_Left = 1;   //没用
			FW_Xian_Right = 1;	//没用
			//////////////////////////////////////////////积分清零！！！！！！
			if(system_mode != 25&&system_mode != 26)
			{
					miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					z_axis_velocity_1msdistance_sum = 0;
					x_axis_velocity_1msdistance_sum = 0;
					y_axis_velocity_1msdistance_sum = 0;
			}
			/////////////////////////////////////////////////////////
	//3回零模式√			
			if(system_mode == 3)    //回零模式
			{
				  miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					z_axis_velocity_1msdistance_sum = 0;
					x_axis_velocity_1msdistance_sum = 0;
				  y_axis_velocity_1msdistance_sum = 0;
					FY_location_loop_error_sum = 0;
					FW_location_loop_error_sum = 0;
					GPIO_ResetBits(GPIOG, GPIO_Pin_7);		//切换指定的GPIO引脚
					miss_distance_X_flag = 1;
					FW_tuoluo_loop_open_flag = 1;
					FY_tuoluo_loop_open_flag = 1;
					FW_pointing_degree_LY = 0;
					Point_FY_set_prepass  = 0;
					counter1=0;
					counter2=0;
					counter_j=0;
					Mode_Chose = 1;//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
					counter_data=1;
					sin_cnt = 0;//sin计数
					sin_value = 0;//sin计算值
					FW_output_angle_pa_last  = FW_encoder_degrees;//方位编码器转换为角度
					FY_output_angle_pa_last  = FY_encoder_degrees;//俯仰编码器转换为角度
					
					EXTI2_FLAG = 0;
					last_position_flag = 0;  //跟踪时记录上次位置一次
					FW_Xian_Left = 0;
					FW_Xian_Right = 0;
	/////////////////////清除内同步缓存/////////////////////////////////				
					SCAN_POINT_FLAG = 0;
					step_design_set_ly = 0;
					Scan_cnt_ly = 0;
					Flag_Com_Direction_ly = 1;
					PanFangXiang = 0;
					Start_cnt = 1;	
					ScanFunction_Number = 0;
					ZhiXiang_Complete_FlagLY1 = 0;
	/////////////////////清除匀速缓存/////////////////////////////////			
					YunSu_POINT_FLAG = 0;
					YunSu_step_design_set_ly = 0;
					YunSu_Start_cnt = 1;
					YunSu_PanFangXiang = 0;
					Flag_YunSu_Direction_ly = 1;
					YunSu_cnt_ly = 0;
					YunSuFunction_Number = 0;
					ZhiXiang_Complete_FlagYunSu = 0;
	/////////////////////清除外同步缓存/////////////////////////////////	
					EXIT_POINT_FLAG = 0;
					WaiTongBu_step_design_set = 0;
					WaiTongBu_Start_cnt = 1;
					WaiTongBu_PanFangXiang = 0;
					WaiTongBu_Flag_Com_Direction = 1;
					WaiTongBu_MaiChong_cnt = 0;
					EXTI2_CNT = 0;
					ZhiXiang_Complete_FlagLY = 0;
					
					EXIT_LianXu_FLAG = 0;
								EXIT_Song = 0;
	/////////////////////////////////////////////////////////////////////////////////			
					JiTing_Set_CNT = 0;          //急停计数
					SpaceSetValueFW = 0;
					SpaceSetValueFY = 0;
					reset_zero(); 										//只发一次指令
					velocity_limit_value_set = v_zhixiang_num_zero;
					accelerate_limit = 1000;
					output_open_flag = 1;
					KongJianZhiXiang_FW = 0;
					KongJianZhiXiang_FY = 0;
					FW_pointing_degree_set = 0;
					FY_pointing_degree_set = 0;
	//////////////////////////////归零稳定脉冲/////////////////////////////////////			
					if((FY_encoder_degrees >= 81.5)&&(FY_encoder_degrees <= 82.5))
					{
							if((FW_encoder_degrees >= 337.00)&&(FW_encoder_degrees <= 338.00))   //201.195     201.205
							{
									GuiLingFlag++;
									if(GuiLingFlag == 1)
									{
											GPIO_SetBits(GPIOA,GPIO_Pin_6);
											Delay_us(110);
											GPIO_ResetBits(GPIOA,GPIO_Pin_6);	
									}
									else
									{
											GuiLingFlag = 2;
									}
							}

					}
					GPIO_SetBits(GPIOG, GPIO_Pin_7);
	///////////////////////////////////////////////////////////////////////////////////			
				//励磁
				//设置方位俯仰零点
				//开启位置闭环
				//限速，闭环参数组选1
	//			FW_output_angle_pa_last  = FW_zero_degree_zheng;     //FW_zero_degree_zheng   
	//			FY_output_angle_pa_last  = FY_zero_degree;     //FY_zero_degree
			}
	//4励磁
			if(system_mode == 4)    //励磁
			{      
				    Lici_flag_GS=1;
					output_open_flag = 0;			//关闭输出
					
					wMO_FW(0x37f,1);//写励磁
					wMO_FY(0x37f,1);
					DA12_out(9999,0);	
					DA12_out(0,9999);
			}			
			if(system_mode == 5)    //励磁FW
			{
					output_open_flag = 0;			//关闭输出
					wMO_FW(0x37f,1);//写励磁
					wMO_FY(0x37f,0);
			}
			if(system_mode == 6)    //励磁FY
			{
					output_open_flag = 0;			//关闭输出
					wMO_FW(0x37f,0);//写励磁
					wMO_FY(0x37f,1);
			}
	//7脱靶量跟踪
			if(system_mode == 7)
			{
					FW_tuoluo_loop_open_flag = 0;
					FY_tuoluo_loop_open_flag = 0;
					JiTing_Set_CNT = 0;          //急停计数
					last_position_flag = 0;  //跟踪时记录上次位置一次
	//		    if(last_position_flag == 0)
	//			{
	//				last_position_flag = 1;
	//				FW_output_angle_pa_last  = FW_encoder_degrees;     //FW_zero_degree_zheng   
	//				FY_output_angle_pa_last  = FY_encoder_degrees;     //FY_zero_degree
	//			}

	//			miss_distance_Y_float = 2.5;      //测试用
				
					if(miss_distance_X_float == -1024 || miss_distance_Y_float == -1024)
					{
							Mode_Chose = 1;											//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
							FY_location_loop_open_flag = 1;
							FW_location_loop_open_flag = 1;
							FW_light_loop_open_flag = 0;
							FY_light_loop_open_flag = 0;
							
							FW_location_para = 4;  //FW_location_para_set
							FY_location_para = 4;  //FY_location_para_set
							
			//				FW_location_para = FW_location_para_set;
			//				FY_location_para = FY_location_para_set;
							TuoBa_Set_CNT++;
							if(TuoBa_Set_CNT == 1)
							{
								FW_encoder_TuoBa_Set = FW_encoder_degrees;
								FY_encoder_TuoBa_Set = FY_encoder_degrees;
							}
							if(TuoBa_Set_CNT > 1)
							{
								TuoBa_Set_CNT = 1;	
							}
							FW_location_set = FW_encoder_TuoBa_Set;			
							FY_Target_Position = FY_encoder_TuoBa_Set;
							velocity_limit_value_set = 5;	//5/10=0.5 表示最高速度0.5°
							accelerate_limit = 1000;
							output_open_flag = 1;
					}
					else
					{
							TuoBa_Set_CNT = 0;
							ServoFunction();		
					}
			}
			
	//17轴上指向(编码器指向)
			if(system_mode == 17)
			{
					float PI=3.1415926;
					output_open_flag = 1;
					FW_tuoluo_loop_open_flag = 1;
					FY_tuoluo_loop_open_flag = 1;
				  GPIO_SetBits(GPIOG, GPIO_Pin_7);
					
					miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					z_axis_velocity_1msdistance_sum = 0;
					x_axis_velocity_1msdistance_sum = 0;
				  y_axis_velocity_1msdistance_sum = 0;
					FY_location_loop_error_sum = 0;
					FW_location_loop_error_sum = 0;
//				  GS_TRACE_FW_DEG=FW_Degree_GS;
//				  GS_TRACE_FY_DEG=FY_Degree_GS;
//				  GS_FW_DEG=cos(GS_TRACE_FW_DEG*(PI/180))*cos(GS_TRACE_FY_DEG*(PI/180))*y_axis_velocity_1msdistance_sum+sin(GS_TRACE_FW_DEG*(PI/180))*x_axis_velocity_1msdistance_sum-cos(GS_TRACE_FW_DEG*(PI/180))*sin(GS_TRACE_FY_DEG*(PI/180))*z_axis_velocity_1msdistance_sum;
//				  GS_FY_DEG=0-sin(GS_TRACE_FW_DEG*(PI/180))*cos(GS_TRACE_FY_DEG*(PI/180))*y_axis_velocity_1msdistance_sum+cos(GS_TRACE_FW_DEG*(PI/180))*x_axis_velocity_1msdistance_sum+sin(GS_TRACE_FW_DEG*(PI/180))*sin(GS_TRACE_FY_DEG*(PI/180))*z_axis_velocity_1msdistance_sum;
//				  GS_HG_DEG=sin(GS_TRACE_FY_DEG*(PI/180))*y_axis_velocity_1msdistance_sum+cos(GS_TRACE_FY_DEG*(PI/180))*z_axis_velocity_1msdistance_sum;
				  Deg_InverseSolution(FW_Degree_GS,FY_Degree_GS);//光栅项目指向公式（5.7）FW_Degree_GS
					
					SpacePoint();
//					if(wMO_count == 1)
//					{
//						output_open_flag = 0;	
//						wMO_FW(0x37f,1);//写励磁
//						wMO_FY(0x37f,1);
//						DA12_out(9999,0);	
//						DA12_out(0,9999);
//						wMO_count = 0;
//					}
					GPIO_ResetBits(GPIOG, GPIO_Pin_7);
			}	
			
//25轴上指向(编码器指向)跟踪
			if(system_mode == 25)
			{   
				  float PI=3.1415926;
				  float cos_x,cos_y,sin_x,sin_y;
					output_open_flag = 1;
					FW_tuoluo_loop_open_flag = 0;
				
					FW_tuoluo_loop_open_flag = 0;
					FY_tuoluo_loop_open_flag = 0;
					x_axis_velocity_1msdistance = x_axis_velocity_float * 0.001;
					x_axis_velocity_1msdistance_sum = x_axis_velocity_1msdistance_sum + x_axis_velocity_1msdistance;
					y_axis_velocity_1msdistance = gs_x_k*y_axis_velocity_float * 0.001;
					y_axis_velocity_1msdistance_sum = y_axis_velocity_1msdistance_sum + y_axis_velocity_1msdistance;
		      z_axis_velocity_1msdistance = z_axis_velocity_float * 0.001;
					z_axis_velocity_1msdistance_sum = z_axis_velocity_1msdistance_sum + z_axis_velocity_1msdistance;
					
//				    Deg_InverseSolution(FW_Degree_GS+z_axis_velocity_1msdistance_sum+kp_guangshan*miss_distance_X_float_sum,FY_Degree_GS-x_axis_velocity_1msdistance_sum+kp_guangshan_Y*miss_distance_Y_float_sum);//光栅项目指向公式（5.7）FW_Degree_GS
				if(FY_Degree_GS_suidong*FY_Degree_GS_suidong+FW_Degree_GS_suidong*FW_Degree_GS_suidong>=9&&FY_Degree_GS_suidong*FY_Degree_GS_suidong+FW_Degree_GS_suidong*FW_Degree_GS_suidong<=221)
				{
				if(GS_Trace_flag==1)//光斑判断（有光斑时执行跟踪程序）
				{
					FY_location_kp=220;
					FY_location_kp=220;
					FW_location_ki = 0.05;//0.01
          FY_location_ki = 0.05;//0
//					Deg_InverseSolution(GS_Trace_FW,GS_Trace_FY);
					GS_Deadscale_Flag++;
					
					if(GS_Deadscale_Flag>=500)
					{
						if(GS_Deadscale_Flag==500)
					{
					GS_1=FW_Degree_GS_suidong;
					GS_2=FY_Degree_GS_suidong;
					}
						FW_Degree_GS=GS_1;
						FY_Degree_GS=GS_2;
					miss_distance_X_float_sum = miss_distance_X_float_sum + miss_distance_X_float* 16.5 * 0.000057295;
					miss_distance_Y_float_sum = miss_distance_Y_float_sum + miss_distance_Y_float* 16.5 * 0.000057295;
					
				  GS_TRACE_FW_DEG=FW_Degree_GS;
				  GS_TRACE_FY_DEG=FY_Degree_GS;
				  cos_x=cos(GS_TRACE_FW_DEG*(PI/180));
				  cos_y=cos(GS_TRACE_FY_DEG*(PI/180));
				  sin_x=sin(GS_TRACE_FW_DEG*(PI/180));
					sin_y=sin(GS_TRACE_FY_DEG*(PI/180));
				  GS_HG_DEG=cos_y*cos_x*y_axis_velocity_1msdistance_sum+cos_y*sin_x*x_axis_velocity_1msdistance_sum-sin_y*z_axis_velocity_1msdistance_sum;
				  GS_FY_DEG=0-sin_x*y_axis_velocity_1msdistance_sum+cos_x*x_axis_velocity_1msdistance_sum;
				  GS_FW_DEG=sin_y*cos_x*y_axis_velocity_1msdistance_sum+sin_y*sin_x*x_axis_velocity_1msdistance_sum+cos_y*z_axis_velocity_1msdistance_sum;
          GS_Trace_FW=FW_Degree_GS-GS_FW_DEG+kp_guangshan*miss_distance_X_float_sum;
          GS_Trace_FY=FY_Degree_GS+GS_FY_DEG+kp_guangshan_Y*miss_distance_Y_float_sum;
					Deg_InverseSolution(GS_Trace_FW,GS_Trace_FY);
//					Deg_InverseSolution(FW_Degree_GS_suidong,FY_Degree_GS_suidong);
					SpacePoint();
					}
					else
					{
					FY_location_kp=300;
					FY_location_kp=300;
					FW_location_ki = 0;
          FY_location_ki = 0;
					FY_location_loop_error_sum=0;
					FW_location_loop_error_sum=0;
					miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					z_axis_velocity_1msdistance_sum = 0;
					x_axis_velocity_1msdistance_sum = 0;
				  y_axis_velocity_1msdistance_sum = 0;
					Deg_InverseSolution(FW_Degree_GS_suidong,FY_Degree_GS_suidong);
					SpacePoint();
					}
				}
				else//光斑判断(无光斑时执行稳定程序)
				{
					FY_location_kp=300;
					FY_location_kp=300;
					FW_location_ki = 0;//0.01
          FY_location_ki = 0;//0
					FY_location_loop_error_sum=0;
					FW_location_loop_error_sum=0;
//					miss_distance_X_float_sum = 0;
//   				miss_distance_Y_float_sum = 0;
//				  Deg_InverseSolution(FW_Degree_GS-GS_FW_DEG,FY_Degree_GS+GS_FY_DEG);
					GS_Deadscale_Flag=0;
					miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					z_axis_velocity_1msdistance_sum = 0;
					x_axis_velocity_1msdistance_sum = 0;
				  y_axis_velocity_1msdistance_sum = 0;
					Deg_InverseSolution(FW_Degree_GS_suidong,FY_Degree_GS_suidong);
					SpacePoint();

				}
					
					

//					}
			}	
				else
				{
				GS_Deadscale_Flag=0;
				miss_distance_X_float_sum = 0;
				miss_distance_Y_float_sum = 0;
				z_axis_velocity_1msdistance_sum = 0;
				x_axis_velocity_1msdistance_sum = 0;
				y_axis_velocity_1msdistance_sum = 0;
					FY_location_loop_error_sum=0;
					FW_location_loop_error_sum=0;
				Deg_InverseSolution(0.01,0.01);
				SpacePoint();
				}
			}
//稳定
			if(system_mode == 26)
			{       
					output_open_flag = 1;
					GPIO_SetBits(GPIOG, GPIO_Pin_7);
					KongJianPoin_flag = 0;
					TuoLuo_Flag = 3;
					ZhiXiangFlag = 2;
					Eqution_Flag = 6;
				
					FW_tuoluo_loop_open_flag = 0;
					FY_tuoluo_loop_open_flag = 0;
					
					miss_distance_X_float_sum = 0;
					miss_distance_Y_float_sum = 0;
					x_axis_velocity_1msdistance = x_axis_velocity_float * 0.001;
					x_axis_velocity_1msdistance_sum = x_axis_velocity_1msdistance_sum + x_axis_velocity_1msdistance;
					
					y_axis_velocity_1msdistance = y_axis_velocity_float * 0.001;
					y_axis_velocity_1msdistance_sum = y_axis_velocity_1msdistance_sum + y_axis_velocity_1msdistance;
					
		      z_axis_velocity_1msdistance = z_axis_velocity_float * 0.001;
					z_axis_velocity_1msdistance_sum = z_axis_velocity_1msdistance_sum + z_axis_velocity_1msdistance;
					
				    Deg_InverseSolution(FW_Degree_GS-z_axis_velocity_1msdistance_sum,FY_Degree_GS+x_axis_velocity_1msdistance_sum);//光栅项目指向公式（5.7）FW_Degree_GS
//					x_axis_sum = FW_Degree_GS+z_axis_velocity_1msdistance_sum;
//					y_axis_sum = 
					
					SpacePoint();
					
					
					GPIO_ResetBits(GPIOG, GPIO_Pin_7);
			}				

	//21 扫描设置	(环形走位设置)
			if(system_mode == 21)
			{
					if(counter_data==1)
					{
						SaoMiao_Init();	
					}
					else
					{
						counter1++;
						if(counter1==100)	//延时100MS
						{
							counter_j++;
							if(counter_j >= n_LYJ)
							{
								counter_j = n_LYJ-1;
							}
							counter1=0; 
							Mode_Chose = 1;			//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
							JiTing_Set_CNT = 0;          //急停计数
							last_position_flag = 0;  //跟踪时记录上次位置一次
							KongJianPoin_flag = 0;		//空间指向到定点标志位（SpacePoint();）
							ZhiXiangFlag = 2;				
											
							FW_pointing_degree_LY = ab[counter_j][0];		 //方位角
							Point_FY_set_prepass = ab[counter_j][1];		 //俯仰角
						
							FW_tuoluo_loop_open_flag = 0;
							FY_tuoluo_loop_open_flag = 0;
							
							SpacePoint();
						}	
					}	
			}	
	
			/////////////////俯仰电机选择位置环、速度环控制////////////////////////
			if(system_mode == 7)
			{
				Mode_Chose = 2;		//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
			}
			else
			{
				Mode_Chose = 1;				//Mode_Chose = 1位置环控制，Mode_Chose = 2速度环控制
			}
			
	//输出
			if(output_open_flag == 1)
			{
						//光闭环跟踪
						if(FW_light_loop_open_flag == 1)  
						{
								//光闭环输出角度 = 上一刻角度 + 脱靶量校正角度
//								FW_output_angle_pa_now = FW_output_angle_pa_last + FW_light_loop_revise;//FW_output_angle_pa_last
								//设定编码器闭环值（位置环）
								if(FW_Choose_Mode == 1)   // FW_Choose_Mode = 0
								{
									FW_location_set = FW_output_angle_pa_now;//光闭环输出角度
								}
								//选择方位速度环
								else
								{
									FW_Speed_Position = FY_light_loop_revise;//í?°Dá?D￡?y???è
									FW_location_loop_open_flag = 0;//1?±?????±??·
									BeginMortorFW();
								}
								
//								FW_track_qiankui_speed = FW_light_loop_revise * 1000;       
//								FW_track_qiankui_speed_lowpass = FsmLeadLag1(FW_track_qiankui_speed,FSM_X,4);//跟踪速度前馈滤波	 ，目前没用这个功能
//								
//								//存储当前角度
//								FW_output_angle_pa_last = FW_output_angle_pa_now;//光闭环输出角度
//								//打开位置闭环
								FW_location_loop_open_flag = 1;
						}
						if(FY_light_loop_open_flag == 1)
						{
								//光闭环输出角度 = 上一刻角度 + 脱靶量校正角度
//								FY_output_angle_pa_now = FY_output_angle_pa_last + FY_light_loop_revise;
								//设定编码器闭环值
								if(FY_Choose_Mode == 1)
								{
									FY_location_set = FY_output_angle_pa_now;//光闭环输出角度
								}
								else
								{
									FY_Speed_Position = FW_light_loop_revise;//í?°Dá?D￡?y???è
									FY_location_loop_open_flag = 0;//1?±?????±??·
									BeginMortorFY();
								}	
			/////////////////////485位置环//////////////////////////////////
			//				FY_Target_Position = FY_output_angle_pa_now;
										
			/////////////////////485速度环/////////////////////////////////
			//                FY_Speed_Position = (FY_output_angle_pa_now - FY_output_angle_pa_last) * 1000;
			//                 FY_Speed_Position = FY_light_loop_revise;
							
//								FY_track_qiankui_speed = FY_light_loop_revise * 1000;       
//								FY_track_qiankui_speed_lowpass = FsmLeadLag1(FY_track_qiankui_speed,FSM_Y,4);     //跟踪速度前馈滤波	 
//								
//								//存储当前角度
//								FY_output_angle_pa_last = FY_output_angle_pa_now;//光闭环输出角度
//								//打开位置闭环
								FY_location_loop_open_flag = 1;
//								FY_tuoluo_loop_open_flag  = 1;
						}

		//前馈滤波
				//位置闭环
						if(FY_location_loop_open_flag == 1) //打开俯仰位置闭环
						{        
								if(FY_location_para == 7)
								{		
										//	 w2*s+1
										//	---------
										//	 w1*s+1
										FY_encoder_degrees_lowpass = FsmLeadLag1(FY_encoder_degrees,FSM_Y,10);      //
								}
//								if(system_mode==26||system_mode==25)
//								{
//				FY_location_kp=330;
//				FY_location_ki=0.012;
//				kp_guangshan_Y=0.024;
//								}
								fy_close_loop();//校正函数
								BeginMortorFY();//限速函数
//								FY_location_last = FY_location_set;//编码器闭环设定值322.8715
						}
					
						Delay_us(10);
						if(FW_location_loop_open_flag == 1) //打开方位位置闭环
						{
								if(FW_location_para == 7)
								{		
										//	 w2*s+1
										//	---------
										//	 w1*s+1
										FW_encoder_degrees_lowpass = FsmLeadLag1(FW_encoder_degrees,FSM_X,10);      //一阶滞后校正80Hz滤波
								}
//								if(system_mode==26||system_mode==25)
//								{
//				FW_location_kp=330;
//				FW_location_ki=0.012;
//				kp_guangshan=0.024;
//								}
								fw_close_loop();
								BeginMortorFW();
						}
						
				//打开稳定环闭环
						if(FW_tuoluo_loop_open_flag == 1) //打开方位稳定环闭环
						{
								fw_tuoluo_close_loop();
								BeginMortorFW();
						}
						if(FY_tuoluo_loop_open_flag == 1) //打开俯仰稳定环闭环
						{
								fy_tuoluo_close_loop();
								BeginMortorFY();
						}
			}
			SF_Jixia ++;
			if(SF_Jixia == 10)											//100hz发送一帧数据
			{
					SF_Jixia = 0;
//					Send_SW_Data_TX();										//粗伺服发给机下总控板的数据
				Send_Data_TX();
			}
		GuanDaoWR_JiShu++;
		if(GuanDaoWR_JiShu == 200)    //5Hz向惯导发一次指令
		{
//			GPIO_ToggleBits(GPIOE, GPIO_Pin_2);
			WriteGuanDao();
			GuanDaoWR_JiShu = 0;
		}
		}
		TIM_ClearITPendingBit(TIM10,TIM_IT_Update);    //清除中断标志位
}

////////////////////////////////////////↓↓↓子函数↓↓↓////////////////////////////////////////////////////////////////////////
//读取俯仰编码器
extern u8 FY_Xianwei;									//俯仰限位
float FY_encoder_quzhi = 0;
void read_FY_encoder(void)            
{
		if((TEST_ReadDataFlag&0x0040) == 0x0040)
		{		
			FY_encoder_l = *(uint32_t*)(((u32)(0x6400BB00)));
			FY_encoder_h = *(uint32_t*)(((u32)(0x6400BC00)));
			FY_encoder_u32 = FY_encoder_l + (FY_encoder_h <<16);
			low_2bit_FY = FY_encoder_u32 & 0x00000003;
			FY_encoder_u32 = FY_encoder_u32 >> 2;
			FY_encoder_degrees = FY_encoder_u32 / 186413.5 ;  //186413.6
			
//			FY_encoder_quzhi = FY_encoder_degrees;
			FY_Difference = FY_encoder_degrees_First - FY_encoder_degrees;	
			if(FY_Difference > -180.0 && FY_Difference < 180.0)
			{
					k_count_Y = k_count_Y;
			}
			else if(FY_Difference <= -180.0)
			{
					k_count_Y --;
			}
			else if(FY_Difference >= 180.0)
			{
					k_count_Y ++;
			}
			FY_encoder_degrees_First = FY_encoder_degrees;
			
			FY_encoder_degrees= FY_encoder_degrees + k_count_Y*360;	
//			if(FY_encoder_degrees > 180)
//			{
//					FY_encoder_degrees = FY_encoder_degrees - 360;
//			}
//			FY_encoder_degrees_lowpass = FsmLeadLag1(FY_encoder_degrees,FSM_Y,1);      //一阶滞后校正80Hz滤波
//			FY_encoder_Flag=1;	//没用
		}
		else
		{
			FY_encoder_Flag=2;
		}
		Encoder_Data.EncoderData_FY.EncodeLow = FY_encoder_l;
		Encoder_Data.EncoderData_FY.EncodeHigh=	FY_encoder_h;
		Encoder_Data.EncoderData_FY.Degrees	=	FY_encoder_u32;
		Encoder_Data.EncoderData_FY.Encode  = FY_encoder_degrees;
		Encoder_Data.EncoderData_FY.EncoderFlag = FY_encoder_Flag;
		Encoder_Data.EncoderData_FY.Low2bits = low_2bit_FY; 
			//判断俯仰限位
		if(FY_encoder_degrees > FY_encoder_Shangxianwei)
		{
				FY_Xianwei = 1;				//上限位
		}
		else if(FY_encoder_degrees < FY_encoder_Xiaxianwei)
		{
				FY_Xianwei = 2;				//下限位
		}
		else
		{
				FY_Xianwei = 0;
		}
}

//float FW_encoder_degrees1 = 0;
//读取方位编码器
void read_FW_encoder(void)            
{
		if((TEST_ReadDataFlag&0x0020) == 0x0020)
		{		
//			GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
//			GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
			FW_encoder_Flag=1;
			FW_encoder_l = *(uint32_t*)(((u32)(0x6400B900)));
			FW_encoder_h = *(uint32_t*)(((u32)(0x6400BA00)));
			FW_encoder_u32 = FW_encoder_l + (FW_encoder_h <<16);
			low_2bit_FW = FW_encoder_u32 & 0x00000003;
			FW_encoder_u32 = FW_encoder_u32 >> 2;
			FW_encoder_degrees = FW_encoder_u32  / 186413.5;//186413.511111111f
			
			FW_Difference = FW_encoder_degrees_First - FW_encoder_degrees;	
			if(FW_Difference > -180.0 && FW_Difference < 180.0)
			{
					k_count = k_count;
			}
			else if(FW_Difference <= -180.0)
			{
					k_count --;
			}
			else if(FW_Difference >= 180.0)
			{
					k_count ++;
			}
			FW_encoder_degrees_First = FW_encoder_degrees;
			FW_encoder_degrees= FW_encoder_degrees + k_count*360;	
	
//			FW_encoder_degrees_lowpass = FsmLeadLag1(FW_encoder_degrees,FSM_X,1);      //一阶滞后校正80Hz滤波
//			FW_encoder_degrees_lowpass = FW_encoder_degrees;
		}
		else
		{
			FW_encoder_Flag=2;
		}
		Encoder_Data.EncoderData_FW.EncodeLow = FW_encoder_l;
		Encoder_Data.EncoderData_FW.EncodeHigh=	FW_encoder_h;
		Encoder_Data.EncoderData_FW.Degrees	=	FW_encoder_u32;
		Encoder_Data.EncoderData_FW.Encode  = FW_encoder_degrees;
		Encoder_Data.EncoderData_FW.EncoderFlag = FW_encoder_Flag;
		Encoder_Data.EncoderData_FW.Low2bits = low_2bit_FW; 
}

void read_M1tuoluo(void)            
{
	if((TEST_ReadDataFlag&0x0004) == 0x0004)
	{		
		u8 usart_3_num = 0;
		FPGA_READ_TEST_ADDR = PFPGA_WRITE_USART3;
		for(usart_3_num=0;usart_3_num<9;usart_3_num++)
		{
			read_usart3[usart_3_num] = *(uint32_t*)(FPGA_READ_TEST_ADDR);
			FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
		}
		if(read_usart3[0] == 0x80)
		{
			IMU_bit1 = (read_usart3[1])&0x7F; 
			IMU_bit2 = (read_usart3[2])&0x7F; 
			IMU_bit3 = (read_usart3[3])&0x7F; 
			IMU_bit4 = (read_usart3[4])&0x7F; 
			IMU_bit5 = (read_usart3[5])&0x7F;
	        IMU_bit_u32 = (IMU_bit5<<28)|(IMU_bit4<<21)|(IMU_bit3<<14)|(IMU_bit2<<7)|IMU_bit1;		
            IMU_bit_int = IMU_bit_u32;	
            IMU_bit_float = IMU_bit_int	* 0.0001;
			
			//IMU_bit_float_lowpass = FsmLeadLag1(IMU_bit_float,0,4);  
		}
	}
}

void read_labview_set(void)
{
	if((TEST_ReadDataFlag&0x0008) == 0x0008)
	{
		u8 usart_4_num = 0;		
		FPGA_READ_TEST_ADDR = PFPGA_WRITE_USART4;
		for(usart_4_num=0;usart_4_num<196;usart_4_num++)	 //比FE大1
		{
			read_usart4[usart_4_num] = *(uint32_t*)(FPGA_READ_TEST_ADDR);
			FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
		}
		//if((read_usart4[0] == 0x55)&&(read_usart4[1] == 0xAA)&&(read_usart4[119] == 0xFE))
		if((read_usart4[0] == 0x55)&&(read_usart4[1] == 0xAA)&&(read_usart4[195] == 0xFE))   //FE协议前数字编号一致
		{
//			FW_location_loop_k = Ary4ToFloat(read_usart4[6],read_usart4[5],read_usart4[4],read_usart4[3]);               
//			FW_location_loop_k_1 = Ary4ToFloat(read_usart4[10],read_usart4[9],read_usart4[8],read_usart4[7]); 
//			FW_location_loop_k_2 = Ary4ToFloat(read_usart4[14],read_usart4[13],read_usart4[12],read_usart4[11]); 

//			FY_location_loop_k = Ary4ToFloat(read_usart4[18],read_usart4[17],read_usart4[16],read_usart4[15]);
//			FY_location_loop_k_1 = Ary4ToFloat(read_usart4[22],read_usart4[21],read_usart4[20],read_usart4[19]);
//			FY_location_loop_k_2 = Ary4ToFloat(read_usart4[26],read_usart4[25],read_usart4[24],read_usart4[23]);
//			
//			FW_wending_yijie_tao = Ary4ToFloat(read_usart4[30],read_usart4[29],read_usart4[28],read_usart4[27]);
//			FW_wending_yijie_T = Ary4ToFloat(read_usart4[34],read_usart4[33],read_usart4[32],read_usart4[31]);
//			FY_wending_yijie_tao = Ary4ToFloat(read_usart4[38],read_usart4[37],read_usart4[36],read_usart4[35]);
//			FY_wending_yijie_T = Ary4ToFloat(read_usart4[42],read_usart4[41],read_usart4[40],read_usart4[39]);
//			
//			FW_wending_erjie_tao = Ary4ToFloat(read_usart4[46],read_usart4[45],read_usart4[44],read_usart4[43]);
//			FW_wending_erjie_T = Ary4ToFloat(read_usart4[50],read_usart4[49],read_usart4[48],read_usart4[47]);
//			FY_wending_erjie_tao = Ary4ToFloat(read_usart4[54],read_usart4[53],read_usart4[52],read_usart4[51]);
//			FY_wending_erjie_T = Ary4ToFloat(read_usart4[58],read_usart4[57],read_usart4[56],read_usart4[55]);
//			
//			FW_lowpass = Ary4ToFloat(read_usart4[62],read_usart4[61],read_usart4[60],read_usart4[59]);
//          FY_lowpass = Ary4ToFloat(read_usart4[66],read_usart4[65],read_usart4[64],read_usart4[63]);
//			
//			FW_wending_qiankui_speed_k = Ary4ToFloat(read_usart4[70],read_usart4[69],read_usart4[68],read_usart4[67]);
//          FY_wending_qiankui_speed_k = Ary4ToFloat(read_usart4[74],read_usart4[73],read_usart4[72],read_usart4[71]);  
//			
//			FW_wending_qiankui_lvbo = Ary4ToFloat(read_usart4[78],read_usart4[77],read_usart4[76],read_usart4[75]);
//			FY_wending_qiankui_lvbo = Ary4ToFloat(read_usart4[82],read_usart4[81],read_usart4[80],read_usart4[79]);

//			FW_pre_lvbo = Ary4ToFloat(read_usart4[86],read_usart4[85],read_usart4[84],read_usart4[83]);
//			FY_pre_lvbo = Ary4ToFloat(read_usart4[90],read_usart4[89],read_usart4[88],read_usart4[87]);
			
			FW_light_loop_k = Ary4ToFloat(read_usart4[94],read_usart4[93],read_usart4[92],read_usart4[91]);
//			FW_light_loop_k_1 = Ary4ToFloat(read_usart4[98],read_usart4[97],read_usart4[96],read_usart4[95]);
			FW_light_loop_k_2 = Ary4ToFloat(read_usart4[102],read_usart4[101],read_usart4[100],read_usart4[99]);
			
			FY_light_loop_k = Ary4ToFloat(read_usart4[106],read_usart4[105],read_usart4[104],read_usart4[103]);
//			FY_light_loop_k_1 = Ary4ToFloat(read_usart4[110],read_usart4[109],read_usart4[108],read_usart4[107]);
			FY_light_loop_k_2 = Ary4ToFloat(read_usart4[114],read_usart4[113],read_usart4[112],read_usart4[111]);

//			FW_track_yijie_tao_servo = Ary4ToFloat(read_usart4[118],read_usart4[117],read_usart4[116],read_usart4[115]);  
//			FW_track_yijie_T_servo = Ary4ToFloat(read_usart4[122],read_usart4[121],read_usart4[120],read_usart4[119]);
//			FY_track_yijie_tao_servo = Ary4ToFloat(read_usart4[126],read_usart4[125],read_usart4[124],read_usart4[123]);
//			FY_track_yijie_T_servo = Ary4ToFloat(read_usart4[130],read_usart4[129],read_usart4[128],read_usart4[127]);
			
			FW_track_erjie_tao_servo = Ary4ToFloat(read_usart4[134],read_usart4[133],read_usart4[132],read_usart4[131]);  
			FW_track_erjie_T_servo = Ary4ToFloat(read_usart4[138],read_usart4[137],read_usart4[136],read_usart4[135]);  
			FY_track_erjie_tao_servo = Ary4ToFloat(read_usart4[142],read_usart4[141],read_usart4[140],read_usart4[139]);  
			FY_track_erjie_T_servo = Ary4ToFloat(read_usart4[146],read_usart4[145],read_usart4[144],read_usart4[143]);  
			
			FW_lowpass_servo = Ary4ToFloat(read_usart4[150],read_usart4[149],read_usart4[148],read_usart4[147]); 
			FY_lowpass_servo = Ary4ToFloat(read_usart4[154],read_usart4[153],read_usart4[152],read_usart4[151]); 
			
			FW_track_qiankui_speed_k = Ary4ToFloat(read_usart4[158],read_usart4[157],read_usart4[156],read_usart4[155]); 
            FY_track_qiankui_speed_k = Ary4ToFloat(read_usart4[162],read_usart4[161],read_usart4[160],read_usart4[159]); 
			
			FW_track_qiankui_lvbo = Ary4ToFloat(read_usart4[166],read_usart4[165],read_usart4[164],read_usart4[163]); 
            FY_track_qiankui_lvbo = Ary4ToFloat(read_usart4[170],read_usart4[169],read_usart4[168],read_usart4[167]); 
			
			system_mode = read_usart4[171];
			FW_pointing_degree_set = Ary4ToFloat(read_usart4[175],read_usart4[174],read_usart4[173],read_usart4[172]);
			FY_pointing_degree_set = Ary4ToFloat(read_usart4[179],read_usart4[178],read_usart4[177],read_usart4[176]);
			

//        	target_select = read_usart4[181];	          //稳定目标选择	
//			stabilization_select = read_usart4[182];	  //惯导稳定下目标选择
//			
//			FW_location_para_set = read_usart4[183];	  
//      	FY_location_para_set = read_usart4[184];
            FW_light_para_set = read_usart4[185];	    //
            FY_light_para_set = read_usart4[186];	
            stabilize_type_select = read_usart4[187];
			track_axis_select = read_usart4[188];
			FW_pi_K = Ary4ToFloat(read_usart4[192],read_usart4[191],read_usart4[190],read_usart4[189]);
		}
	}
}
void read_LVDS(void)
{
	u8 lvds1_num = 0;
    u8 lvds2_num = 0;
	if((TEST_ReadDataFlag&0x1000) == 0x1000)
	{
		FPGA_READ_TEST_ADDR=PFPGA_WRITE_LVDS+0x100;
		for(lvds1_num=0;lvds1_num<110;lvds1_num++)
		{
			lvds_array1[lvds1_num] = *(uint32_t*)(FPGA_READ_TEST_ADDR);
			FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
		}
	
	}
    if((TEST_ReadDataFlag&0x2000) == 0x2000)
	{	
	    FPGA_READ_TEST_ADDR=PFPGA_WRITE_LVDS+0x200;
		for(lvds2_num=0;lvds2_num<110;lvds2_num++)
		{
			lvds_array2[lvds2_num] = *(uint32_t*)(FPGA_READ_TEST_ADDR);
			FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
		}		
	}
}

void write_LVDS(void)
{
	u8 TX_DATA=0;
	u8 lent = 0;
	u8 testNUM1 = 0;
		
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x54;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x10;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x17;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x31;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 100;
		
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x54;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x10;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x17;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x31;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 100;
		
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x54;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x10;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x17;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x31;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 100;
	for(testNUM1=0; testNUM1<100; testNUM1++)    //写0x64002000
	{
		TX_DATA++;
		*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= TX_DATA;
		*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= TX_DATA;
		*(uint32_t*)(PFPGA_WRITE_LVDS)= TX_DATA;  
	}
		
	lent=11;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= lent;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS)= 0xfe;
	lent=22;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= lent;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x100)= 0xfe;
	lent=33;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= lent;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0x5a;
	*(uint32_t*)(PFPGA_WRITE_LVDS+0x200)= 0xfe;
}

void read_temperature(void)
{
	if((TEST_ReadDataFlag&0x0400) == 0x0400)
	{
		FPGA_READ_TEST_ADDR=PFPGA_WRITE_TEMP1;
	  temp1_u16 =*(uint32_t*)(FPGA_READ_TEST_ADDR);
		temp1_f=(float)(temp1_u16>>3)*(float)0.0625;
	}
	if((TEST_ReadDataFlag&0x0800) == 0x0800)
	{
		FPGA_READ_TEST_ADDR=PFPGA_WRITE_TEMP2;
	    temp2_u16 =*(uint32_t*)(FPGA_READ_TEST_ADDR);
		temp2_f=(float)(temp2_u16>>3)*(float)0.0625;
	}

}	

u8 usart_5_num = 0;

u8 usart_count1= 0;

//2022.5.7串口3读取光栅的脱靶量
void read_miss_distance(void)        
{
		if((TuoBa_fLAG&0x0004) == 0x0004)
		{
				GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
//				GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
				

				FPGA_READ_TEST_ADDR = PFPGA_WRITE_USART5;		//读取光栅相机脱靶量地址（光栅项目串口3）
				
				for(usart_5_num=0; usart_5_num<14; usart_5_num++)
				{
					read_usart5[usart_5_num]=*(uint32_t*)(FPGA_READ_TEST_ADDR);
					FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;		
				}
				
				if(read_usart5[0] == 0x55 && read_usart5[13] == 0xFE)
				{
//						GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
//						GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
						if(read_usart5[2] == 0x20 )
						{
							usart_count1++;
						}
						
						miss_distance_judge_X = (read_usart5[3] << 16) + (read_usart5[4] << 8) + read_usart5[5]; 
						miss_distance_judge_Y = (read_usart5[6] << 16) + (read_usart5[7] << 8) + read_usart5[8];
						if((miss_distance_judge_X & 0x00800000) == 0)
						{
								miss_distance_X = miss_distance_judge_X;
						}
						else
						{
								miss_distance_X = miss_distance_judge_X|0xFF000000;
						}		
						if((miss_distance_judge_Y & 0x00800000) == 0)
						{
								miss_distance_Y = miss_distance_judge_Y;
						}
						else
						{
								miss_distance_Y = miss_distance_judge_Y|0xFF000000;
						}

						//X轴脱靶量
						miss_distance_X_float_LowpassBefore = miss_distance_X / 100.0; //（像素点）
						miss_distance_X_float = miss_distance_X_float_LowpassBefore ;
						
						miss_distance_X_float_to_angle = miss_distance_X_float * 73.5 * 0.000057295; //（脱靶量转换为度）//14
			//          miss_distance_X_float_to_angle = miss_distance_X_float_to_angle * 0.5;
						//Y轴脱靶量
						miss_distance_Y_float_LowpassBefore = miss_distance_Y / 100.0;
						miss_distance_Y_float = miss_distance_Y_float_LowpassBefore;
						
						miss_distance_Y_float_to_angle = miss_distance_Y_float * 73.5 * 0.000057295;
			//			miss_distance_Y_float_to_angle = miss_distance_Y_float_to_angle * 0.5;			
				}
		}
		RMS2_weihudu_X=RMS2_vol1(miss_distance_X_float); 
		RMS2_weihudu_Y=RMS2_vol2(miss_distance_Y_float);
}

//串口1读取MEMS陀螺惯导
/*
void read_inertial_navigation_slave(void)    
{
	if((TEST_ReadDataFlag&0x0001) == 0x0001)
	{
		u8 usart_1_num = 0;
		FPGA_READ_TEST_ADDR = PFPGA_WRITE_USART1;
		//接收MEMS惯导数据
		for(usart_1_num=0; usart_1_num<30; usart_1_num++)
		{
	    	read_usart1[usart_1_num]=*(uint32_t*)(FPGA_READ_TEST_ADDR);
			FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
		}
		if(read_usart1[0] == 0x80 && read_usart1[29] == 0xFF)//判断MEMS惯导帧头
		{
			ReadImuData();
	        
            //俯仰速度                                    //越大越往下 0.085
			X_axis_palstance_float = X_axis_palstance_float - X_speed_off_set;  //X轴角速度偏置补偿
			X_slave_1ms_distance = X_axis_palstance_float * -0.001;             //镜子在1ms内走的路程，取反
			X_slave_1ms_distance = X_slave_1ms_distance * 0.5;                  //镜子动θ，光动2θ
			//跟踪脱靶量时为负
			
			//方位轴不乘0.5                               //越大越往左
			Z_axis_palstance_float = Z_axis_palstance_float + Z_speed_off_set;   //Y轴角速度偏置补偿
			Z_slave_1ms_distance = Z_axis_palstance_float * 0.001;
			//跟踪脱靶量时为正
		}
	}
}
*/
void ReadImuData(void)
{
	yaw_angle_float_slave = small_navigation_calculate(YAW_ANGLE,ANGLE_GAIN);     //   -->Z轴角速度
	roll_angle_float_slave = small_navigation_calculate(ROLL_ANGLE,ANGLE_GAIN);   //   -->Y轴角速度
	pitch_angle_float_slave = small_navigation_calculate(PITCH_ANGLE,ANGLE_GAIN); //
	
	X_axis_palstance_float = small_navigation_calculate(PITCH_GERO,GERO_GAIN);
	Y_axis_palstance_float = small_navigation_calculate(ROLL_GERO,GERO_GAIN);	  //补偿+0.039		
	Z_axis_palstance_float = small_navigation_calculate(YAW_GERO,GERO_GAIN);      //补偿+0.019
}
float small_navigation_calculate(u8 index,float gain)
{
	u32 imid0 = 0;
	u32 imid1 = 0;
	u32 imid2 = 0;
	u32 imid3 = 0;
	float small_navigation_value = 0;
	
	imid0 = (read_usart1[index])&0x7F; 
	imid1 = (read_usart1[index+1])&0x7F; 
	imid2 = (read_usart1[index+2])&0x3F;
	imid3 = (imid2<<14)|(imid1<<7)|imid0;
	if(imid3 & 0x00080000)
	{
		imid3 = imid3|0xFFF00000;
	}
	small_navigation_value = (float)((int)imid3 * gain);
	
	return small_navigation_value;
}

unsigned int ltou_true = 0;
unsigned int lwei_true = 0;
unsigned int lhe_true = 0;
int zhenjishu_last = 0;

unsigned int Gyro_InterruptCnt1 = 0;
unsigned int Gyro_InterruptCnt2 = 0;
unsigned int Gyro_InterruptCnt3 = 0;
unsigned int Gyro_InterruptCnt4 = 0;
unsigned int Gyro_InterruptTrue = 0;
float x1_axis_velocity_float = 0;
float y1_axis_velocity_float = 0;
float z1_axis_velocity_float = 0;
u32 count_Test_1 = 0;
u32 count_Test_2 = 0;
float x_axis_velocity_1msdistance_sum1 = 0;

//串口5读取光栅光纤陀螺惯导(460800)
void read_inertial_navigation(void)    
{
		unsigned char usart_2_num = 0;
		unsigned char sum = 0;

//		if((TEST_ReadDataFlag&0x0020) == 0x0020)	//串口6
		if((TuoBa_fLAG&0x0010) == 0x0010)		//串口5  , 正确的
		{
				count_Test_2++;
				Gyro_InterruptCnt1++;			//进入中断的总次数

//				GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
				GPIO_ToggleBits(GPIOF,GPIO_Pin_11);
				GuangxianGuandao_Flag = 1;
				u8 usart_2_num = 0;
				FPGA_READ_TEST_ADDR = PFPGA_WRITE_USART2;
				for(usart_2_num=0; usart_2_num<33; usart_2_num++)
				{
						read_usart2[usart_2_num] = *(uint32_t*)(FPGA_READ_TEST_ADDR);
						FPGA_READ_TEST_ADDR=FPGA_READ_TEST_ADDR+2;
				}
				
				sum = 0;
				for(usart_2_num = 4; usart_2_num < 31; usart_2_num++)
				{
						sum = sum ^ read_usart2[usart_2_num];
				}
				if(read_usart2[0] == 0xEB && read_usart2[1] == 0x90)
				{
						ltou_true++;						//针头正确
				}
				else
				{
						Gyro_InterruptCnt2++;		//针头错误
				}
				if(read_usart2[32] == 0xBB)
				{
						lwei_true++;						//针尾正确
				}
				else
				{
						Gyro_InterruptCnt3++;		//针尾错误
				}
				if(sum == read_usart2[31])
				{
						lhe_true++;							//校验和正确
				}
				else
				{
						Gyro_InterruptCnt4++;		//校验和错误
				}
			
				//姿态角1绕Z转，0-360
				//姿态角2绕X转，正负180
				//姿态角3绕Y转，正负180			
				if(read_usart2[0] == 0xEB && read_usart2[1] == 0x90 && read_usart2[32] == 0xBB)
				{
						//速度		//俯仰速度	
						Gyro_InterruptTrue++;
						x_axis_velocity = read_usart2[4] + (read_usart2[5] << 8) + (read_usart2[6] << 16);
						x_axis_velocity_supplement = get_optical_navigation(x_axis_velocity);
						x_axis_velocity_float_ly = (int)x_axis_velocity_supplement * 0.0001;
						x_axis_velocity_float = x_axis_velocity_float_ly;                     //调转方向后的横滚速度  SpeedX
			//		x_axis_velocity_float = (int)x_axis_velocity_supplement * 0.0001;
						x_axis_velocity_float = x_axis_velocity_float - FY_OFSET;
						
						//横滚速度
						y_axis_velocity = read_usart2[7] + (read_usart2[8] << 8) + (read_usart2[9] << 16);
						y_axis_velocity_supplement = get_optical_navigation(y_axis_velocity);
						y_axis_velocity_float_ly = (int)y_axis_velocity_supplement * 0.0001;
						y_axis_velocity_float = y_axis_velocity_float_ly;                    //调转方向后的俯仰速度   SpeedY
			//		y_axis_velocity_float = (int)y_axis_velocity_supplement * 0.0001;
						
						//偏航速度    方位
						z_axis_velocity = read_usart2[10] + (read_usart2[11] << 8) + (read_usart2[12] << 16);
						z_axis_velocity_supplement = get_optical_navigation(z_axis_velocity);
						z_int = (int)z_axis_velocity_supplement;
						z_axis_velocity_float = (float)z_int * 0.0001;															//SpeedZ
			//			z_axis_velocity_float = (int)z_axis_velocity_supplement * 0.0001;
						z_axis_velocity_float = z_axis_velocity_float - FW_OFSET;
						
//						z_axis_velocity_float = 0;
						
						//加速度			
						x_axis_accelerated = read_usart2[13] + (read_usart2[14] << 8) + (read_usart2[15] << 16);
						x_axis_accelerated_supplement = get_optical_navigation(x_axis_accelerated);
						x_axis_accelerated_float = (int)x_axis_accelerated_supplement * 0.0001;	          //AccX
						
						y_axis_accelerated = read_usart2[16] + (read_usart2[17] << 8) + (read_usart2[18] << 16);
						y_axis_accelerated_supplement = get_optical_navigation(y_axis_accelerated);
						y_axis_accelerated_float = (int)y_axis_accelerated_supplement * 0.0001;	          //AccY
						
						z_axis_accelerated = read_usart2[19] + (read_usart2[20] << 8) + (read_usart2[21] << 16);
						z_axis_accelerated_supplement = get_optical_navigation(z_axis_accelerated);
						z_axis_accelerated_float = (int)z_axis_accelerated_supplement * 0.0001;						//AccZ
						
						//姿态			
						yaw_attitude = read_usart2[22] + (read_usart2[23] << 8) + (read_usart2[24] << 16);
						yaw_attitude_float_ly = yaw_attitude * 0.001;																			//AttYaw
//						if(yaw_attitude_float_ly > 300)
//						{
//								yaw_attitude_float_ly = yaw_attitude_float_ly - 360;
//						}                                                                             //方位
//						yaw_attitude_float_ly = yaw_attitude_float_ly + yaw_offset_val;///补偿
			//		yaw_attitude_float = yaw_attitude_float_ly;
						
						pitch_attitude = read_usart2[25] + (read_usart2[26] << 8) + (read_usart2[27] << 16);
						pitch_attitude_supplement = get_optical_navigation(pitch_attitude);
						pitch_attitude_float_ly = (int)pitch_attitude_supplement * 0.001;
						pitch_attitude_float_ly = pitch_attitude_float_ly + pitch_offset_val;					 //AttPitch
											
//						roll_attitude_before = pitch_attitude_float_ly;
			//		pitch_attitude_float = (int)pitch_attitude_supplement * 0.001;                 //俯仰
			//		pitch_attitude_float = pitch_attitude_float + pitch_offset_val;
						
						roll_attitude = read_usart2[28] + (read_usart2[29] << 8) + (read_usart2[30] << 16);
						roll_attitude_supplement = get_optical_navigation(roll_attitude);
						roll_attitude_float_ly = (int)roll_attitude_supplement * 0.001;
						roll_attitude_float_ly = roll_attitude_float_ly + roll_offset_val; 					 //AttRoll
						
//						pitch_attitude_before = roll_attitude_float_ly;
				
						//////////////////惯导坐标转换////////////////////////////
						
						//roll_attitude_float = sinf(GD_angle * 3.1415 / 180) * pitch_attitude_before + cosf(GD_angle * 3.1415 / 180)* roll_attitude_before ;
						//pitch_attitude_float = cosf(GD_angle * 3.1415 / 180) * pitch_attitude_before - sinf(GD_angle * 3.1415 / 180) * roll_attitude_before ;
//						roll_attitude_float = roll_attitude_before;
//						pitch_attitude_float = pitch_attitude_before;
						
//						if(ly_GD_flag == 1)
//						{
//							yaw_attitude_change = yaw_attitude_float_ly + (roll_attitude_float * sinf(pitch_attitude_float * 3.1415 / 180));
//						}
//						else if(ly_GD_flag == 2)
//						{
//							yaw_attitude_change = yaw_attitude_float_ly - (roll_attitude_float * sinf(pitch_attitude_float * 3.1415 / 180));
//						}
//						yaw_attitude_float = yaw_attitude_change;
//						
//									//俯仰1ms走的距离			
//						X_slave_1ms_distance = x_axis_velocity_float * (-0.001);             //镜子在1ms内走的路程，取反

//						if(system_mode == POINT_MODE || system_mode == 10 || system_mode == 14)
//						{
//							X_slave_1ms_distance_PT = X_slave_1ms_distance * 1;                  //镜子动θ，光动2θ
//						}
//						else
//						{
//							X_slave_1ms_distance_SV = X_slave_1ms_distance * 0.5;                  //镜子动θ，光动2θ
//						}
//						Z_slave_1ms_distance = z_axis_velocity_float * (-0.001f);
//						

//						if(z_axis_velocity_float > 0.025)
//						{
//							slave_CNT++;
//							watch_slave_Z[slave_CNT]= z_axis_velocity_float;
//							if(slave_CNT == 1000)
//							{
//								slave_CNT = 0;
//							}					
//						}
//						if(x_axis_velocity_float > 0.025)
//						{
//							slaveX_CNT++;
//							watch_slave_X[slaveX_CNT]= x_axis_velocity_float;
//							if(slaveX_CNT == 1000)
//							{
//								slaveX_CNT = 0;
//							}					
//						}
					
		//			slaveX_CNT++;
		//			watch_slave_X[slaveX_CNT]= z_axis_velocity_float;
				
		//			if(slaveX_CNT == 1002)
		//			{
		//			   slaveX_CNT = 1001;
		//			}
					
//					x_axis_velocity_1msdistance = x_axis_velocity_float * 0.001;
//					x_axis_velocity_1msdistance_sum = x_axis_velocity_1msdistance_sum + x_axis_velocity_1msdistance;
//					y_axis_velocity_1msdistance = y_axis_velocity_float * 0.001;
//					y_axis_velocity_1msdistance_sum = y_axis_velocity_1msdistance_sum + y_axis_velocity_1msdistance;
//		            z_axis_velocity_1msdistance = z_axis_velocity_float * 0.001;
//					z_axis_velocity_1msdistance_sum = z_axis_velocity_1msdistance_sum + z_axis_velocity_1msdistance;
				}	
		}
		else
		{
			GuangxianGuandao_Flag = 2;
		}
}
u32 get_optical_navigation(u32 threebit)
{
	u32 supplement = 0;
	if(threebit & 0x00800000)
	{
		supplement = threebit|0xFF000000;
	}
	else
	{
		supplement = threebit;
	}
	return supplement;
		
}

//电机驱动函数
void motor_move(float FW_location,float FY_location)
{
	//俯仰程序限位
	int FW_location_int = FW_location * 186413;
	int FY_location_int = FY_location * 186413;
	if(FY_location_int > 62448355)    //限位（335--270） 实际(337--268)     186413
	{
		FY_location_int = 62448355;
	}
	if(FY_location_int < 50331510)
	{
		FY_location_int = 50331510;
	}
	//方位程序限位
	wPA_FY(0x37f,FY_location_int);
	wPA_FW(0x37f,FW_location_int);
	wBG_FY(0x37f);	
	wBG_FW(0x37f);
}

//读取上位机指令函数
void read_master_control()      
{
	if((TEST_ReadDataFlag&0x0008) == 0x0008)           //422串口标志位
//	if((TEST_ReadDataFlag&0x002000) == 0x002000)         //LVDS标志位
	{
		Send++;
		AA_Com_Read();
	}
}
///////////////
//读取机上惯导函数
void read_jishangguandao()      
{
	//if((TEST_ReadDataFlag&0x0020) == 0x0020)           //422串口标志位
	if((TEST_ReadDataFlag&0x001000) == 0x001000)         //LVDS标志位
	{
		//Send++;
		AB_Com_Read();
	}
}
/////////

//回零
void reset_zero(void)
{
				FY_location_loop_open_flag = 0;//打开位置闭环标志位  FY_location_loop_open_flag = 0
				FW_location_loop_open_flag = 0;//FW_location_loop_open_flag = 0
				FW_light_loop_open_flag = 0;		//FW_light_loop_open_flag = 0
				FY_light_loop_open_flag = 0;		//光闭环
				FW_tuoluo_loop_open_flag = 1;
				FY_tuoluo_loop_open_flag = 1;
			//打开PID校正标志位 。校正模式选择：FW_location_para=1时，只有K校正；为2时，一阶校正；为3时，二阶校正；为4时，PID校正
//				FW_location_para = 4;  //FW_location_para_set；FW_location_para = 0;
//				FY_location_para = 4;  //FY_location_para_set；FY_location_para =
				FW_tuoluo_para = 4;
				FY_tuoluo_para = 4;
				FW_location_set = FW_zero_degree_zheng;//FW_zero_degree_zheng=361; 
				FY_location_set = FY_zero_degree;				
}

void power_down(void)
{
		wMO_FW(0x37f,0);//写励磁
		wMO_FY(0x37f,0);
}

void space_scan(void)
{
//	step_sin_value = 5*sin(step_sin_cnt / 5000.0 * 2 * 3.1415926);
//	step_sin_cnt++;
//	if(step_sin_cnt == 5000)
//	{
//		step_sin_cnt = 0;
//	}

//	Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;  //Z方位
//	X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;
//	
//	
//	
////	//赵老师公式起   step_design_arry_prepass[diaoyong_cnt]
////	//陀螺的FW_relative_rad = (Z_slave_1ms_distance_sum + step_design_arry_prepass[diaoyong_cnt]) * 3.1415926 / 180.0;  
////	FW_relative_rad = (Z_slave_1ms_distance_sum + step_design_arry_prepass[diaoyong_cnt]) * 3.1415926 / 180.0;
////	FY_relative_rad = 0 * 3.1415926 / 180.0;
////		
////	matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad) + 0.965925*sin(FY_relative_rad);
////	matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
////	matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
////		
////	FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 ;   //轴1.2189 --> 空1.1774
////	FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926 + 15.0) * 0.5;
////	//赵老师公式终
//	
////	GPIO_SetBits(GPIOE,GPIO_Pin_2);
////	r = -0.15 * pi /180.0;
////	p = X_slave_1ms_distance_sum * pi /180.0;
////	y = (Z_slave_1ms_distance_sum + step_design_arry_prepass[diaoyong_cnt]) * pi /180.0;
////	angle_fy = 0 * pi /180.0;
////	angle_fw = 0 * pi /180.0;
////	angle_qj = -105 * pi /180.0;
////	
////	x1 = (-cos(p)*sin(y)*cos(angle_qj)+(sin(r)*cos(y)+cos(r)*sin(p)*sin(y))*-sin(angle_qj))*cos(-angle_fy)*sin(-angle_fw) + (cos(p)*cos(y)*cos(angle_qj)+(sin(r)*sin(y)-cos(r)*sin(p)*cos(y))*-sin(angle_qj))*cos(-angle_fy)*cos(-angle_fw) + (sin(p)*cos(angle_qj)+cos(p)*cos(r)*-sin(angle_qj))*sin(-angle_fy);
////	y1 = -(cos(r)*cos(y)-sin(r)*sin(p)*sin(y))*cos(-angle_fy)*sin(-angle_fw) + -(cos(r)*sin(y)+sin(r)*sin(p)*cos(y))*cos(-angle_fy)*cos(-angle_fw) + sin(r)*cos(p)*sin(-angle_fy);
////	z1 = (-cos(p)*sin(y)*sin(angle_qj)+(sin(r)*cos(y)+cos(r)*sin(p)*sin(y))*cos(angle_qj))*cos(-angle_fy)*sin(-angle_fw) + (cos(p)*cos(y)*sin(angle_qj)+(sin(r)*sin(y)-cos(r)*sin(p)*cos(y))*cos(angle_qj))*cos(-angle_fy)*cos(-angle_fw) + (sin(p)*sin(angle_qj)+cos(p)*cos(r)*cos(angle_qj))*sin(-angle_fy);
////	
////	FW_buchang_degree = atan(y1/z1)*180.0/pi;
////	FY_buchang_degree = (atan(x1/sqrt(y1*y1+z1*z1))*180/pi+15)/2.0;
//////	GPIO_ResetBits(GPIOE,GPIO_Pin_2);//GPIOF9,F10设置高，灯灭
////	
////	//FW_location_set = FW_zero_degree_zheng - Z_slave_1ms_distance_sum;	
////	//FW_location_set = FW_zero_degree_zheng - FW_buchang_degree;	
////	//FY_location_set = FY_zero_degree - FY_buchang_degree;
////					
////    FW_location_set = FW_zero_degree_zheng - FW_buchang_degree; //FW_buchang_degree内含阶跃
////	//FW_location_set = 201 + step_design_arry_prepass[diaoyong_cnt] + step_sin_value;  //step_sin_value
////	FW_location_actual_set = 201 + step_design_arry[diaoyong_cnt] - FW_buchang_degree; 
////	FY_location_set = FY_zero_degree - FY_buchang_degree;
//	
//	//FY_location_set = FY_zero_degree - zhengfu * FY_buchang_degree + X_slave_1ms_distance_sum;

////改Kp,记初始脱靶量，累计求平均误差	
//	if(diaoyong_cnt == 1)
//	{
////		GPIO_SetBits(GPIOE,GPIO_Pin_2);//计时电平上拉
//		FW_location_kp = FW_location_kp_step;    //阶跃阶段Kp=130
//		//记阶跃初始脱靶量1
//		//miss_distance_X_float_start1 = miss_distance_X_float;
//	}
//	else if(diaoyong_cnt == open_camera_value)                
//	{
////		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//计时电平下拉
//	}
//	else if(diaoyong_cnt > open_camera_value && diaoyong_cnt< 110)  //稳定拍照时间
//	{
//		FW_rms_cnt++;
//		FW_rms[FW_rms_cnt] = FW_location_loop_error;                //编码器误差存入数组
//		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;               //误差累加
//		
//		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;         //前置滤波前编码器误差存入数组
//		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum; //前置滤波前编码器误差累加
//	}
//	
//	else if(diaoyong_cnt == 110)                         //计时电平上拉       
//	{
////		GPIO_SetBits(GPIOE,GPIO_Pin_2);
//		FW_rms_ave = FW_rms_sum * 0.025;//40个点                //编码器误差求均值
//		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;  //前置滤波前编码器误差求均值
//		//FW_rms_ave = FW_rms[1];
//		//FW_actual_rms_ave = FW_actual_rms[1];
//		
//		FW_rms_sum = 0; 
//		FW_actual_rms_sum = 0;
//		FW_rms_cnt = 0;
//		FW_location_kp = FW_location_kp_step;   //阶跃阶段Kp=130
//		//记阶跃初始脱靶量2
//		//miss_distance_X_float_start2 = miss_distance_X_float;
//	}
//	else if(diaoyong_cnt == (110 + open_camera_value))   
//	{
////		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//计时电平下拉 
//	}
//	else if(diaoyong_cnt > (110 + open_camera_value) && diaoyong_cnt< 220)   //稳定拍照时间
//	{
//		FW_rms_cnt++;
//		FW_rms[FW_rms_cnt] = FW_location_loop_error;     //编码器误差存入数组
//		FW_rms_sum = FW_rms[FW_rms_cnt] + FW_rms_sum;    //误差累加
//		
//		FW_actual_rms[FW_rms_cnt] = FW_location_loop_actual_error;
//		FW_actual_rms_sum = FW_actual_rms[FW_rms_cnt] + FW_actual_rms_sum;
//	}
//	else if(diaoyong_cnt == 220)                      //清零
//	{
//		diaoyong_cnt = 220;                 //需要清零
//	    FW_rms_ave = FW_rms_sum * 0.025;
//		FW_actual_rms_ave = FW_actual_rms_sum * 0.025;
//		//FW_rms_ave = FW_rms[1];
//		//FW_actual_rms_ave = FW_actual_rms[1];	
//		FW_rms_sum = 0;
//	    FW_actual_rms_sum = 0;
//		FW_rms_cnt = 0;
//	}
//	
////稳定拍照阶段切换高开环增益
//	if(diaoyong_cnt > change_kp_value && diaoyong_cnt< 110)
//	{
//		FW_location_kp = FW_location_kp_stable; 
//	}
//    else if(diaoyong_cnt > (110 + change_kp_value) && diaoyong_cnt< 220)
//	{
//		FW_location_kp = FW_location_kp_stable; 
//	}
//	
//////稳定拍照时的脱靶量误差
////	if(diaoyong_cnt == 70)
////    {
////		miss_distance_X_float_end1 = miss_distance_X_float;  //endx存的是稳定后
////		//start_end_arry[]数组存的是脱靶量相对误差
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end1 - miss_distance_X_float_start1;
////		//对脱靶量相对误差转存，可进行操作
////		miss_distance_X_float_sub1 = start_end_arry[start_end_cnt];
////		start_end_cnt++;   //start_end_arry[]数组存200个数
////	}
////	else if(diaoyong_cnt == 80)
////	{
////		miss_distance_X_float_end2 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end2 - miss_distance_X_float_start1;
////		miss_distance_X_float_sub2 = start_end_arry[start_end_cnt];
////		start_end_cnt++;
////	}
////    else if(diaoyong_cnt == 90)
////	{
////		miss_distance_X_float_end3 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end3 - miss_distance_X_float_start1;
////		miss_distance_X_float_sub3 = start_end_arry[start_end_cnt];			
////		start_end_cnt++;
////	}
////    else if(diaoyong_cnt == 100)
////	{
////		miss_distance_X_float_end4 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end4 - miss_distance_X_float_start1;
////		miss_distance_X_float_sub4 = start_end_arry[start_end_cnt];		
////		start_end_cnt++;
////	}
////	else if(diaoyong_cnt == 110)
////	{
////		miss_distance_X_float_end5 = miss_distance_X_float;
////	    start_end_arry[start_end_cnt] = miss_distance_X_float_end5 - miss_distance_X_float_start1;
////		miss_distance_X_float_sub5 = start_end_arry[start_end_cnt];		
////		start_end_cnt++;
////		
////		//一次阶跃计算5个点，累加求和再求平均

////		//********
////        if(store_open_flag == 1)
////		{
////			if(miss_distance_X_float_ave > error_set_value || miss_distance_X_float_ave < -error_set_value)	
////			{
////				store0 = miss_distance_X_float_start1;
////				store1 = miss_distance_X_float_end1;
////				store2 = miss_distance_X_float_end2;
////				store3 = miss_distance_X_float_end3;
////				store4 = miss_distance_X_float_end4;
////			    store5 = miss_distance_X_float_end5;	
////                store_open_flag	= 0;				
////			}
////            		
////		}
////	}
////	
////	else if(diaoyong_cnt == 180)
////	{
////		miss_distance_X_float_end6 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end6 - miss_distance_X_float_start2;
////		miss_distance_X_float_sub6 = start_end_arry[start_end_cnt];		
////		start_end_cnt++;
////	}
////	else if(diaoyong_cnt == 190)
////	{
////		miss_distance_X_float_end7 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end7 - miss_distance_X_float_start2;
////		miss_distance_X_float_sub7 = start_end_arry[start_end_cnt];			
////		start_end_cnt++;
////	}
////	else if(diaoyong_cnt == 200)
////	{
////		miss_distance_X_float_end8 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end8 - miss_distance_X_float_start2;
////		miss_distance_X_float_sub8 = start_end_arry[start_end_cnt];			
////		start_end_cnt++;
////	}
////	else if(diaoyong_cnt == 210)
////	{
////		miss_distance_X_float_end9 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end9 - miss_distance_X_float_start2;
////		miss_distance_X_float_sub9 = start_end_arry[start_end_cnt];			
////		start_end_cnt++;
////	}
////	else if(diaoyong_cnt == 220)
////	{
////		miss_distance_X_float_end10 = miss_distance_X_float;
////		start_end_arry[start_end_cnt] = miss_distance_X_float_end10 - miss_distance_X_float_start2;
////		miss_distance_X_float_sub10 = start_end_arry[start_end_cnt];		
////		start_end_cnt++;
////		
////		diaoyong_cnt = 0;

////	}
////	
////	if(start_end_cnt == 201)
////	{
////		start_end_cnt = 0;  //start_end_arry[]数组存200个数，每5个存一组
////	}
//////稳定拍照时的脱靶量误差
//	X_slave_1ms_distance_sum1_2 = X_slave_1ms_distance_sum * 0.5;
//	
//	FW_step_chafen_speed = getFW_step_chafen_speed(global_FW_step_piror,Z_slave_1ms_distance_sum,1000);
//    FY_step_chafen_speed = getFY_step_chafen_speed(global_FY_step_piror,X_slave_1ms_distance_sum1_2,1000);
//	
//	FW_wending_qiankui_speed_lowpass = FsmLeadLag1(FW_step_chafen_speed,FSM_X,2);     //稳定速度前馈滤波
//	FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_step_chafen_speed,FSM_Y,2);
//	//FY_wending_qiankui_speed_lowpass = FsmLeadLag1(FY_chafen_speed,FSM_Y,2);        //稳定速度前馈滤波
//	//FY_chafen_speed = getFY_chafen_speed(global_FY_position_piror,X_slave_1ms_distance_sum,1000);
//	//DA12_out(FW_location_set,FW_location_set);
//	
//	
//	//前置滤波
////	FW_pre_lowpss_filter = FsmLeadLag1(FW_simulated_target_motion,FSM_X,0);
////	FY_pre_lowpss_filter = FsmLeadLag1(FY_simulated_target_motion,FSM_Y,0);
////	
////	FW_location_set = FW_pre_lowpss_filter;
////	FY_location_set = FY_pre_lowpss_filter;
}



/*
//void stabilization(void)
//{
//	if(stabilize_type_select == 1)  //稳定目标为惯导
//	{
//		if(stabilize_axis_select == 1)   //只稳定俯仰
//		{
//			//1ms走的路程已经取反了
//			X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;      //X轴角速度积分=俯仰
//			FY_location_set = FY_zero_degree + X_slave_1ms_distance_sum;
//			FW_location_set = FW_zero_degree_zheng;
//		}
//		else if(stabilize_axis_select == 2)    //只稳定方位
//		{
//			FY_location_set = FY_zero_degree;
//			Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;
////			//赵老师公式起
////			FW_relative_rad = (Z_slave_1ms_distance_sum - 1.188) * 3.1415926 / 180.0;   //1.108   1.267
////			FY_relative_rad = FY_relative_rad;
////				
////			matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad);
////			matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
////			matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
////				
////			FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 + 1.2298;   //轴1.2189 --> 空1.1774
////			FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926+15.0) / 2.0;
////			//赵老师公式终
//			FW_location_set = FW_zero_degree_zheng - Z_slave_1ms_distance_sum;	
//		}
//		else if(stabilize_axis_select == 3)  //稳定方位和俯仰
//		{
//			X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;      //X轴角速度积分=俯仰
//			FY_location_set = FY_zero_degree + X_slave_1ms_distance_sum;
//			
//			Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;
//			//赵老师公式起
//			FW_relative_rad = (Z_slave_1ms_distance_sum - 1.188) * 3.1415926 / 180.0;   //1.108   1.267
//			FY_relative_rad = FY_relative_rad;
//				
//			matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad);
//			matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
//			matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
//				
//			FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 + 1.2298;   //轴1.2189 --> 空1.1774
//			FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926+15.0) / 2.0;
//			//赵老师公式终
//			FW_location_set = FW_zero_degree_zheng - FW_buchang_degree;	
//		}
//		else    //稳定在回零位置 
//		{
//			FW_location_set = FW_zero_degree_zheng;
//			FY_location_set = FY_zero_degree;
//		}
//	}
//	else if(stabilize_type_select == 2)   //稳定目标位给定的正弦
//	{
//		//5°0.2Hz
//		sin_value = 5*sin(sin_cnt / 5000.0 * 2 * 3.1415926);
//		sin_cnt++;
//		if(sin_cnt == 5000)
//		{
//			sin_cnt = 0;
//		}
//		FW_location_set = FW_zero_degree_zheng + sin_value;
//		FY_location_set = FY_zero_degree + sin_value;

//		X_slave_1ms_distance_sum = sin_value;
//		Z_slave_1ms_distance_sum = sin_value;
//	}
//	else    //稳定在回零位置
//	{
//		FW_location_set = FW_zero_degree_zheng;
//		FY_location_set = FY_zero_degree;
//	}
//}

void stabilization(void)
{
	if(stabilize_type_select == 1)  //稳定目标为惯导
	{
		if(stabilize_axis_select == 1)   //只稳定俯仰
		{
			//1ms走的路程已经取反了
			X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;      //X轴角速度积分=俯仰
			FY_location_set = FY_zero_degree + X_slave_1ms_distance_sum;
			
			FW_location_set = FW_zero_degree_zheng;
		}
		else if(stabilize_axis_select == 2)    //只稳定方位
		{
			FY_location_set = FY_zero_degree;
			
			Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;
//			//赵老师公式起
//			FW_relative_rad = (Z_slave_1ms_distance_sum - 1.188) * 3.1415926 / 180.0;   //1.108   1.267
//			FY_relative_rad = FY_relative_rad;
//				
//			matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad);
//			matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
//			matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
//				
//			FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 + 1.2298;   //轴1.2189 --> 空1.1774
//			FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926+15.0) / 2.0;
//			//赵老师公式终
		    //赵老师公式起
			FW_relative_rad = Z_slave_1ms_distance_sum * 3.1415926 / 180.0;   //1.108   1.267
			FY_relative_rad = FY_relative_rad;
				
			matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad);
			matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
			matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
				
			FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 ;   //轴1.2189 --> 空1.1774
			FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926+15.0) / 2.0;
			//赵老师公式终
			//FW_location_set = FW_zero_degree_zheng - Z_slave_1ms_distance_sum;	
			FW_location_set = FW_zero_degree_zheng - FW_buchang_degree;	
		}
		else if(stabilize_axis_select == 3)  //稳定方位和俯仰
		{
			X_slave_1ms_distance_sum = X_slave_1ms_distance + X_slave_1ms_distance_sum;      //X轴角速度积分=俯仰
			FY_location_set = FY_zero_degree + X_slave_1ms_distance_sum;
			
			Z_slave_1ms_distance_sum = Z_slave_1ms_distance + Z_slave_1ms_distance_sum;
			//赵老师公式起
			FW_relative_rad = (Z_slave_1ms_distance_sum - 1.188) * 3.1415926 / 180.0;   //1.108   1.267
			FY_relative_rad = FY_relative_rad;
				
			matrix_a = -0.25881905 * cos(FY_relative_rad) * cos(FW_relative_rad);
			matrix_b = -1.0 * cos(FY_relative_rad) * sin(FW_relative_rad);
			matrix_c = -0.96592583 * cos(FY_relative_rad) * cos(FW_relative_rad) + (-0.25881905) * sin(FY_relative_rad);
				
			FW_buchang_degree = atan(matrix_b / matrix_c) * 180.0 / 3.1415926 + 1.2298;   //轴1.2189 --> 空1.1774
			FY_buchang_degree = (atan(matrix_a/sqrt(matrix_b*matrix_b+matrix_c*matrix_c))*180.0/3.1415926+15.0) / 2.0;
			//赵老师公式终
			FW_location_set = FW_zero_degree_zheng - FW_buchang_degree;	
		}
		else    //稳定在回零位置 
		{
			FW_location_set = FW_zero_degree_zheng;
			FY_location_set = FY_zero_degree;
		}
	}
	else if(stabilize_type_select == 2)   //稳定目标位给定的正弦
	{
		//5°0.2Hz
		sin_value = 5*sin(sin_cnt / 5000.0 * 2 * 3.1415926);
		sin_cnt++;
		if(sin_cnt == 5000)
		{
			sin_cnt = 0;
		}
		FW_location_set = FW_zero_degree_zheng + sin_value;
		FY_location_set = FY_zero_degree + sin_value;

		X_slave_1ms_distance_sum = sin_value;
		Z_slave_1ms_distance_sum = sin_value;
	}
	else    //稳定在回零位置
	{
		FW_location_set = FW_zero_degree_zheng;
		FY_location_set = FY_zero_degree;
	}
}
*/

void para_set(void)
{
	//每个周期重新计算一次双线性变换
	X0_fsmT1 = FW_pre_lvbo;       //阶跃前置滤波   
	Y0_fsmT1 = FY_pre_lvbo;
	
	X1_fsmT1 = FW_lowpass;        //速度环低通滤波
	Y1_fsmT1 = FY_lowpass;
	
	X2_fsmT1 = FW_wending_qiankui_lvbo;    //稳定前馈滤波
	Y2_fsmT1 = FY_wending_qiankui_lvbo;
	
	X4_fsmT1 = FW_track_qiankui_lvbo;      //跟踪前馈滤波
	Y4_fsmT1 = FY_track_qiankui_lvbo;
	
	X5_fsmTao1 = FW_wending_yijie_tao;     //稳定的一阶校正
	X5_fsmT1 = FW_wending_yijie_T;
	Y5_fsmTao1 = FY_wending_yijie_tao;      
	Y5_fsmT1 = FY_wending_yijie_T;  
	
	X3_fsmTao1 = FW_wending_erjie_tao;     //稳定的二阶校正
	X3_fsmTao2 = FW_wending_erjie_tao;
	X3_fsmT1 = FW_wending_erjie_T;
	X3_fsmT2 = FW_wending_erjie_T;
	Y3_fsmTao1 = FY_wending_erjie_tao;
	Y3_fsmTao2 = FY_wending_erjie_tao;
	Y3_fsmT1 = FY_wending_erjie_T;
	Y3_fsmT2 = FY_wending_erjie_T;
	
	X1_fsmT1_servo = FW_lowpass_servo;     //10MS速度环低通滤波
	Y1_fsmT1_servo = FY_lowpass_servo;
	
	X2_fsmTao1_servo = X2_fsmTao1_servo;   //10MS光闭环一阶校正
	X2_fsmT1_servo = X2_fsmT1_servo;
	Y2_fsmTao1_servo = Y2_fsmTao1_servo;
	Y2_fsmT1_servo = Y2_fsmT1_servo;

	X7_fsmTao1_servo = FW_track_erjie_tao_servo;   //10MS光闭环二阶校正
	X7_fsmTao2_servo = FW_track_erjie_tao_servo;
	X7_fsmT1_servo = FW_track_erjie_T_servo;
	X7_fsmT2_servo = FW_track_erjie_T_servo;
	Y7_fsmTao1_servo = FY_track_erjie_tao_servo;
	Y7_fsmTao2_servo = FY_track_erjie_tao_servo;
	Y7_fsmT1_servo = FY_track_erjie_T_servo;
	Y7_fsmT2_servo = FY_track_erjie_T_servo;
	
//	AskfsmFilterPara();         //双线性变换函数
//	AskfsmFilterPara_servo();
	
	accelerate_limit_1ms = accelerate_limit * 0.001;
}






float Ary4ToFloat(char c1, char c2, char c3, char c4)     //将4个字节转换成单精度浮点
{    
	union FloatToArray fta;
	fta.ary[0] = c1;    
	fta.ary[1] = c2;    
	fta.ary[2] = c3;    
	fta.ary[3] = c4;    
	return fta.f;
}
float getFY_chafen_speed(float piror,float current,float time)
{
	float chafenspeed;
	chafenspeed = (current - piror) * time;
	global_FY_position_piror = current;
	return chafenspeed;
}
float getFW_chafen_speed(float piror,float current,float time)
{
	float chafenspeed;
	chafenspeed = (current - piror) * time;
	global_FW_position_piror = current;
	return chafenspeed;
}
	
	
float aa_test;
//采集命令应答
void CaiJiMingLing_YingDa()
{	
  u8 i=0,Sum2=0;
	u32 Sum=0;
	///应答//
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4c;
	AA_TXData[5]=63;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
//if(EXTI2_CNT<=30)aa_test=aa_test+0.02;
//	else if(EXTI2_CNT<=60)aa_test=aa_test-0.02;
//	if(EXTI2_CNT>60)EXTI2_CNT=0;
//	TX_Coarse_Azimuth=FW_encoder_degrees;
	//粗跟踪伺服方位角度
//	if(TX_Coarse_Azimuth>360.0)TX_Coarse_Azimuth=0;
//	TX_Coarse_Azimuth=TX_Coarse_Azimuth+0.001;
	Temp_Data.f=FW_encoder_degrees;
//	Temp_Data.f=aa_test;
	AA_TXData[10]=Temp_Data.ary[0];
	AA_TXData[11]=Temp_Data.ary[1];
	AA_TXData[12]=Temp_Data.ary[2];
	AA_TXData[13]=Temp_Data.ary[3];

	Temp_Data.f=FY_encoder_degrees;
	//Temp_Data.f=FY_buchang_degree;
	
//	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪伺服俯仰角度
  //Temp_Data.f=aa_test;
	AA_TXData[14]=Temp_Data.ary[0];
	AA_TXData[15]=Temp_Data.ary[1];
	AA_TXData[16]=Temp_Data.ary[2];
	AA_TXData[17]=Temp_Data.ary[3];
//姿态图1-偏航
  TX_Ins_Attitude_FW = yaw_attitude_float;    //偏航  yaw_attitude_float   z_axis_velocity_1msdistance_sum
	Temp_Data.f=TX_Ins_Attitude_FW;
	AA_TXData[18]=Temp_Data.ary[0];
	AA_TXData[19]=Temp_Data.ary[1];
	AA_TXData[20]=Temp_Data.ary[2];
	AA_TXData[21]=Temp_Data.ary[3];
//姿态图2-俯仰	
  TX_Ins_Attitude_FY = pitch_attitude_float;    //俯仰  pitch_attitude_float   x_axis_velocity_1msdistance_sum
	Temp_Data.f=TX_Ins_Attitude_FY;
	AA_TXData[22]=Temp_Data.ary[0];
	AA_TXData[23]=Temp_Data.ary[1];
	AA_TXData[24]=Temp_Data.ary[2];
	AA_TXData[25]=Temp_Data.ary[3];
//姿态图3-横滚   
  TX_Ins_Attitude_Roll = roll_attitude_float;     //横滚  roll_attitude_float  y_axis_velocity_1msdistance_sum
	Temp_Data.f=TX_Ins_Attitude_Roll;
	AA_TXData[26]=Temp_Data.ary[0];
	AA_TXData[27]=Temp_Data.ary[1];
	AA_TXData[28]=Temp_Data.ary[2];
	AA_TXData[29]=Temp_Data.ary[3];
//速度图1
	TX_Ins_Speed_FW=z_axis_velocity_float;                //偏航角速度
	Temp_Data.f=TX_Ins_Speed_FW;
	AA_TXData[30]=Temp_Data.ary[0];
	AA_TXData[31]=Temp_Data.ary[1];
	AA_TXData[32]=Temp_Data.ary[2];
	AA_TXData[33]=Temp_Data.ary[3];
//速度图2
	TX_Ins_Speed_FY=x_axis_velocity_float;                  //俯仰角速度
	Temp_Data.f=TX_Ins_Speed_FY;
	AA_TXData[34]=Temp_Data.ary[0];
	AA_TXData[35]=Temp_Data.ary[1];
	AA_TXData[36]=Temp_Data.ary[2];
	AA_TXData[37]=Temp_Data.ary[3];
//速度图3
	TX_Ins_Speed_Roll=y_axis_velocity_float;          //横滚角速度
	Temp_Data.f=TX_Ins_Speed_Roll;
	AA_TXData[38]=Temp_Data.ary[0];
	AA_TXData[39]=Temp_Data.ary[1];
	AA_TXData[40]=Temp_Data.ary[2];
	AA_TXData[41]=Temp_Data.ary[3];

	TX_Encoder_FW=FW_encoder_degrees;                   //★方位编码器 
	Temp_Data.f=TX_Encoder_FW;
	AA_TXData[42]=Temp_Data.ary[0];
	AA_TXData[43]=Temp_Data.ary[1];
	AA_TXData[44]=Temp_Data.ary[2];
	AA_TXData[45]=Temp_Data.ary[3];

	TX_Encoder_FY=FY_encoder_degrees;                    //★俯仰编码器
	Temp_Data.f=TX_Encoder_FY;
	AA_TXData[46]=Temp_Data.ary[0];
	AA_TXData[47]=Temp_Data.ary[1];
	AA_TXData[48]=Temp_Data.ary[2];
	AA_TXData[49]=Temp_Data.ary[3];

// FW_miss_distance_sub  
// FY_miss_distance_sub
// miss_distance_X_float
// miss_distance_Y_float
// TX_Miss_FW=sin_value;    正弦检验RMS值
// pitch_angle_float_slave
// IMU_bit_float
  
//	TX_Miss_FW = FW_Miss_distance;            //实际用
  TX_Miss_FW = FW_send_array[FW_send_array_select];    //测试用
//  TX_Miss_FW=FY_miss_distance_sub;                   //★方位脱靶量          
	Temp_Data.f=TX_Miss_FW;       

	AA_TXData[50]=Temp_Data.ary[0];
	AA_TXData[51]=Temp_Data.ary[1];
	AA_TXData[52]=Temp_Data.ary[2];
	AA_TXData[53]=Temp_Data.ary[3];

//  TX_Miss_FY = FY_Miss_distance;               //实际用
    TX_Miss_FY = FY_send_array[FY_send_array_select];    //测试用
//	TX_Miss_FY=miss_distance_Y_float;                   //★俯仰脱靶量
	Temp_Data.f=TX_Miss_FY;     
	AA_TXData[54]=Temp_Data.ary[0];
	AA_TXData[55]=Temp_Data.ary[1];
	AA_TXData[56]=Temp_Data.ary[2];
	AA_TXData[57]=Temp_Data.ary[3];

	
//	read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)	
//	{
		AA_TXData[58] = Read_JiaoYanHouShuJu[15];                //快反射镜方位角度
		AA_TXData[59] = Read_JiaoYanHouShuJu[16];
		AA_TXData[60] = Read_JiaoYanHouShuJu[17];
		AA_TXData[61] = Read_JiaoYanHouShuJu[18];
		
		AA_TXData[62] = Read_JiaoYanHouShuJu[19];                //快反射镜俯仰角度
		AA_TXData[63] = Read_JiaoYanHouShuJu[20];
		AA_TXData[64] = Read_JiaoYanHouShuJu[21];
		AA_TXData[65] = Read_JiaoYanHouShuJu[22];
		
//		Read_JiaoYan_Flag =0;
//	}

	
//	TX_Fast_Reflection_Mirror_FW=13;                       //快反射镜方位角度
//	Temp_Data.f=TX_Fast_Reflection_Mirror_FW;
//	AA_TXData[58]=Temp_Data.ary[0];
//	AA_TXData[59]=Temp_Data.ary[1];
//	AA_TXData[60]=Temp_Data.ary[2];
//	AA_TXData[61]=Temp_Data.ary[3];

	
//	TX_Fast_Reflection_Mirror_FY=15;                       //快反射镜俯仰角度
//	Temp_Data.f=TX_Fast_Reflection_Mirror_FY;
//	AA_TXData[62]=Temp_Data.ary[0];
//	AA_TXData[63]=Temp_Data.ary[1];
//	AA_TXData[64]=Temp_Data.ary[2];
//	AA_TXData[65]=Temp_Data.ary[3];
//	if(Work_Mode==0x03)
//	{ 

//		TX_Servo_Steady_State=0xcc;		//跟踪过程中，此状态需要返回  0xaa稳定0xcc运动中
//	}
//	else
//	{
//		TX_Servo_Steady_State=0xAA;        //跟踪过程中，此状态需要返回  0xaa稳定0xcc运动中
//	}
	AA_TXData[66]=TX_Servo_Steady_State;
	AA_TXData[67]=0XAA;//备份
	AA_TXData[68]=0XAA;//备份
	for(i=6;i<(69);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[69]=Sum2;//校验
	AA_TXData[70]=0X5A;
	AA_TXData[71]=0XFE;
	AA_TX_Len=72;	
	AA_Com_Write();
	CommandOld=CommandNew;
	

	Watch_CaiJi[watchi] = FW_encoder_degrees;
	watchi++;
	if(watchi == 202)
	{
		watchi = 201;
	}
	
	CaiJi_CNT++;
//	Ma_CaiJi[Ma_watch] = AA_TXData[10];
//	Ma_CaiJi[Ma_watch+1] = AA_TXData[11];
//	Ma_CaiJi[Ma_watch+2] = AA_TXData[12];
//	Ma_CaiJi[Ma_watch+3] = AA_TXData[13];

	
	
//	Ma_watch = Ma_watch + 4;
////	TX_watch = TX_watch + 4;
//	
//	if(Ma_watch == 244)
//	{
//		watchi = 240;
//	}
//	if(TX_watch == 62)
//	{
//		watchi = 61;
//	}
	
	
}

//自检应答
void ZiJian_YingDa()
{
	u8 i=0;
	u8 Sum2=0;             //和校验低8位校验字节（补成零的）
	u32 Sum=0;	           //和校验总和
	AA_TXData[0]=0X5A;    
	AA_TXData[1]=0X54;     //帧头
	AA_TXData[2]=0X17;     //发送端为伺服设备
	AA_TXData[3]=0X10;     //接收端为主控设备
	AA_TXData[4]=0X40;     //自检应答
	AA_TXData[5]=11;       //数据长度
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;	
//	AA_TXData[10]=0;//32--39bit故障位赋值            ☆故障代码
	
	if(EE1_fw==0)    //方位轴编码器工作状态      0         
	{
		AA_TXData[10] = AA_TXData[10]&0XFE;     
	}
	else
	{
	  AA_TXData[10] = AA_TXData[10]|0X01;     
	}
//	AA_TXData[11]=0;//40--41bit故障位赋值            ☆故障代码
 //	if(FY_encoder_Flag==1)
//	EE1_fy
	if(EE1_fy==0)     //俯仰轴编码器工作状态     1
	{
		AA_TXData[10] = AA_TXData[10]&0XFD;     
	}
	else
	{
	  AA_TXData[10] = AA_TXData[10]|0X02;     
	}	
//	Temp_Data.f=1;///方位驱动器电流            ☆方位电流   
//	AA_TXData[12]=1;// 
	if(GuangxianGuandao_Flag==1)  //光纤惯导工作状态    2
	{
		AA_TXData[10] = AA_TXData[10]&0XFB; 
	}
	else
	{
		AA_TXData[10] = AA_TXData[10]|0X04; 		
	}
	
//	AA_TXData[14]=Temp_Data.ary[1];//
//	AA_TXData[15]=Temp_Data.ary[2];//
//	AA_TXData[16]=Temp_Data.ary[3];//

//	Temp_Data.f=2;//俯仰驱动器电流             ☆俯仰电流  
//	AA_TXData[13]=2;//
	//MF_fy
//	if(FW_encoder_Flag==1)
	if(MF_fw==0)    //方位电机工作状态     3    改  原来是1
	{
		AA_TXData[10] = AA_TXData[10]&0XF7;   
	}
	else
	{
	  AA_TXData[10] = AA_TXData[10]|0X08;    
	}
	
//	AA_TXData[18]=Temp_Data.ary[1];//
//	AA_TXData[19]=Temp_Data.ary[2];//
//	AA_TXData[20]=Temp_Data.ary[3];//

//	Temp_Data.f=3;///方位驱动器温度            ☆方位驱动器温度
//	AA_TXData[14]=3;//
	
	if(MF_fy==0)     //俯仰电机工作状态     4   改   原来是1
	{
		AA_TXData[10] = AA_TXData[10]&0XEF;     
	}
	else
	{
	  AA_TXData[10] = AA_TXData[10]|0X10;      
	}	

	if(Read_JiaoYanHouShuJu[3] == 0x01)       //精跟踪方位电机状态         5      改  原来是0x00
	{
		AA_TXData[10] = AA_TXData[10]&0XDF; 			
	}
	else
	{
		AA_TXData[10] = AA_TXData[10]|0X20;
	}
	if(Read_JiaoYanHouShuJu[4] == 0x01)       //精跟踪俯仰电机状态         6     改   原来是0x00
	{
		AA_TXData[10] = AA_TXData[10]&0XBF; 			
	}
	else
	{
		AA_TXData[10] = AA_TXData[10]|0X40;
	}
	if(JingGenZong_Err == 0x00)       //信号检测板与精跟踪伺服板通信状态   7
	{
		AA_TXData[10] = AA_TXData[10]&0X7F; 			
	}
	else
	{
		AA_TXData[10] = AA_TXData[10]|0X80;
	}
	if(Read_JiaoYanHouShuJu[6] == 0x00)       //精跟踪振镜伺服板状态		 1    
	{
		AA_TXData[11] = AA_TXData[11]&0XFD; 			
	}
	else
	{
		AA_TXData[11] = AA_TXData[11]|0X02;
	}

//	AA_TXData[22]=Temp_Data.ary[1];//
//	AA_TXData[23]=Temp_Data.ary[2];//
//	AA_TXData[24]=Temp_Data.ary[3];//

//	Temp_Data.f=4;//俯仰驱动器温度             ☆俯仰驱动器温度
//	AA_TXData[15]=4;            //精跟踪方位电机状态
//	AA_TXData[26]=Temp_Data.ary[1];//
//	AA_TXData[27]=Temp_Data.ary[2];//
//	AA_TXData[28]=Temp_Data.ary[3];//			
//  AA_TXData[16]=0x00;       //精跟踪俯仰电机状态  
//	AA_TXData[17]=0x00;       //信号检测板与精跟踪伺服板通信状态
	AA_TXData[11]=AA_TXData[11]&0XFE;       //信号检测伺服板状态              0
//	AA_TXData[19]=0x00;       //精跟踪振镜伺服板状态

    DianLiu_1 = (int)(DianLiu_FW/0.5);
	DianLiu_2 = (int)(DianLiu_FY/0.5);
	AA_TXData[12]=DianLiu_1;       //方位驱动器电流	
    AA_TXData[13]=DianLiu_2;       //俯仰驱动器电流	
	AA_TXData[14]=(int) WD_FW;    //方位驱动器温度
    AA_TXData[15]=(int) WD_FY;    //俯仰驱动器温度
	
	if(system_mode == 13)    //外同步
	{
		AA_TXData[16]=0x01;
	}
	else if(system_mode == 7)   //跟踪
	{
		AA_TXData[16]=0x02;
	}
	else if(system_mode == 1)   //待机
	{
		AA_TXData[16]=0x03;
	}
	else if(system_mode == 14)   //人工引导
	{
		AA_TXData[16]=0x04;
	}
	if(Work_Mode==0x05)   //边扫边跟
	{
		AA_TXData[16]=0x05;
	}
	
//	AA_TXData[16]=0x00;       //备用	
	
	for(i=6;i<(17);i++)             //各数据字节累加（i比字节索引大1）
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[17]=Sum2;            //校验字节
	AA_TXData[18]=0X5A;            
	AA_TXData[19]=0XFE;            //帧尾
	AA_TX_Len=20;	               //发送字节长度，（比字节索引大1）	
	AA_Com_Write();
	CommandOld=CommandNew;
}

//状态查询模式应答
void ZhuangTaiChaXun_YingDa()
{
	STMFLASH_Read(FLASH_SAVE_ADDR,(u32*)CanShu_Read,38);
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A; //帧头
	AA_TXData[1]=0X54; //帧头
	AA_TXData[2]=0X17;//发送设备ID
	AA_TXData[3]=0X10;//接收设备ID
	AA_TXData[4]=0X42;       //状态查询
	AA_TXData[5]=51;///数据长度
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
	
	
	AA_TXData[10]=CanShu_Read[0];     //粗跟踪方位回零位置
	AA_TXData[11]=CanShu_Read[1];
	AA_TXData[12]=CanShu_Read[2];
	AA_TXData[13]=CanShu_Read[3];
	
	AA_TXData[14]=CanShu_Read[4];     //粗跟踪俯仰回零位置
	AA_TXData[15]=CanShu_Read[5];
	AA_TXData[16]=CanShu_Read[6];
	AA_TXData[17]=CanShu_Read[7];
	
	AA_TXData[18]=CanShu_Read[30];     //精跟踪方位回零位置
	AA_TXData[19]=CanShu_Read[31];
	AA_TXData[20]=CanShu_Read[32];
	AA_TXData[21]=CanShu_Read[33];
	
	AA_TXData[22]=CanShu_Read[34];     //精跟踪俯仰回零位置
	AA_TXData[23]=CanShu_Read[35];
	AA_TXData[24]=CanShu_Read[36];
	AA_TXData[25]=CanShu_Read[37];
//	switch(AA_Data[10])
//	{
////extern int32_t   SO_fw;
////extern int32_t   EE1_fw;
////extern int32_t   EC_fw;
////extern int32_t   MF_fw;
////extern int32_t   SR_fw;
//		case 1:
//		{
//			AA_TXData[26]=EE1_fw;     
//	    AA_TXData[27]=EE1_fw>>8;
//	    AA_TXData[28]=EE1_fw>>16;
//	    AA_TXData[29]=EE1_fw>>24;            
//	
//	    AA_TXData[30]=EE1_fy;     
//	    AA_TXData[31]=EE1_fy>>8;
//	    AA_TXData[32]=EE1_fy>>16;
//	    AA_TXData[33]=EE1_fy>>24;  
//			break;
//		}
//		case 2://自检
//		{
//			AA_TXData[26]=SO_fw;     
//	    AA_TXData[27]=SO_fw>>8;
//	    AA_TXData[28]=SO_fw>>16;
//	    AA_TXData[29]=SO_fw>>24;            
//	
//	    AA_TXData[30]=SO_fy;     
//	    AA_TXData[31]=SO_fy>>8;
//	    AA_TXData[32]=SO_fy>>16;
//	    AA_TXData[33]=SO_fy>>24;  
//			break;
//		}
//		case 3://自检
//		{
//			AA_TXData[26]=EC_fw;     
//	    AA_TXData[27]=EC_fw>>8;
//	    AA_TXData[28]=EC_fw>>16;
//	    AA_TXData[29]=EC_fw>>24;            
//	
//	    AA_TXData[30]=EC_fy;     
//	    AA_TXData[31]=EC_fy>>8;
//	    AA_TXData[32]=EC_fy>>16;
//	    AA_TXData[33]=EC_fy>>24;  
//			break;
//		}
//		case 4://自检
//		{
//			AA_TXData[26]=MF_fw;     
//	    AA_TXData[27]=MF_fw>>8;
//	    AA_TXData[28]=MF_fw>>16;
//	    AA_TXData[29]=MF_fw>>24;            
//	
//	    AA_TXData[30]=MF_fy;     
//	    AA_TXData[31]=MF_fy>>8;
//	    AA_TXData[32]=MF_fy>>16;
//	    AA_TXData[33]=MF_fy>>24;  
//			break;
//		}
//		case 5://自检
//		{
//			AA_TXData[26]=SR_fw;     
//	    AA_TXData[27]=SR_fw>>8;
//	    AA_TXData[28]=SR_fw>>16;
//	    AA_TXData[29]=SR_fw>>24;            
//	
//	    AA_TXData[30]=SR_fy;     
//	    AA_TXData[31]=SR_fy>>8;
//	    AA_TXData[32]=SR_fy>>16;
//	    AA_TXData[33]=SR_fy>>24;  
//			break;
//		}
//		default:
//		
//			break;
//		
//	}
	AA_TXData[26]=0;     
	AA_TXData[27]=0;
	AA_TXData[28]=0;
	AA_TXData[29]=0;            
	
	AA_TXData[30]=0;     
	AA_TXData[31]=0;
	AA_TXData[32]=0;
	AA_TXData[33]=0;  
	
	AA_TXData[34]=0;     
	AA_TXData[35]=0;
	AA_TXData[36]=0;
	AA_TXData[37]=0;  
	
	AA_TXData[38]=0;     
	AA_TXData[39]=0;
	AA_TXData[40]=0;
	AA_TXData[41]=0;  
	
	AA_TXData[42]=CanShu_Read[16];       //步进间隔时间
	AA_TXData[43]=CanShu_Read[17];  
	
	AA_TXData[44]=CanShu_Read[18];       //方位每次步进角度
	AA_TXData[45]=CanShu_Read[19];  
	
	AA_TXData[46]=CanShu_Read[20];       //俯仰每次步进角度
	AA_TXData[47]=CanShu_Read[21];  
	
	AA_TXData[48]=CanShu_Read[22];       //稳定时间
	AA_TXData[49]=CanShu_Read[23];  
	AA_TXData[50]=CanShu_Read[24];       
	AA_TXData[51]=CanShu_Read[25]; 
	
	AA_TXData[52]=CanShu_Read[26];       //扫描最大方位角1
	AA_TXData[53]=CanShu_Read[27];       //扫描最大方位角2
	
	AA_TXData[54]=CanShu_Read[28];       //扫描最大俯仰角1
	AA_TXData[55]=CanShu_Read[29];       //扫描最大俯仰角2
	AA_TXData[56]=0XAA;       //备份
	
	for(i=6;i<(57);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[57]=Sum2;      //校验字节
	AA_TXData[58]=0X5A;
	AA_TXData[59]=0XFE;
	AA_TX_Len=60;	
	AA_Com_Write();
	CommandOld=CommandNew;
}

//指向模式应答
void ZhiXiang_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	read_FY_encoder();
	read_FW_encoder();
	////应答//
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X44;
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒

	AA_TXData[10]=Point_Mode;//指向模式01仅粗跟踪02仅精跟踪03粗精跟踪04精跟踪校准补偿


	TX_Coarse_Azimuth=FW_encoder_degrees;          // ★
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪当前方位位置

	AA_TXData[11]=Temp_Data.ary[0];//粗跟踪方位
	AA_TXData[12]=Temp_Data.ary[1];//粗跟踪方位
	AA_TXData[13]=Temp_Data.ary[2];//粗跟踪方位
	AA_TXData[14]=Temp_Data.ary[3];//粗跟踪方位

	TX_Coarse_Pitch=FY_encoder_degrees;               // ★
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪当前俯仰位置

	AA_TXData[15]=Temp_Data.ary[0];//粗跟踪俯仰
	AA_TXData[16]=Temp_Data.ary[1];//粗跟踪俯仰
	AA_TXData[17]=Temp_Data.ary[2];//粗跟踪俯仰
	AA_TXData[18]=Temp_Data.ary[3];//粗跟踪俯仰
					
//	read_JingGenZong_Data();
//	JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[19]= Read_JiaoYanHouShuJu[15];     //精跟踪当前方位位置
		AA_TXData[20]= Read_JiaoYanHouShuJu[16];		
		AA_TXData[21]= Read_JiaoYanHouShuJu[17];	
		AA_TXData[22]= Read_JiaoYanHouShuJu[18];	
		
		AA_TXData[23]=Read_JiaoYanHouShuJu[19];      //精跟踪当前俯仰位置
		AA_TXData[24]=Read_JiaoYanHouShuJu[20];
		AA_TXData[25]=Read_JiaoYanHouShuJu[21];
		AA_TXData[26]=Read_JiaoYanHouShuJu[22];
//		Read_JiaoYan_Flag = 0;
//	}
//	TX_Precise_Azimuth=3;
//	Temp_Data.f=TX_Precise_Azimuth;//精跟踪初始方位位置

//	AA_TXData[19]=Temp_Data.ary[0];//精跟踪方位
//	AA_TXData[20]=Temp_Data.ary[1];//
//	AA_TXData[21]=Temp_Data.ary[2];//
//	AA_TXData[22]=Temp_Data.ary[3];//

//	TX_Precise_Pitch=4;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪初始俯仰位置

//	AA_TXData[23]=Temp_Data.ary[0];//精跟踪俯仰
//	AA_TXData[24]=Temp_Data.ary[1];//
//	AA_TXData[25]=Temp_Data.ary[2];//
//	AA_TXData[26]=Temp_Data.ary[3];//

	for(i=6;i<(27);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;//校验
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;		
}

//步进扫描模式应答
void BuJinSaoMiao_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X45;
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒

	TX_Coarse_Azimuth=FW_encoder_degrees;            
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪初始方位位置

	AA_TXData[10]=Temp_Data.ary[0];//粗跟踪方位
	AA_TXData[11]=Temp_Data.ary[1];//粗跟踪方位
	AA_TXData[12]=Temp_Data.ary[2];//粗跟踪方位
	AA_TXData[13]=Temp_Data.ary[3];//粗跟踪方位

	TX_Coarse_Pitch=FY_encoder_degrees;
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪初始俯仰位置

	AA_TXData[14]=Temp_Data.ary[0];//粗跟踪俯仰
	AA_TXData[15]=Temp_Data.ary[1];//粗跟踪俯仰
	AA_TXData[16]=Temp_Data.ary[2];//粗跟踪俯仰
	AA_TXData[17]=Temp_Data.ary[3];//粗跟踪俯仰
					
//  read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[18]= Read_JiaoYanHouShuJu[15];     //精跟踪当前方位位置
		AA_TXData[19]= Read_JiaoYanHouShuJu[16];		
		AA_TXData[20]= Read_JiaoYanHouShuJu[17];	
		AA_TXData[21]= Read_JiaoYanHouShuJu[18];	
		
		AA_TXData[22]=Read_JiaoYanHouShuJu[19];      //精跟踪当前俯仰位置
		AA_TXData[23]=Read_JiaoYanHouShuJu[20];
		AA_TXData[24]=Read_JiaoYanHouShuJu[21];
		AA_TXData[25]=Read_JiaoYanHouShuJu[22];
//		Read_JiaoYan_Flag = 0;
//	}	
	
	
	
//	TX_Precise_Azimuth=3;
//	Temp_Data.f=TX_Precise_Azimuth;//精跟踪初始方位位置

//	AA_TXData[18]=Temp_Data.ary[0];//精跟踪方位
//	AA_TXData[19]=Temp_Data.ary[1];//
//	AA_TXData[20]=Temp_Data.ary[2];//
//	AA_TXData[21]=Temp_Data.ary[3];//

//	TX_Precise_Pitch=4;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪初始俯仰位置

//	AA_TXData[22]=Temp_Data.ary[0];//精跟踪俯仰
//	AA_TXData[23]=Temp_Data.ary[1];//
//	AA_TXData[24]=Temp_Data.ary[2];//
//	AA_TXData[25]=Temp_Data.ary[3];//

	AA_TXData[26]=0XAA;//备份
	for(i=6;i<(27);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;//校验
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}

//跟踪模式应答
void GenZong_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X46;    //启动跟踪应答
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	//AA_TXData[11]=0XAA;//指向模式01仅粗跟踪02仅精跟踪03粗精跟踪04精跟踪校准补偿

	TX_Coarse_Azimuth=FW_encoder_degrees;
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪初始方位位置
	AA_TXData[10]=Temp_Data.ary[0];//粗跟踪方位
	AA_TXData[11]=Temp_Data.ary[1];//粗跟踪方位
	AA_TXData[12]=Temp_Data.ary[2];//粗跟踪方位
	AA_TXData[13]=Temp_Data.ary[3];//粗跟踪方位

	TX_Coarse_Pitch=FY_encoder_degrees;
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪初始俯仰位置

	AA_TXData[14]=Temp_Data.ary[0];//粗跟踪俯仰
	AA_TXData[15]=Temp_Data.ary[1];//粗跟踪俯仰
	AA_TXData[16]=Temp_Data.ary[2];//粗跟踪俯仰
	AA_TXData[17]=Temp_Data.ary[3];//粗跟踪俯仰
					
//	read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[18]= Read_JiaoYanHouShuJu[15];     //精跟踪当前方位位置
		AA_TXData[19]= Read_JiaoYanHouShuJu[16];		
		AA_TXData[20]= Read_JiaoYanHouShuJu[17];	
		AA_TXData[21]= Read_JiaoYanHouShuJu[18];	
		
		AA_TXData[22]=Read_JiaoYanHouShuJu[19];      //精跟踪当前俯仰位置
		AA_TXData[23]=Read_JiaoYanHouShuJu[20];
		AA_TXData[24]=Read_JiaoYanHouShuJu[21];
		AA_TXData[25]=Read_JiaoYanHouShuJu[22];
//		Read_JiaoYan_Flag = 0;
//	}
	
	
//	TX_Precise_Azimuth=3;
//	Temp_Data.f=TX_Precise_Azimuth;//精跟踪初始方位位置

//	AA_TXData[18]=Temp_Data.ary[0];//精跟踪方位
//	AA_TXData[19]=Temp_Data.ary[1];//
//	AA_TXData[20]=Temp_Data.ary[2];//
//	AA_TXData[21]=Temp_Data.ary[3];//

//	TX_Precise_Pitch=4;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪初始俯仰位置

//	AA_TXData[22]=Temp_Data.ary[0];//精跟踪俯仰
//	AA_TXData[23]=Temp_Data.ary[1];//
//	AA_TXData[24]=Temp_Data.ary[2];//
//	AA_TXData[25]=Temp_Data.ary[3];//

	AA_TXData[26]=0XAA;//备份
	for(i=6;i<(27);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;//校验
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;
}
//匀速扫描模式应答
void YunSuSaoMiao_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X47;      //启动匀速扫描应答 
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	TX_Coarse_Azimuth=FW_encoder_degrees;
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪初始方位位置

	AA_TXData[10]=Temp_Data.ary[0];
	AA_TXData[11]=Temp_Data.ary[1];
	AA_TXData[12]=Temp_Data.ary[2];
	AA_TXData[13]=Temp_Data.ary[3];

	TX_Coarse_Pitch=FY_encoder_degrees;
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪初始俯仰位置

	AA_TXData[14]=Temp_Data.ary[0];
	AA_TXData[15]=Temp_Data.ary[1];
	AA_TXData[16]=Temp_Data.ary[2];
	AA_TXData[17]=Temp_Data.ary[3];
					
//	read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[18]= Read_JiaoYanHouShuJu[15];     //精跟踪当前方位位置
		AA_TXData[19]= Read_JiaoYanHouShuJu[16];		
		AA_TXData[20]= Read_JiaoYanHouShuJu[17];	
		AA_TXData[21]= Read_JiaoYanHouShuJu[18];	
		
		AA_TXData[22]=Read_JiaoYanHouShuJu[19];      //精跟踪当前俯仰位置
		AA_TXData[23]=Read_JiaoYanHouShuJu[20];
		AA_TXData[24]=Read_JiaoYanHouShuJu[21];
		AA_TXData[25]=Read_JiaoYanHouShuJu[22];
//		Read_JiaoYan_Flag = 0;
//	}
	
	
//	TX_Coarse_Pitch=3;
//	Temp_Data.f=TX_Coarse_Pitch;//精跟踪初始方位位置

//	AA_TXData[18]=Temp_Data.ary[0];
//	AA_TXData[19]=Temp_Data.ary[1];
//	AA_TXData[20]=Temp_Data.ary[2];
//	AA_TXData[21]=Temp_Data.ary[3];

//	TX_Precise_Pitch=4;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪初始俯仰位置

//	AA_TXData[22]=Temp_Data.ary[0];
//	AA_TXData[23]=Temp_Data.ary[1];
//	AA_TXData[24]=Temp_Data.ary[2];
//	AA_TXData[25]=Temp_Data.ary[3];

	AA_TXData[26]=0XAA;//备份
	
	for(i=6;i<(27);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;           //校验字节
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}

//回零模式应答
void HuiLing_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X48;    //归零应答
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	TX_Coarse_Azimuth=FW_Huiling_last;
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪伺服方位回零前位置
	AA_TXData[10]=Temp_Data.ary[0];
	AA_TXData[11]=Temp_Data.ary[1];
	AA_TXData[12]=Temp_Data.ary[2];
	AA_TXData[13]=Temp_Data.ary[3];

	TX_Coarse_Pitch=FY_Huiling_last;
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪伺服俯仰回零前位置
	AA_TXData[14]=Temp_Data.ary[0];
	AA_TXData[15]=Temp_Data.ary[1];
	AA_TXData[16]=Temp_Data.ary[2];
	AA_TXData[17]=Temp_Data.ary[3];

//	read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[18]= Read_JiaoYanHouShuJu[23];     //精跟装方位回零前位置
		AA_TXData[19]= Read_JiaoYanHouShuJu[24];		
		AA_TXData[20]= Read_JiaoYanHouShuJu[25];	
		AA_TXData[21]= Read_JiaoYanHouShuJu[26];	
		
		AA_TXData[22]=Read_JiaoYanHouShuJu[27];      //精跟装俯仰回零前位置
		AA_TXData[23]=Read_JiaoYanHouShuJu[28];
		AA_TXData[24]=Read_JiaoYanHouShuJu[29];
		AA_TXData[25]=Read_JiaoYanHouShuJu[30];
//		Read_JiaoYan_Flag = 0;
//	}
	
//	TX_Precise_Azimuth=3;
//	Temp_Data.f=TX_Precise_Azimuth;//精跟踪伺服方位回零前位置
//	AA_TXData[18]=Temp_Data.ary[0];
//	AA_TXData[19]=Temp_Data.ary[1];
//	AA_TXData[20]=Temp_Data.ary[2];
//	AA_TXData[21]=Temp_Data.ary[3];

//	TX_Precise_Pitch=4;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪伺服俯仰回零前位置
//	AA_TXData[22]=Temp_Data.ary[0];
//	AA_TXData[23]=Temp_Data.ary[1];
//	AA_TXData[24]=Temp_Data.ary[2];
//	AA_TXData[25]=Temp_Data.ary[3];

	AA_TXData[26]=0XAA;//备份
	for(i=6;i<(27);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;      //校验字节
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;
}

//急停命令应答
void JiTing_YingDa()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X49;    //急停应答
	AA_TXData[5]=21;
	
	AA_TXData[6]=Time&0x000000ff;
  AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
	
	//AA_TXData[11]=0XAA;//指向模式01仅粗跟踪02仅精跟踪03粗精跟踪04精跟踪校准补偿
	
	TX_Coarse_Azimuth=FW_encoder_degrees;
	Temp_Data.f=TX_Coarse_Azimuth;//粗跟踪伺服方位 当前位置
	AA_TXData[10]=Temp_Data.ary[0];
	AA_TXData[11]=Temp_Data.ary[1];
	AA_TXData[12]=Temp_Data.ary[2];
	AA_TXData[13]=Temp_Data.ary[3];
	
	TX_Coarse_Pitch=FY_encoder_degrees;
	Temp_Data.f=TX_Coarse_Pitch;//粗跟踪伺服俯仰 当前位置
	AA_TXData[14]=Temp_Data.ary[0];
	AA_TXData[15]=Temp_Data.ary[1];
	AA_TXData[16]=Temp_Data.ary[2];
	AA_TXData[17]=Temp_Data.ary[3];
					
//	read_JingGenZong_Data();
//  JiaoYan_Read_Data();
//	if(Read_JiaoYan_Flag == 1)
//	{
		AA_TXData[18]= Read_JiaoYanHouShuJu[15];     //精跟踪当前方位位置
		AA_TXData[19]= Read_JiaoYanHouShuJu[16];		
		AA_TXData[20]= Read_JiaoYanHouShuJu[17];	
		AA_TXData[21]= Read_JiaoYanHouShuJu[18];	
		
		AA_TXData[22]=Read_JiaoYanHouShuJu[19];      //精跟踪当前俯仰位置
		AA_TXData[23]=Read_JiaoYanHouShuJu[20];
		AA_TXData[24]=Read_JiaoYanHouShuJu[21];
		AA_TXData[25]=Read_JiaoYanHouShuJu[22];
//		Read_JiaoYan_Flag = 0;
//	}
	
//	TX_Precise_Azimuth=5;
//	Temp_Data.f=TX_Precise_Azimuth;//精跟踪伺服方位 当前位置
//	AA_TXData[18]=Temp_Data.ary[0];
//	AA_TXData[19]=Temp_Data.ary[1];
//	AA_TXData[20]=Temp_Data.ary[2];
//	AA_TXData[21]=Temp_Data.ary[3];
//	
//	TX_Precise_Pitch=5;
//	Temp_Data.f=TX_Precise_Pitch;//精跟踪伺服俯仰 当前位置
//	AA_TXData[22]=Temp_Data.ary[0];
//	AA_TXData[23]=Temp_Data.ary[1];
//	AA_TXData[24]=Temp_Data.ary[2];
//	AA_TXData[25]=Temp_Data.ary[3];
	
	AA_TXData[26]=0XAA;          //备份
	for(i=6;i<(27);i++)
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[27]=Sum2;          //校验字节
	AA_TXData[28]=0X5A;
	AA_TXData[29]=0XFE;
	AA_TX_Len=30;	
	AA_Com_Write();
	CommandOld=CommandNew;
}

void BuJinSaoMiao()
{
//方位
			if(FW_BuJin_Flag == 1||FW_BuJin_Flag <= FW_BuJin_Count)                 //执行方位步进扫描零位
			{
				FW_BuJin_degree_out = FW_BuJin_degree_Start;
			}
			else
			{
				system_mode = 1;
				FW_BuJin_Flag = 0;
			}		
			if(FW_BuJin_Flag >> 1||FW_BuJin_Flag <= FW_BuJin_Count)			             //按步进行扫描
			{
				if(FW_BuJin_Direct == 0x01)      //0x01:向右扫描
				{
					FW_BuJin_limit = FW_BuJin_LiangCheng_Degree + FW_BuJin_degree_Start;  //计算右扫描限位									
					FW_BuJin_degree_out = FW_BuJin_degree_out + FW_BuJin_degree_step;
					if(FW_BuJin_degree_out >= FW_BuJin_limit)
					{
						FW_BuJin_Direct = 0x02;
					}
				}
				else if(FW_BuJin_Direct == 0x02)   //0x02:向左扫描
				{
					FW_BuJin_limit = FW_BuJin_degree_Start - FW_BuJin_LiangCheng_Degree;  //计算左扫描限位	
					FW_BuJin_degree_out = FW_BuJin_degree_out - FW_BuJin_degree_step;
					if(FW_BuJin_degree_out <= FW_BuJin_limit)
					{
						FW_BuJin_Direct = 0x01;
					}
				}
			}
			else
			{
				system_mode = 1;
				FW_BuJin_Flag = 0;
			}
//俯仰
			if(FY_BuJin_Flag == 1||FY_BuJin_Flag <= FY_BuJin_Count)                 //执行俯仰步进扫描零位
			{
				FY_BuJin_degree_out = FY_BuJin_degree_Start;
			}
      else
			{
				system_mode = 1;
				FY_BuJin_Flag = 0;
			}		
			if(FY_BuJin_Flag >> 1||FY_BuJin_Flag <= FY_BuJin_Count)			             //按步进行扫描
			{
				if(FY_BuJin_Direct == 0x01)      //0x01:向上扫描
				{
					FY_BuJin_limit = FY_BuJin_LiangCheng_Degree + FY_BuJin_degree_Start;  //计算右扫描限位									
					FY_BuJin_degree_out = FY_BuJin_degree_out + FY_BuJin_degree_step;
					if(FY_BuJin_degree_out >= FY_BuJin_limit)
					{
						FY_BuJin_Direct = 0x02;
					}
				}
				else if(FY_BuJin_Direct == 0x02)   //0x02:向下扫描
				{
					FY_BuJin_limit = FY_BuJin_degree_Start - FY_BuJin_LiangCheng_Degree;  //计算左扫描限位	
					FY_BuJin_degree_out = FY_BuJin_degree_out - FY_BuJin_degree_step;
					if(FY_BuJin_degree_out <= FY_BuJin_limit)
					{
						FY_BuJin_Direct = 0x01;
					}
				}
			}
			else
			{
				system_mode = 1;
				FW_BuJin_Flag = 0;
			}	

			FW_location_set = FW_BuJin_degree_out;
			FY_location_set = FY_BuJin_degree_out;
}

float getFW_step_chafen_speed(float piror,float current,float time)
{
	float chafenspeed;
	chafenspeed = (current - piror) * time;
	global_FW_step_piror = current;
	return chafenspeed;
}	
float getFY_step_chafen_speed(float piror,float current,float time)
{
	float chafenspeed;
	chafenspeed = (current - piror) * time;
	global_FY_step_piror = current;
	return chafenspeed;
}	


unsigned int touluo_InterruptCnt1 = 0;	//进入中断的总次数
unsigned int touluo_InterruptCnt2 = 0;	//帧头错误
unsigned int touluo_InterruptCnt3 = 0;	//校验和错误
unsigned int touluo_InterruptTrue = 0;	//数据正确

//读方位光纤陀螺
void Com_Read_FW_touluo(void)
{
		u32 SF_ADDR_tmp_touluo=0;
		SF_ADDR_tmp_touluo = 0x64001800;
		for(SF_touluo_i=0; SF_touluo_i<SF_RX_touluo_Len; SF_touluo_i++)
		{
			SF_RX_touluo_Data[SF_touluo_i]=*(uint32_t*)(SF_ADDR_tmp_touluo);
			SF_ADDR_tmp_touluo = SF_ADDR_tmp_touluo + 2;
		}
		touluo_DATA_FW_Parse();
}

//写方位光纤陀螺
void Com_Write_FW_touluo(void)
{
		u32 SF_ADDR_tmp_touluo=0;
		SF_ADDR_tmp_touluo = 0x64002000;
		for(SF_touluo_i=0; SF_touluo_i<SF_TX_touluo_Len; SF_touluo_i++)
		{
			*(uint32_t*)(SF_ADDR_tmp_touluo)= SF_TX_touluo_Data[SF_touluo_i];
		}
		*(uint32_t*)(0x64002400)= 1;
		
}

//解方位光纤陀螺信号
void touluo_DATA_FW_Parse(void)
{
		u16 i=0,Sum1=0,Sum2=0;
		touluo_InterruptCnt1++;						//进入中断的总次数
		if(SF_RX_touluo_Data[0] == 0x80)  //帧头
		{
				for(i=1 ;i<(SF_RX_touluo_Len-4) ;i++)//10-4=6
				{
						Sum1 ^= SF_RX_touluo_Data[i];    //1 2 3 4 5
				}
				for(i=7 ;i< SF_RX_touluo_Len - 1 ;i++)//10-1=9
				{
						Sum2 ^= SF_RX_touluo_Data[i];			//7 8 
				}
				if((SF_RX_touluo_Data[6] == Sum1) & (SF_RX_touluo_Data[9] == Sum2))
				{
						touluo_InterruptTrue++;
						Tuoluo_bit1 = (SF_RX_touluo_Data[1])&0x7F; 
						Tuoluo_bit2 = (SF_RX_touluo_Data[2])&0x7F; 
						Tuoluo_bit3 = (SF_RX_touluo_Data[3])&0x7F; 
						Tuoluo_bit4 = (SF_RX_touluo_Data[4])&0x7F; 
						Tuoluo_bit5 = (SF_RX_touluo_Data[5])&0x0F;
						Tuoluo_bit_u32 = (Tuoluo_bit5<<28)|(Tuoluo_bit4<<21)|(Tuoluo_bit3<<14)|(Tuoluo_bit2<<7)|Tuoluo_bit1;	
						Tuoluo_bit_int = Tuoluo_bit_u32;
						if((Tuoluo_bit_int & 0x8000) == 1)
						{
								Tuoluo_bit_float = - Tuoluo_bit_int	/ touluo_biaoduyingsu;	//陀螺角速度（°/s）
						}
						else
						{
								Tuoluo_bit_float =   Tuoluo_bit_int	/ touluo_biaoduyingsu;	//陀螺角速度（°/s）
						}
					
						Tuoluo_temperature1 = (SF_RX_touluo_Data[7])&0x7F; 
						Tuoluo_temperature2	=	(SF_RX_touluo_Data[8])&0x7F; 
						Tuoluo_temperature_u16 = (Tuoluo_temperature2<<7)|Tuoluo_temperature1;
						Tuoluo_temperature_int = Tuoluo_temperature_u16;
						if((Tuoluo_temperature_int & 0x0020) == 1)
						{
								Tuoluo_temperature_float = - Tuoluo_temperature_int	/ touluo_biaoduyingsu;	//温度（°C）
						}
						else
						{
								Tuoluo_temperature_float =   Tuoluo_temperature_int	/ touluo_biaoduyingsu;	//温度（°C）
						}
				}
				else
				{
						touluo_InterruptCnt3++;
				}
		}
		else
		{
				touluo_InterruptCnt2++;
		}
}

void touluo_Data_FW_TX(void)
{
//		u16 i=0,Sum1=0,Sum2=0;

}

//读俯仰光纤陀螺
void Com_Read_FY_touluo(void)
{
		u32 SF_ADDR_tmp_touluo=0;
		u8 SF_touluo_Y = 0;
		SF_ADDR_tmp_touluo = 0x64002800;
		for(SF_touluo_Y=0; SF_touluo_Y<SF_RX_touluo_Len_Y; SF_touluo_Y++)
		{
			SF_RX_touluo_Data_Y[SF_touluo_Y]=*(uint32_t*)(SF_ADDR_tmp_touluo);
			SF_ADDR_tmp_touluo = SF_ADDR_tmp_touluo + 2;
		}
		touluo_DATA_FY_Parse();
}

//写俯仰光纤陀螺
void Com_Write_FY_touluo(void)
{
		u32 SF_ADDR_tmp_touluo=0;
		u8 SF_touluo_Y = 0;
		SF_ADDR_tmp_touluo = 0x64002000;
		for(SF_touluo_Y=0; SF_touluo_Y<SF_TX_touluo_Len_Y; SF_touluo_Y++)
		{
			*(uint32_t*)(SF_ADDR_tmp_touluo)= SF_TX_touluo_Data_Y[SF_touluo_Y];
		}
		*(uint32_t*)(0x64002400)= 1;
		
}

unsigned int touluo_InterruptCnt1_Y = 0;	//进入中断的总次数
unsigned int touluo_InterruptCnt2_Y = 0;	//帧头错误
unsigned int touluo_InterruptCnt3_Y = 0;	//校验和错误
unsigned int touluo_InterruptTrue_Y = 0;	//数据正确
//解俯仰光纤陀螺信号
void touluo_DATA_FY_Parse(void)
{
		u16 i=0,Sum1=0,Sum2=0;
		touluo_InterruptCnt1_Y++;						//进入中断的总次数
		if(SF_RX_touluo_Data_Y[0] == 0x80)  //帧头
		{
				for(i=1 ;i<(SF_RX_touluo_Len_Y-4) ;i++)//10-4=6
				{
						Sum1 ^= SF_RX_touluo_Data_Y[i];			//1 2 3 4 5
				}
				for(i=7 ;i< SF_RX_touluo_Len_Y - 1 ;i++)//10-1=9
				{
						Sum2 ^= SF_RX_touluo_Data_Y[i];
				}
				if((SF_RX_touluo_Data_Y[6] == Sum1) & (SF_RX_touluo_Data_Y[9] == Sum2))
				{
						touluo_InterruptTrue_Y++;
						Tuoluo_bit1_Y = (SF_RX_touluo_Data_Y[1])&0x7F; 
						Tuoluo_bit2_Y = (SF_RX_touluo_Data_Y[2])&0x7F; 
						Tuoluo_bit3_Y = (SF_RX_touluo_Data_Y[3])&0x7F; 
						Tuoluo_bit4_Y = (SF_RX_touluo_Data_Y[4])&0x7F; 
						Tuoluo_bit5_Y = (SF_RX_touluo_Data_Y[5])&0x0F;
						Tuoluo_bit_u32_Y = (Tuoluo_bit5_Y<<28)|(Tuoluo_bit4_Y<<21)|(Tuoluo_bit3_Y<<14)|(Tuoluo_bit2_Y<<7)|Tuoluo_bit1_Y;	
						Tuoluo_bit_int_Y = Tuoluo_bit_u32_Y;
						if((Tuoluo_bit_int_Y & 0x8000) == 1)
						{
								Tuoluo_bit_float_Y = - Tuoluo_bit_int_Y	/ touluo_biaoduyingsu_Y;	//陀螺角速度（°/s）
						}
						else
						{
								Tuoluo_bit_float_Y =   Tuoluo_bit_int_Y	/ touluo_biaoduyingsu_Y;	//陀螺角速度（°/s）
						}
					
						Tuoluo_temperature1_Y = (SF_RX_touluo_Data_Y[7])&0x7F; 
						Tuoluo_temperature2_Y	=	(SF_RX_touluo_Data_Y[8])&0x7F; 
						Tuoluo_temperature_u16_Y = (Tuoluo_temperature2_Y<<7)|Tuoluo_temperature1_Y;
						Tuoluo_temperature_int_Y = Tuoluo_temperature_u16_Y;
						if((Tuoluo_temperature_int_Y & 0x0020) == 1)
						{
								Tuoluo_temperature_float_Y = - Tuoluo_temperature_int_Y	/ touluo_biaoduyingsu_Y;	//温度（°C）
						}
						else
						{
								Tuoluo_temperature_float_Y =   Tuoluo_temperature_int_Y	/ touluo_biaoduyingsu_Y;	//温度（°C）
						}
				}
				else
				{
						touluo_InterruptCnt3_Y++;
				}
		}
		else
		{
				touluo_InterruptCnt2_Y++;
		}
}

void touluo_Data_FY_TX(void)
{
//		u16 i=0,Sum1=0,Sum2=0;

}
