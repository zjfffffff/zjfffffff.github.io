#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f4xx.h"	

#define YAW_ANGLE 24
#define ROLL_ANGLE 21
#define PITCH_ANGLE 18

#define ANGLE_GAIN 0.01

#define PITCH_GERO 1
#define ROLL_GERO 4
#define YAW_GERO 7

#define GERO_GAIN 0.00025

float RMS2_vol(float Final,float Final2);
float RMS2_vol1(float Final);
float RMS2_vol2(float Final2);

void TIM10_Init(u16 arr,u16 psc);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM2_config();

float fhan(float x1,float x2, float r, float h);

void read_FY_encoder(void);
void read_FW_encoder(void);
void read_inertial_navigation(void);  //串口2读取总控
void motor_move(float FW_location,float FY_location);
void read_master_control(void);              //串口6读取总控
void read_miss_distance(void);               //串口5读取脱靶量
//void read_inertial_navigation_slave(void);    //串口1读取陀螺惯导
float small_navigation_calculate(u8 index,float gain);
void ReadImuData(void);
void reset_zero(void);
void miss_distance_track(void);
void power_down(void);
void space_scan(void);
void stabilization(void);
void read_M1tuoluo(void);
void read_LVDS(void);
void write_LVDS(void);
void read_temperature(void);
void para_set(void);
////////////
void read_jishangguandao(); 



void read_labview_set(void);
float Ary4ToFloat(char c1, char c2, char c3, char c4);  
float getFW_chafen_speed(float piror,float current,float time);
float getFY_chafen_speed(float piror,float current,float time);
u32 get_optical_navigation(u32 threebit);
void TIM2_config(void);
void CaiJiMingLing_YingDa();
void ZiJian_YingDa();
void ZhuangTaiChaXun_YingDa();
void ZhiXiang_YingDa();
void BuJinSaoMiao_YingDa();
void GenZong_YingDa();
void YunSuSaoMiao_YingDa();
void HuiLing_YingDa();
void JiTing_YingDa();
void BuJinSaoMiao();
float getFW_step_chafen_speed(float piror,float current,float time);
float getFY_step_chafen_speed(float piror,float current,float time);

void Com_Read_FW_touluo(void);		//读方位光纤陀螺
void Com_Write_FW_touluo(void); 	//写方位光纤陀螺
void touluo_DATA_FW_Parse(void);	//方位解包
void touluo_Data_FW_TX(void);

void Com_Read_FY_touluo(void);		//读俯仰光纤陀螺
void Com_Write_FY_touluo(void); 	//写俯仰光纤陀螺
void touluo_DATA_FY_Parse(void);	//俯仰解包
void touluo_Data_FY_TX(void);


#endif

