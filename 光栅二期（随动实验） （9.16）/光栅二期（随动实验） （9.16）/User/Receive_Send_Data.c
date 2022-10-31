#include "Receive_Send_Data.h"
#include "stm32f4xx.h"
#include "elmo.h"
#include "math.h"
#include "FsmCompensator.h"
#include "FsmCompensator_servo.h"
extern double jiao_1,jiao_2;
extern float yaw_attitude_float_ly ;
extern float pitch_attitude_float_ly ;
extern float roll_attitude_float_ly ;
extern float FW_tuoluo_loop_error;
extern float FY_tuoluo_loop_error;
float GS_Suidong_Flag=0;
extern float GS_FW_DEG,GS_FY_DEG,GS_HG_DEG;;
float GS_Suidong_offset_FW=0;
float GS_Suidong_offset_FY=0;
int Suidong_jishu=0;
float FY_Degree_GS_suidong,FW_Degree_GS_suidong;
/////////////////////////////////
//粗伺服接收机上总控板的数据
u16 baochang = 0;
u16 jieshoushijian = 0;
int zhenjishu = 0;
extern float Tuoluo_bit_float_k_2;
extern float FW_Tuoluo_bit_float ;		//方位光纤陀螺角速度
extern float FY_Tuoluo_bit_float ;		//俯仰光纤陀螺角速度
extern u8 GOY_K;
extern float k_11 ;
extern float GOY_K2 ;
extern float GOY_K1;


extern float FW_location_set ;    //方位设定值
extern float FY_location_set ;    //编码器闭环设定值

int genzon_biaozhi = 0;
float Four_Quadrant_bit_X = 0;					//四象限X
float Four_Quadrant_bit_Y = 0;					//四象限Y

float Shangweiji_Pitch_Angle_Value_prepass = 0;
float Tuoluo_bit_float12 = 0;
float Tuoluo_bit_float11 = 0;
float Tuoluo_bit_float_k = 1.82;
float Tuoluo_bit_float_k1 = - 0.019;
float Tuoluo_bit_float1 = 0;
float Tuoluo_bit_float_Y1 = 0;	
float Tuoluo_bit_float = 0;							//光纤陀螺传回角速度
float Tuoluo_bit_float_Y = 0;						//光纤陀螺传回角速度
extern float touluo_biaoduyingsu;				//陀螺标度因数
s32 Tuoluo_bit_u32_X1;								//陀螺X
s32 Tuoluo_bit_u32_Y1;								//陀螺Y

u8 XunHuan_Ma = 0;
u8 Receive_XunHuan_Ma_low = 0;	//接收循环码
u8 Receive_XunHuan_Ma_high = 0;	
u8 Send_XunHuan_Ma_TX = 0;				//向机下总控发送循环码

extern float miss_offset_X;				//设置的偏置方位脱靶量的变量
extern float miss_offset_Y;				//设置的偏置方位脱靶量的变量

//extern float Point_FY_set_prepass;
u8 Receive_Data[Receive_RX_Len];
u8 Receive_TXData[Receive_TX_Len];
u8 Receive_DBJ_Data[Receive_Suidong_GS_Len];
//extern u8 SF_mode;					//控制命令 1:指向 2:回零 3:跟踪 4:停止   5:设置相机
//extern float FW_pointing_degree_LY;
//extern float FY_pointing_degree_LY;
s32 	Receive_Direction_Angle_Value;	//指向方位位置
s32 	Receive_Pitch_Angle_Value;			//指向俯仰位置
u8 Receive_IP;											//跟踪伺服IP号
u8 Receive_suidong_IP; //danbaijingzhuangtai
typedef union
{
	struct
	{
		u8 Low_byte;
		u8 MLow_byte;
		u8 MHigh_byte;
		u8 High_byte;
	}Receive_Data_byte;
	float Receive_Data_float;
	u32 Data_u32;	  //无符号整形
	s32 Data_s32;     //有符号整形
}Receive_Data_value;
Receive_Data_value Temp_32_Receive;
/////////////////////////////////////////////////////

/////////////////////////////////////////////////////
//粗伺服发给机下总控板的数据
//typedef union
//{
//	struct
//	{
//		u8 Low_byte;
//		u8 MLow_byte;
//		u8 MHigh_byte;
//		u8 High_byte;
//	}Send_Data_byte;
//	float Send_Data_float;
//	u32 Send_Data_u32;	  //无符号整形
//	s32 Send_Data_s32;     //有符号整形
//}Send_Data_value;
//Send_Data_value Temp_32_Send;

extern float x_axis_velocity_float ;
extern float y_axis_velocity_float ;
extern float z_axis_velocity_float ;

extern float FW_encoder_degrees;  //方位编码器转换为角度
extern float FY_encoder_degrees;  //俯仰编码器转换为角度

extern float Speed_encode_FY;
extern float Speed_encode_FW;

int cugenzongmoshi = 0;
int cugenzongzhiling = 0;

extern float miss_distance_X_float;
extern float miss_distance_Y_float;

float Send_Direction_Angle_AnswerValue;	//指向方位位置
float Send_Pitch_Angle_AnswerValue;		//指向俯仰位置

float FW_Jixia_tuoba_degrees = 0;		//脱靶量X
float FY_Jixia_tuoba_degrees = 0;		//脱靶量Y

float FX_JixiaTuoluo_bit_float = 0; //陀螺X
float FY_JixiaTuoluo_bit_float = 0;	//陀螺Y

float Send_Four_Quadrant_bit_X = 0; //四象限X
float Send_Four_Quadrant_bit_Y = 0; //四象限Y

float Send_com_En = 0x02;				//粗跟踪使能:0x01:卸载；0x02：使能

extern u8 system_mode; 					//
u8 Send_gai_Status = 0;					//粗跟踪模式
u8 Send_mode;										//粗跟踪指令 1:指向 2:零点修正 3:跟仔拚 4:归零5:粗伺服使能  6:停止 7:图像处理板切换
u8 Send_Status = 0;					      	//粗跟踪指令执行状态 0x00:待机 0x01:执行中 0x02:执行完 0x03:未执行
u8 FY_Xianwei;									//俯仰限位
u8 Send_Err1;										//粗伺服故障代码1
u8 Send_Err2;										//粗伺服故障代码2
u8 Send_CAMERA_Err = 0;					//惯导通信状态 0x00:通信正常 0x01:通信故障

float POS_SP_SWITCH = 0;        //发送速度或编码器位置

///////////////////////////////////////////////////////////////////////////////////////////
///粗伺服接收机上位机的数据
u8 Shangweiji_Data[Shangweiji_RX_Len];	
u8 Shangweiji_Data_len = 0;             //数据包长度
u8 Shangweiji_Command;									//指令执行状态 
float 	Shangweiji_Direction_Angle_Value;				//指向方位位置
float 	Shangweiji_Pitch_Angle_Value;						//指向俯仰位置
float GS_Suidong_FW=0.01;
float GS_Suidong_chafen_FW;
float GS_Suidong_chafen_FY;
float GS_Suidong_FW_before;
float GS_Suidong_FY_before;
float GS_Suidong_FY=0.01;

float Shangweiji_Direction_Angle_prepass = 0;

extern float FW_pointing_degree_LY;
//extern float FY_pointing_degree_LY;
extern float Point_FY_set_prepass;
float Shangweiji_Send_time = 0;						//上位机本包数据发送时间
u8 BaoJishu_Ma = 0;												//包计数码
char Shineng_biaozhi = 0;										//使能标志
char lici_biaozhi = 0;	                    //励磁标志
extern float FX_set_prepass_xiuzheng;
extern float FY_set_prepass_xiuzheng;
//////////////////////////////////////////////////////////////////////////////////////////

unsigned int touluo_Cnt1 = 0;	//进入中断的总次数
unsigned int touluo_Cnt2 = 0;	//帧头错误
unsigned int touluo_Cnt3 = 0;	//帧未错误
unsigned int touluo_Cnt4 = 0;	//校验和错误
unsigned int touluo_True = 0;	//数据正确
extern double jiao1,jiao2;//di 1 zu jie
extern double jiao3,jiao4;//di 2 zu jie

extern float FY_zero_degree;              //0.06 《- 323.02  2 

extern double FW_Degree_GS;
extern double FY_Degree_GS;

//float MissKx=0.72,MissKy=0.72;
float MissKx=1,MissKy=1;
//粗伺服接收机上总控板的数据
void Receive_DATA_Parse()
{
		u8 i=0,Sum=0;
		touluo_Cnt1 ++;			        //进入中断的总次数
		if((Receive_Data[0]==0xEB) & (Receive_Data[1]==0x90))//帧头
		{
				if(0==0)//帧尾
				{
						Sum=0;
				for(i=0;i<(Receive_RX_Len-1);i++)
					 {
						Sum+=Receive_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_Data[Receive_RX_Len-1]==(Sum))//求和校验
			
						{	
								touluo_True ++;	//数据正确
							  baochang  = Receive_Data[3];
								Receive_IP = Receive_Data[4];								
							if(Receive_IP == 6)/////////////////粗跟踪伺服指向/////
							{
									  system_mode = 17;
										
										//粗跟踪伺服方位
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[5];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[6];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[7];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[8];
										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Data_s32; 
//							    	Shangweiji_Direction_Angle_prepass = FsmLeadLag1(Shangweiji_Direction_Angle_Value,FSM_X,0);
								
										FW_Degree_GS = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
										//粗跟踪伺服俯仰
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[9];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[10];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[11];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[12];
										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Data_s32; 
//								    Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(Shangweiji_Pitch_Angle_Value,FSM_Y,0);
								
								
										FY_Degree_GS = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
							}
							else if(Receive_IP == 7)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
//							system_mode = 26;	
							}
							else if(Receive_IP == 8)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
//							system_mode = 26;	
							}
							else if(Receive_IP == 9)/////////////粗跟踪伺服跟踪修正/////（调试阶段设为跟踪模式）
							{
//								    system_mode = 25;
										
//										//粗跟踪伺服方位
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[6];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[7];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[8];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Direction_Angle_Value = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//										//粗跟踪伺服俯仰
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[10];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[11];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[12];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Pitch_Angle_Value = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//									
//										miss_offset_X = Shangweiji_Direction_Angle_Value;  //接收设置的偏置方位脱靶量
//										miss_offset_Y = Shangweiji_Pitch_Angle_Value;      //接收设置的偏置俯仰脱靶量
							}
							else if(Receive_IP == 18)///////////////粗跟踪伺服归零
							{
								    system_mode = 3;
							}
							else if(Receive_IP == 19)/////////////粗跟踪伺服使能
							{
						      	Shineng_biaozhi = Receive_Data[5];
										if(Shineng_biaozhi == 0)	 //失能
										{
												system_mode = 2;
										}
										if(Shineng_biaozhi == 1)   //使能
										{
												system_mode = 4;
										}
							}
							else if (Receive_IP == 20)/////////////粗跟踪伺服停止
							{
								    system_mode = 1;
							}
							else if (Receive_IP == 21)///////////////粗跟踪伺服闭环
							{
										lici_biaozhi = Receive_Data[5];
										if (lici_biaozhi == 2)///////////////可见光闭环
										{
//											system_mode = 7;
										}
										else if (lici_biaozhi == 1)/////////////////红外闭环
										{
										}    
							}
							else if (Receive_IP == 25)////////////////////外引/////////////
							{
							}
							else if (Receive_IP == 26)///////////////////收藏////////////////
							{
							}
							else if (Receive_IP == 27)/////////////////展开///////////////
							{
							}
							else if (Receive_IP == 30)////////////////////粗跟踪曲线切换/////////////
							{	
							}
							else if (Receive_IP == 33)////////////////粗精伺服状态查询//////////////////////
							{	
							}
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[24];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[25];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[26];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[27];
										jieshoushijian =Temp_32_Receive.Receive_Data_float;
							
							 
						}
						else
						{
							touluo_Cnt4 ++;	//校验和错误
						}
				}
				else
				{
					touluo_Cnt3 ++;	//帧未错误
				}
		}
		else
		{
			touluo_Cnt2 ++;			//帧头错误
		}
		Receive_XunHuan_Ma_low = Receive_Data[31];					//循环码
		Receive_XunHuan_Ma_high = Receive_Data[32];
	 zhenjishu = Receive_XunHuan_Ma_high + Receive_XunHuan_Ma_low>>8;
}

//粗伺服接收机上位机的数据
void Receive_Shangweiji_DATA_Parse()
{
//		u16 i=0,Sum=0;
//		if((Shangweiji_Data[0]==0xEB) & (Shangweiji_Data[1]==0x90))//帧头
//		{
//				Shangweiji_Data_len = Shangweiji_Data[2];			//数据包长度43
//				if(Shangweiji_Data[Shangweiji_RX_Len-1]==0xFE)//帧尾
//				{
//						for(i=2;i<(Shangweiji_RX_Len-2);i++)
//						{
//								Sum += Shangweiji_Data[i];
//						}
//						if(Shangweiji_Data[Shangweiji_RX_Len-2]==Sum)//求和校验
//						{
//								Shangweiji_Command = Shangweiji_Data[3];	//指令状态 1:指向 2:零点修正 3:跟踪修正 4:归零 5:使能 6:停止
//								
//								if(Shangweiji_Command == 1)    //指向
//								{
//										system_mode = 17;
//										
//										//粗跟踪伺服方位
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FW_pointing_degree_LY = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//										//粗跟踪伺服俯仰
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Point_FY_set_prepass = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//								}
//								if(Shangweiji_Command == 2)				//零点修正
//								{
//										system_mode = 3;
//									
//										//粗跟踪伺服方位
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FX_set_prepass_xiuzheng = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//										//粗跟踪伺服俯仰
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										FY_set_prepass_xiuzheng = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//								}
//								if(Shangweiji_Command == 3)    //跟踪修正
//								{
//										system_mode = 7;
//										
//										//粗跟踪伺服方位
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[4];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[5];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[6];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[7];
//										Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Direction_Angle_Value = Shangweiji_Direction_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//										//粗跟踪伺服俯仰
//										Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[8];
//										Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[9];
//										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[10];
//										Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[11];
//										Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float; 
//										Shangweiji_Pitch_Angle_Value = Shangweiji_Pitch_Angle_Value; 							//*360/2/3.14/1000/1000;//urad转°;
//									
//										miss_offset_X = Shangweiji_Direction_Angle_Value;  //接收设置的偏置方位脱靶量
//										miss_offset_Y = Shangweiji_Pitch_Angle_Value;      //接收设置的偏置俯仰脱靶量
//								}
//								if(Shangweiji_Command == 4)    //回零
//								{
//										system_mode = 3;
//								}
//								if(Shangweiji_Command == 5)    //使能标志
//								{	
//										Shineng_biaozhi = Shangweiji_Data[5];
//										if(Shineng_biaozhi == 1)	 //失能
//										{
////												wMO_FW(0x37F,0);
////												wMO_FY(0x37F,0);
//												system_mode = 2;
//										}
//										if(Shineng_biaozhi == 2)   //使能
//										{
////												wMO_FW(0x37F,1);
////												wMO_FY(0x37F,1);
//												system_mode = 4;
//										}
//								}
//								if(Shangweiji_Command == 6)    //停止
//								{
//										system_mode = 1;
//								}
//								
////								//本包数据发送时间（ms）
////								Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[36];
////								Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[37];
////								Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[38];
////								Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[39];
////								Shangweiji_Send_time = Temp_32_Receive.Receive_Data_float; 
//								
//								//包计数码
////								BaoJishu_Ma =	Shangweiji_Data[40];
//						}
//				}
//		}
}

extern float FY_miss_distance_light_diff;

u8 tt = 0;
extern float FY_encoder_quzhi;
extern float FY_output_value_da;
extern float x1_axis_velocity_float;
extern float x_axis_velocity_1msdistance_sum ; 
extern float x_axis_velocity_1msdistance_sum1;
extern float FY_location_loop_error;
extern float FW_location_loop_error;
extern float z_axis_velocity_1msdistance_sum;
extern double *Deg_2_1;
extern double *Deg_1_1;
void Receive_DATA_Parse_2th()
{
	u16 i=0;
	u8 Sum=0;
	touluo_Cnt1 ++;			        //进入中断的总次数
	if((Receive_Data[0]==0xEB) & (Receive_Data[1]==0x90))//帧头
	{
		Sum=0;
		for(i=0;i<(Receive_RX_Len-1);i++)
		{
			Sum+=Receive_Data[i];
		}
		if(Receive_Data[Receive_RX_Len-1]==Sum && Receive_Data[2]==0x1E)//校验值和长度
		{
			touluo_True ++;	//数据正确
			baochang  = Receive_Data[2];
			Receive_IP = Receive_Data[3];
			
     if(Receive_IP == 2)/////////////////粗跟踪伺服指向/////
							{
									  system_mode = 17;	
								//光栅角度1
				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
				Shangweiji_Direction_Angle_Value = Temp_32_Receive.Data_s32; 
				FW_Degree_GS = Shangweiji_Direction_Angle_Value/10000; 
//				FW_Degree_GS = Temp_32_Receive.Receive_Data_float;
				//光栅角度2
				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
				Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Data_s32; 
				FY_Degree_GS = Shangweiji_Pitch_Angle_Value/10000; 							//*360/2/3.14/
//				FY_Degree_GS = Temp_32_Receive.Receive_Data_float; 							//*360/2/3.14/1000/1000;//urad转°;
							}
							else if(Receive_IP == 3)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
							system_mode = 25;	
							}
							else if(Receive_IP == 4)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
							system_mode = 3;	
							}
							else if(Receive_IP == 5)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
							system_mode = 1;	
							}
			        else if(Receive_IP == 0x40)///////////////粗跟踪伺服扫描/////(调试阶段设为稳定模式)
							{
							if(Receive_Data[4]==1)
							{system_mode = 2;}
              if(Receive_Data[4]==2)
							{system_mode = 4;}							
							}
//			switch (Receive_IP)
//			{
//			case 2:
//				system_mode = 17;
//				//光栅角度1
//				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
//				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
//				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
//				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
//				FW_Degree_GS = Temp_32_Receive.Receive_Data_float;
//				//光栅角度2
//				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
//				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
//				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
//				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
//				FY_Degree_GS = Temp_32_Receive.Receive_Data_float;
//				break;
//			case 3:
////				//双光栅伺服方位角度
////				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[4];
////				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[5];
////				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[6];
////				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[7];
////				Shangweiji_Direction_Angle_Value = Temp_32_Receive.Receive_Data_float;
////				FW_Degree_GS = Shangweiji_Direction_Angle_Value;
////				//双光栅伺服俯仰角度
////				Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_Data[8];
////				Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_Data[9];
////				Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_Data[10];
////				Temp_32_Receive.Receive_Data_byte.High_byte = Receive_Data[11];
////				Shangweiji_Pitch_Angle_Value = Temp_32_Receive.Receive_Data_float;
////				FY_Degree_GS = Shangweiji_Pitch_Angle_Value;
//			system_mode = 25;
//				break;
//			case 4:    //双光栅跟踪
//				system_mode = 3;
//				break;
////			case 4:    //双光栅归零
////				system_mode = 3;
////				break;
//			case 5:    //双光栅停止
//				system_mode = 1;
//				break;
//			default:
//				break;
//			}
		}
		else
		{
			touluo_Cnt4 ++;	//校验和错误
		}
	}
	else
	{
		touluo_Cnt2 ++;			//帧头错误
	}

	//本包数据发送时间(ms)
	Temp_32_Receive.Receive_Data_byte.Low_byte = Shangweiji_Data[24];
	Temp_32_Receive.Receive_Data_byte.MLow_byte = Shangweiji_Data[25];
	Temp_32_Receive.Receive_Data_byte.MHigh_byte = Shangweiji_Data[26];
	Temp_32_Receive.Receive_Data_byte.High_byte = Shangweiji_Data[27];
	Shangweiji_Send_time = Temp_32_Receive.Receive_Data_float;

	//包计数码
	BaoJishu_Ma = Receive_Data[28];
}
//粗伺服发给机下总控板的数据
void Send_Data_TX(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//帧头
	Receive_TXData[2] = 49;
	Temp_32_Receive.Data_s32 = (FW_encoder_degrees)*10000;
//	Temp_32_Receive.Data_s32 = (GS_Suidong_chafen_FW)*10000;
//	Temp_32_Receive.Data_s32 = (jiao_1)*10000;
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	Temp_32_Receive.Data_s32 = (FY_encoder_degrees )*10000;
//	Temp_32_Receive.Data_s32 = (GS_Suidong_chafen_FY )*10000;
//	Temp_32_Receive.Data_s32 = (jiao_2)*10000;
	Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	if(POS_SP_SWITCH == 1)
	{
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FW * 1000.0f);
		Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FY * 1000.0f);
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	}
	if(system_mode == 4)
	{
		Temp_Uchar = 1;
	}
	else if(system_mode == 2 )
	{
		Temp_Uchar = 0;
	}
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{

		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{

		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// 惯导///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
//	Temp_32_Receive.Data_s32 = z_axis_velocity_float*10000;		//方位速度
//	Temp_32_Receive.Data_s32 = GS_FW_DEG*10000;		//方位速度
	Temp_32_Receive.Data_s32 = yaw_attitude_float_ly*10000;		//方位速度9.16
//	Temp_32_Receive.Data_s32 = FW_location_loop_error*10000;		//方位速度9.16
	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
//	Temp_32_Receive.Data_s32 = x_axis_velocity_float*10000; 	// 俯仰速度
//	Temp_32_Receive.Data_s32 = GS_FY_DEG*10000;		//方位速度
	Temp_32_Receive.Data_s32 = pitch_attitude_float_ly*10000; 	// 俯仰速度9.16
//	Temp_32_Receive.Data_s32 = FY_location_loop_error*10000; 	// 俯仰速度9.16
	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
//	Temp_32_Receive.Data_s32 = y_axis_velocity_float*10000;		//横滚速度
//	Temp_32_Receive.Data_s32 = GS_HG_DEG*10000;		//方位速度
	Temp_32_Receive.Data_s32 = roll_attitude_float_ly*10000;		//横滚姿态
	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//  Receive_TXData[31] = 0x00;
//  Receive_TXData[32] = 0x00;
//	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[31] = 0;
	//		//粗跟踪伺服脱靶量X
//		FW_Jixia_tuoba_degrees = FW_location_loop_error*100;
		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float*100*15.7;
		Temp_32_Receive.Data_s32 = FW_Jixia_tuoba_degrees;
		Receive_TXData[32] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//粗跟踪伺服脱靶量Y
//		FY_Jixia_tuoba_degrees = FY_location_loop_error*100;
		FY_Jixia_tuoba_degrees = MissKy*miss_distance_Y_float*100*15.7;
		Temp_32_Receive.Data_s32 = FY_Jixia_tuoba_degrees;
		Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[37] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[38] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[39] = Temp_32_Receive.Receive_Data_byte.High_byte;
	  Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
	  for(i = 2; i < (Receive_TX_Len - 2); i++)
			{
				Sum += Receive_TXData[i];
			}
		Receive_TXData[Receive_TX_Len - 2] = (Sum & 0x00FF);//求和校验
		Receive_TXData[Receive_TX_Len - 1] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}
///////////////////////625//////////////////
//		Receive_TXData[36] = Send_gai_Status;			//粗跟踪模式
//		Receive_TXData[37] = Shangweiji_Command;  //粗跟踪指令
//		Receive_TXData[38] = Send_Status;					//粗跟踪指令执行状态
//		Receive_TXData[39] = FY_Xianwei;					//俯仰限位
//		Receive_TXData[40] = Send_Err1;						//粗伺服故障代码1
//		Receive_TXData[41] = Send_Err2;						//粗伺服故障代码2
//		
	
//		Receive_TXData[78] = Send_CAMERA_Err; //惯导通信状态	
		Send_JiXiaData_Write();
}




//粗伺服发给上位机的数据  2022.8.16
void Send_SW_Data_TX(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//帧头
	
//	Temp_32_Receive.Receive_Data_float = (FW_encoder_degrees);
	Temp_32_Receive.Receive_Data_float = miss_distance_X_float * 15.7;
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[2] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Temp_32_Receive.Receive_Data_float = (FY_encoder_degrees );
	Temp_32_Receive.Receive_Data_float = miss_distance_Y_float * 15.7;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.Low_byte;

/*	
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{
//		case :                          ////////////指向
//			cugenzongmoshi = 15;
//			break;
//		case :                           ////////////扫描
//			cugenzongmoshi = 16;
//			break;
//		case :                       ///////////////跟踪修正
//			cugenzongmoshi = 17;
//			break;
//		case :                       ///////////////归零
//			cugenzongmoshi = 18;
//			break;
//		case :                 //////////////使能
//			cugenzongmoshi = 19;
//			break;
//		case :                 //////////////停止
//			cugenzongmoshi = 20;
//			break;
//		case :                     /////////////闭环
//			cugenzongmoshi = 21;
//			break;
//		case :                  ////////////////目标外引导
//			cugenzongmoshi = 25;
//		  break;
//		case :             ///////////////////收藏
//			cugenzongmoshi = 26;
//			break;
//		case :          //////////////////展开 
//			cugenzongmoshi = 27;
//			break;
//		case :                     ///////////粗跟踪曲线切换
//			cugenzongmoshi = 30;
//			break;
//		case :                  //////////////粗精伺服状态查询
//			cugenzongmoshi = 33;
//		  break;
		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{
//		case 15:                          ////////////指向
//			cugenzongzhiling = 15;
//			break;
//		case 16:                           ////////////扫描
//			cugenzongzhiling = 16;
//			break;
//		case 17:                       ///////////////跟踪修正
//			cugenzongzhiling = 17;
//			break;
//		case 18:                       ///////////////归零
//			cugenzongzhiling = 18;
//			break;
//		case 19:                 //////////////使能
//			cugenzongzhiling = 19;
//			break;
//		case 20:                 //////////////停止
//			cugenzongzhiling = 20;
//			break;
//		case 21:                     /////////////闭环
//			cugenzongzhiling = 21;
//			break;
//		case 25:                  ////////////////目标外引导
//			cugenzongzhiling = 25;
//		  break;
//		case 26:             ///////////////////收藏
//			cugenzongzhiling = 26;
//			break;
//		case 27:          //////////////////展开 
//			cugenzongzhiling = 27;
//			break;
//		case 30:                     ///////////粗跟踪曲线切换
//			cugenzongzhiling = 30;
//			break;
//		case 33:                  //////////////粗精伺服状态查询
//			cugenzongzhiling = 33;
//		  break;
		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// 惯导///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
*/

//	Temp_32_Receive.Receive_Data_float = z_axis_velocity_float;		//方位速度
//	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	
////	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
//	Temp_32_Receive.Receive_Data_float = x_axis_velocity_float; 	// 俯仰速度
//	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	
////	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
//	Temp_32_Receive.Receive_Data_float = y_axis_velocity_float;		//横滚速度
//	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
////  Receive_TXData[31] = 0x00;
////  Receive_TXData[32] = 0x00;
////	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
////	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
////	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
////	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
//	Receive_TXData[31] = 0;
	//		//粗跟踪伺服脱靶量X
//		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float;
		    FW_Jixia_tuoba_degrees = FW_encoder_degrees;
		Temp_32_Receive.Receive_Data_float = FW_Jixia_tuoba_degrees;
		Receive_TXData[13] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[12] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[11] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//粗跟踪伺服脱靶量Y
//		FY_Jixia_tuoba_degrees = x_axis_velocity_1msdistance_sum;
		FY_Jixia_tuoba_degrees = FY_encoder_degrees;
		Temp_32_Receive.Receive_Data_float = FY_Jixia_tuoba_degrees;
		Receive_TXData[17] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[16] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[15] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[14] = Temp_32_Receive.Receive_Data_byte.High_byte;
		
//	  for(int i=2;i<33;i++)
//		{
//				Receive_TXData[i] = 0;
//		}
			Receive_TXData[36]++;
//		Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
			Sum = 0;
			for(i = 2; i < 37; i++)
			{
				Sum += Receive_TXData[i];
			}
//		Receive_TXData[34] = (Sum & 0x00FF);//求和校验
			Receive_TXData[37] = Sum;
		Receive_TXData[38] = 0xB5;
		Receive_TXData[39] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}	
	
//		Receive_TXData[78] = Send_CAMERA_Err; //惯导通信状态	
		Send_SW_Data_Write();
}




//单摆镜发送给机上总控数据
void Send_Data_TX_SPM(void)
{
		u16 i=0;
		u8 Sum=0;
	unsigned char Temp_Uchar = 0;
	Receive_TXData[0] = 0xEB;
	Receive_TXData[1] = 0x90;//帧头
	Receive_TXData[2] = 35;
	Temp_32_Receive.Receive_Data_float = (FW_encoder_degrees);
//	Temp_32_Receive.Receive_Data_float = FW_location_set;
	Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	Temp_32_Receive.Receive_Data_float = (FY_encoder_degrees );
//	Temp_32_Receive.Receive_Data_float = FY_location_set;
	Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	if(POS_SP_SWITCH == 1)
	{
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FW * 1000.0f);
		Receive_TXData[6] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[5] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[4] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[3] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Temp_32_Receive.Receive_Data_float = -(int)(Speed_encode_FY * 1000.0f);
		Receive_TXData[10] = Temp_32_Receive.Receive_Data_byte.High_byte;
		Receive_TXData[9] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[8] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[7] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	}
	if(system_mode == 4)
	{
		Temp_Uchar = 1;
	}
	else if(system_mode == 2 )
	{
		Temp_Uchar = 0;
	}
	Receive_TXData[11] = Temp_Uchar;
	switch(system_mode)
	{
//		case :                          ////////////指向
//			cugenzongmoshi = 15;
//			break;
//		case :                           ////////////扫描
//			cugenzongmoshi = 16;
//			break;
//		case :                       ///////////////跟踪修正
//			cugenzongmoshi = 17;
//			break;
//		case :                       ///////////////归零
//			cugenzongmoshi = 18;
//			break;
//		case :                 //////////////使能
//			cugenzongmoshi = 19;
//			break;
//		case :                 //////////////停止
//			cugenzongmoshi = 20;
//			break;
//		case :                     /////////////闭环
//			cugenzongmoshi = 21;
//			break;
//		case :                  ////////////////目标外引导
//			cugenzongmoshi = 25;
//		  break;
//		case :             ///////////////////收藏
//			cugenzongmoshi = 26;
//			break;
//		case :          //////////////////展开 
//			cugenzongmoshi = 27;
//			break;
//		case :                     ///////////粗跟踪曲线切换
//			cugenzongmoshi = 30;
//			break;
//		case :                  //////////////粗精伺服状态查询
//			cugenzongmoshi = 33;
//		  break;
		default:
			break;
	}
	Receive_TXData[12] = cugenzongmoshi;
  switch(Shangweiji_Command)
	{
//		case 15:                          ////////////指向
//			cugenzongzhiling = 15;
//			break;
//		case 16:                           ////////////扫描
//			cugenzongzhiling = 16;
//			break;
//		case 17:                       ///////////////跟踪修正
//			cugenzongzhiling = 17;
//			break;
//		case 18:                       ///////////////归零
//			cugenzongzhiling = 18;
//			break;
//		case 19:                 //////////////使能
//			cugenzongzhiling = 19;
//			break;
//		case 20:                 //////////////停止
//			cugenzongzhiling = 20;
//			break;
//		case 21:                     /////////////闭环
//			cugenzongzhiling = 21;
//			break;
//		case 25:                  ////////////////目标外引导
//			cugenzongzhiling = 25;
//		  break;
//		case 26:             ///////////////////收藏
//			cugenzongzhiling = 26;
//			break;
//		case 27:          //////////////////展开 
//			cugenzongzhiling = 27;
//			break;
//		case 30:                     ///////////粗跟踪曲线切换
//			cugenzongzhiling = 30;
//			break;
//		case 33:                  //////////////粗精伺服状态查询
//			cugenzongzhiling = 33;
//		  break;
		default:
			break;	
	}		
	Receive_TXData[13] = cugenzongzhiling;
	Receive_TXData[14] = 0;
	Temp_Uchar = 0;
	Receive_TXData[15] = 0;
	Receive_TXData[16] = 0;
	Receive_TXData[17] = 0;
	Receive_TXData[18] = 0;
/////////////////// 惯导///////////////////////////
//	Temp_32_Receive.Receive_Data_float = FY_output_value_da;
	if(FY_encoder_degrees == 0||FY_encoder_degrees == 360)
	{
		tt++;
	}
	Temp_32_Receive.Receive_Data_float = z_axis_velocity_float;		//方位速度
	Receive_TXData[22] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[21] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[20] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[19] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao2;		//*Deg_2_1
	Temp_32_Receive.Receive_Data_float = x_axis_velocity_float; 	// 俯仰速度
	Receive_TXData[26] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[25] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[24] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[23] = Temp_32_Receive.Receive_Data_byte.Low_byte;
	
//	Temp_32_Receive.Receive_Data_float = jiao4;   //*Deg_1_1
	Temp_32_Receive.Receive_Data_float = y_axis_velocity_float;		//横滚速度
	Receive_TXData[30] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[29] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
	Receive_TXData[28] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
	Receive_TXData[27] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//  Receive_TXData[31] = 0x00;
//  Receive_TXData[32] = 0x00;
//	Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.Low_byte;
//	Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
//	Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
//	Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.High_byte;
	Receive_TXData[31] = 0;
	//		//粗跟踪伺服脱靶量X
		FW_Jixia_tuoba_degrees = MissKx*miss_distance_X_float*15.7;
//		FW_Jixia_tuoba_degrees = jiao1;
		Temp_32_Receive.Receive_Data_float = FW_Jixia_tuoba_degrees;
		Receive_TXData[32] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[33] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[34] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[35] = Temp_32_Receive.Receive_Data_byte.High_byte;
		//粗跟踪伺服脱靶量Y
		FY_Jixia_tuoba_degrees = MissKy*miss_distance_Y_float*15.7;
//				FY_Jixia_tuoba_degrees = jiao3;
		Temp_32_Receive.Receive_Data_float = FY_Jixia_tuoba_degrees;
		Receive_TXData[36] = Temp_32_Receive.Receive_Data_byte.Low_byte;
		Receive_TXData[37] = Temp_32_Receive.Receive_Data_byte.MLow_byte;
		Receive_TXData[38] = Temp_32_Receive.Receive_Data_byte.MHigh_byte;
		Receive_TXData[39] = Temp_32_Receive.Receive_Data_byte.High_byte;
	  Receive_TXData[Receive_TX_Len - 3] = Send_XunHuan_Ma_TX;
	  for(i = 2; i < (Receive_TX_Len - 2); i++)
			{
				Sum += Receive_TXData[i];
			}
		Receive_TXData[Receive_TX_Len - 2] = (Sum & 0x00FF);//求和校验
		Receive_TXData[Receive_TX_Len - 1] = 0xFE;
		Send_XunHuan_Ma_TX++;
		if(Send_XunHuan_Ma_TX > 255)
		{
			Send_XunHuan_Ma_TX = 0;
		}
///////////////////////625//////////////////
//		Receive_TXData[36] = Send_gai_Status;			//粗跟踪模式
//		Receive_TXData[37] = Shangweiji_Command;  //粗跟踪指令
//		Receive_TXData[38] = Send_Status;					//粗跟踪指令执行状态
//		Receive_TXData[39] = FY_Xianwei;					//俯仰限位
//		Receive_TXData[40] = Send_Err1;						//粗伺服故障代码1
//		Receive_TXData[41] = Send_Err2;						//粗伺服故障代码2
//		
	
//		Receive_TXData[78] = Send_CAMERA_Err; //惯导通信状态	
		Send_JiXiaData_Write();
}


//粗伺服接收机上总控板的数据（粗伺服板与机上总控板通信）串口2
void Receive_JiShangData_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR1;
	for(Receive_i=0; Receive_i < Receive_RX_Len; Receive_i++)
	{
		Receive_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	Receive_DATA_Parse_2th();
}


//粗伺服发给机下总控板的数据（粗伺服板与机下总控板通信）
void Send_JiXiaData_Write()
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR1;     //0800x
	for(Receive_i=0; Receive_i < Receive_TX_Len; Receive_i++)
	{
		*(uint32_t*)(Receive_ADDR_tmp)=Receive_TXData[Receive_i];
	}
	*(uint32_t*)(Receive_Com_En)= 1;
}

//粗伺服板接收机上位机的数据（粗伺服板与上位机通信）
void Receive_ShangweijiData_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = Receive_ADDR3;
	for(Receive_i=0; Receive_i < Shangweiji_RX_Len; Receive_i++)
	{
		Shangweiji_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	Receive_Shangweiji_DATA_Parse();
}

//粗伺服发给上位机的数据   2022.8.16
void Send_SW_Data_Write()
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
//	Receive_ADDR_tmp = Receive_ADDR1;     //0800x
	for(Receive_i=0; Receive_i < 40; Receive_i++)
	{
		*(uint32_t*)(0x64000800)=Receive_TXData[Receive_i];
	}
	*(uint32_t*)(0x64000C00)= 1;
}
void Receive_Suidong_GS()
{
	
u8 i=0,Sum=0;	
if((Receive_DBJ_Data[0]==0xEB) & (Receive_DBJ_Data[1]==0x90))//帧头
		{
			if(system_mode == 25)
	    {GS_Suidong_Flag=0;}
			if(system_mode == 27)
	    {GS_Suidong_Flag=1;}
				if((Receive_DBJ_Data[12]==0xFF)&(GS_Suidong_Flag==1))//帧尾
				{
						Sum=0;
				for(i=0;i<(13-2);i++)
					 {
						Sum+=Receive_DBJ_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_DBJ_Data[13-2]==(Sum))//求和校验
			
						{	
//								touluo_True ++;	//数据正确
							  Suidong_jishu++;
								Receive_suidong_IP = Receive_DBJ_Data[10];
						}
//						else
//						{
//			     Suidong_jishu++;			
//						}
						
           if(Receive_suidong_IP == 14||Receive_suidong_IP == 25)/////////////////粗跟踪伺服指向/////
							{
									  system_mode = 17;
										
										//粗跟踪伺服方位
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[2];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[3];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[4];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[5];
										GS_Suidong_FY = Temp_32_Receive.Receive_Data_float; 
										Shangweiji_Direction_Angle_prepass = FsmLeadLag1(GS_Suidong_FY,FSM_X,0);
								
										FY_Degree_GS = -Shangweiji_Direction_Angle_prepass+GS_Suidong_offset_FY; 							//*360/2/3.14/1000/1000;//urad转°;
								    
										//粗跟踪伺服俯仰
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[6];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[7];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[8];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[9];
										GS_Suidong_FW = Temp_32_Receive.Receive_Data_float; 
							    	Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(GS_Suidong_FW,FSM_Y,0);
								
										FW_Degree_GS = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW; 							//*360/2/3.14/1000/1000;//urad转°;
								    GS_Suidong_chafen_FW=GS_Suidong_FW_before-Shangweiji_Pitch_Angle_Value_prepass;
								    GS_Suidong_chafen_FY=GS_Suidong_FY_before-Shangweiji_Direction_Angle_prepass;

								GS_Suidong_FW_before=Shangweiji_Pitch_Angle_Value_prepass;
								GS_Suidong_FY_before=Shangweiji_Direction_Angle_prepass;
							}
							
}

				}

			
			}

void Receive_Deadscale_Suidong_GS()
{
	
u8 i=0,Sum=0;	
if((Receive_DBJ_Data[0]==0xEB) & (Receive_DBJ_Data[1]==0x90))//帧头
		{
//			if(system_mode == 25)
//	    {GS_Suidong_Flag=0;}
//			if(system_mode == 27)
//	    {GS_Suidong_Flag=1;}
				if((Receive_DBJ_Data[12]==0xFF))//帧尾
				{
						Sum=0;
				for(i=0;i<(13-2);i++)
					 {
						Sum+=Receive_DBJ_Data[i];
					 }
//					 Sum=Sum%65536;
					 //Sum=0;
					if(Receive_DBJ_Data[13-2]==(Sum))//求和校验
			
						{	
//								touluo_True ++;	//数据正确
							  Suidong_jishu++;
								Receive_suidong_IP = Receive_DBJ_Data[10];
						}
//						else
//						{
//			     Suidong_jishu++;			
//						}
						
           if(Receive_suidong_IP == 14||Receive_suidong_IP == 25||Receive_suidong_IP == 17)/////////////////粗跟踪伺服指向/////
							{
//									  system_mode = 17;
										
										//粗跟踪伺服方位
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[2];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[3];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[4];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[5];
										GS_Suidong_FY = Temp_32_Receive.Receive_Data_float; 
										Shangweiji_Direction_Angle_prepass = FsmLeadLag1(GS_Suidong_FY,FSM_X,0);
								
										FY_Degree_GS_suidong = -Shangweiji_Direction_Angle_prepass+GS_Suidong_offset_FY; 							//*360/2/3.14/1000/1000;//urad转°;
								    
										//粗跟踪伺服俯仰
										Temp_32_Receive.Receive_Data_byte.Low_byte = Receive_DBJ_Data[6];
										Temp_32_Receive.Receive_Data_byte.MLow_byte = Receive_DBJ_Data[7];
										Temp_32_Receive.Receive_Data_byte.MHigh_byte = Receive_DBJ_Data[8];
										Temp_32_Receive.Receive_Data_byte.High_byte = Receive_DBJ_Data[9];
										GS_Suidong_FW = Temp_32_Receive.Receive_Data_float; 
							    	Shangweiji_Pitch_Angle_Value_prepass = FsmLeadLag1(GS_Suidong_FW,FSM_Y,0);
                    FW_Degree_GS_suidong = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW;								
//										FW_Degree_GS_suidong = Shangweiji_Pitch_Angle_Value_prepass*2+GS_Suidong_offset_FW; 							//*360/2/3.14/1000/1000;//urad转°;
//								    GS_Suidong_chafen_FW=GS_Suidong_FW_before-Shangweiji_Pitch_Angle_Value_prepass;
//								    GS_Suidong_chafen_FY=GS_Suidong_FY_before-Shangweiji_Direction_Angle_prepass;

//								GS_Suidong_FW_before=Shangweiji_Pitch_Angle_Value_prepass;
//								GS_Suidong_FY_before=Shangweiji_Direction_Angle_prepass;
							}
							
}

				}
			}
void Receive_DBJsuidong_Read()	
{
	u8 Receive_i=0;
	u32 Receive_ADDR_tmp;
	Receive_ADDR_tmp = (u32)(0x64003000);
	for(Receive_i=0; Receive_i < Receive_Suidong_GS_Len; Receive_i++)
	{
		Receive_DBJ_Data[Receive_i]=*(uint32_t*)(Receive_ADDR_tmp);
		Receive_ADDR_tmp = Receive_ADDR_tmp + 2;
	}
	if(system_mode == 25)
	{
	Receive_Deadscale_Suidong_GS();
	}
	else
	{
	Receive_Suidong_GS();
	}
}			