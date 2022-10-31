#include "stm32f4xx.h"//
#include "aaa.h"
#include "stmflash.h"
#include "ioinit.h"
#include "AttitudeAlgorithm.h"
/////////////600000=400ms
//#define CJ_T 25
//#define CJ_Len 8
//#define AA_ADDR_422        ((u32)(0x64006000))     //串口读取地址              422地址
//#define AA_Com_En_422      ((u32)(0x64006800))     //串口发送使能地址          422地址
#define AB_ADDR        ((u32)(0x6400B100))

#define AA_ADDR        ((u32)(0x64004000))     //串口读取地址                LVDS地址
#define AA_Com_En      ((u32)(0x64004800))     //串口发送使能地址            LVDS使能
#define AB_Com_En      ((u32)(0x64000000))     //串口发送使能地址  //0x6400B900
//#define AA_ADDR        ((u32)(0x64006000))     //串口读取地址                422地址
//#define AA_Com_En      ((u32)(0x64006800))     //串口发送使能地址            422使能

#define Control_FineComReadADDR          ((u32)(0x64003000))//精跟踪协议读取地址
#define Fine_ControlComWriteADDR         ((u32)(0x64003000))//精跟踪协议写入地址
#define Fine_ControlComWriteEnable       ((u32)(0x64003800))//精跟踪协议写入使能


u8 AB_i=0,AB_TX_Len;
u8 AB_TXData[100];     //发送数据
extern float NewAngle;
int16_t ArrayToInt161(char c1, char c2)
{    
	union Int16ToArray1 fta;
	fta.ary[0] = c1;    
	fta.ary[1] = c2;    
	
	return fta.k;
}
extern u8 PC_TXData[],JiTing_Set_CNT;
extern u8 AA_Flage; 
extern int Scan_Direction_ly;
union FloatToArray Temp_Data;//
extern float ppp,yyy;
float TX_Coarse_Azimuth=0;//粗跟踪伺服方位
float TX_Coarse_Pitch=0;//粗跟踪伺服俯仰
float TX_Precise_Azimuth=0;//精跟踪伺服方位
float TX_Precise_Pitch=0;//精跟踪伺服俯仰

float RX_Coarse_Azimuth=0;//粗跟踪伺服方位
float RX_Coarse_Pitch=0;//粗跟踪伺服俯仰
float RX_Precise_Azimuth=0;//精跟踪伺服方位
float RX_Precise_Pitch=0;//精跟踪伺服俯仰

float FW_Miss_distance;//方位脱靶量
float FY_Miss_distance;//俯仰脱靶量

float RX_Coarse_Azimuth1=0;//粗跟踪光轴补偿方位角度
float RX_Coarse_Pitch1=0;//粗跟踪光轴补偿俯仰 角度
float RX_Precise_Azimuth1=0;//精跟踪光轴补偿方位角度
float RX_Precise_Pitch1=0;//精跟踪光轴补偿俯仰角度
/*
float TX_Coarse_Azimuth1=0;//粗跟踪伺服方位
float TX_Coarse_Pitch1=0;//粗跟踪伺服俯仰
float TX_Precise_Azimuth1=0;//精跟踪伺服方位
float TX_Precise_Pitch1=0;//精跟踪伺服俯仰
*/
//惯导的俯仰角度信息（InsPitchAngle）、横滚角度信息（InsRollAngle）
float TX_Ins_RollAngle=0;//惯性空间横滚
float TX_Ins_PitchAngle=0;//惯性空间俯仰
float TX_Ins_Yaw=0;//惯性空间偏航
/////采集命令
float TX_Ins_Attitude_FW=0;//惯导姿态方位
float TX_Ins_Attitude_FY=0;//惯导姿态俯仰
float TX_Ins_Attitude_Roll=0;//惯导姿态横滚
float TX_Ins_Speed_FW=0;//惯导角速度方位
float TX_Ins_Speed_FY=0;//惯导角速度俯仰
float TX_Ins_Speed_Roll=0;//惯导角速度横滚

float TX_Encoder_FW=0;//编码器方位
float TX_Encoder_FY=0;//编码器俯仰
float TX_Miss_FY=0;//脱靶量方位
float TX_Miss_FW=0;//脱靶量俯仰
float TX_Fast_Reflection_Mirror_FW=0;//快反镜方位
float TX_Fast_Reflection_Mirror_FY=0;//快反镜俯仰


//float TX_Precise_Pitch=0;//精跟踪伺服俯仰
float FW_position;//方位位置
float FY_position;//俯仰位置

u8 Scan_direction,Scan_times;//扫描方向，扫描次数
s8 Scan_FW_Angle,Scan_FY_Angle;//扫描方位角，扫描俯仰角
s8 Scan_FW_Start_Angle;
s8 Scan_FY_Start_Angle;
int16_t FW_Scan_Step; //方位扫描步长
int16_t FY_Scan_Step; // 俯仰扫描步长
float FW_Scan_Step_Float=0;
float FY_Scan_Step_Float=0;

u8 Scan_Speed; //扫描速度
u8 RX_Servo_Steady_State;//伺服稳态
u8 TX_Servo_Steady_State;//伺服稳态
u8 TX_Coarse_State;  ///粗跟踪状态字
u8 TX_Precise_State; ///精跟踪状态字
u8 Point_Mode; //指向模式
u16 Step_T;//步进间隔时间，
u16 FW_Step_Angle;//方位每次步进角；
u16 FY_Step_Angle;//俯仰每次步进角
u32 Stable_T; ///稳定时间
s8 FW_Angle_MAX1,FY_Angle_MAX1;//扫描最大方位俯仰角1
int16_t FW_Angle_MAX2,FY_Angle_MAX2;//扫描最大方位 俯仰角2
u8 Primary_Mirror_Work_Mode;//主反射镜工作模式
u8 Fast_Mirror_Work_Mode;//快反镜工作模式

u32 AA_Time=0;
u8 AA_Flage=0;
u8 AA_i=0;
u8 AA_frame;
u8 AA_TX_Len,AA_RX_Len;//数据长度
u32 AA_ADDR_tmp;       //
u8 AA_TXData[100];     //发送数据
u8 AA_Data[100];       //接收数据
u8 AA_Err,AA_fail;   
u8 CommandNew,CommandOld,CO_ID,ST_ID,Work_Mode=0X03;//命令 设备ID 工作模式
u8 Time_T=10,Time_M=7,Time_S=1,Time_mS_H=1,Time_mS_L=10;//分系统时间
u8 SYS_Time_T,SYS_Time_M,SYS_Time_S,SYS_Time_mS_H,SYS_Time_mS_L; //  系统时间
u8 Driver_Set1,Driver_Set2,Driver_Set3,Driver_Set4,Driver_Set5,Driver_Set6;//伺服驱动器设置命令
u8 Driver_Set_Mode;///伺服驱动器设置方式

u8 Work_Mode_Old =0;
u8 Clearn_flag = 0;

//下位机使用参数
extern u8 system_mode;
extern float FW_pointing_degree_set;
extern float FY_pointing_degree_set; 

extern float FY_encoder_degrees;  //俯仰编码器转换为角度
extern float FW_encoder_degrees;


extern float miss_distance_X_float;     
extern float miss_distance_Y_float;

extern float FY_miss_distance_sub;
extern float FW_miss_distance_sub;


extern float yaw_angle_float_slave;      //小惯导偏航

extern float pitch_angle_float_slave;    //小惯导俯仰

extern float roll_angle_float_slave;     //小惯导横滚


extern float X_axis_palstance_float;     //小惯导角速度
extern float Y_axis_palstance_float;     //小惯导角速度
extern float Z_axis_palstance_float;     //小惯导角速度

extern float roll_angle_float_compensation;

extern float sin_value ;

extern float X_axis_palstance_float;
extern float X_axis_palstance_float_lowpass;

extern float Y_slave_1ms_distance_sum;

extern float X_axis_palstance_float_lowpass_out;

extern float qiankui_speed;

extern float qiankui_speed_lowpass;

extern float simulated_target_motion;


u8 guandao_yaw_select = 0;
u8 guandao_pitch_select = 0;
u8 guandao_roll_select = 0;

extern float FW_location_loop_error; 
extern float FY_location_loop_error;      //设定值 - 编码器值 

extern float simulated_target_motion;

extern float IMU_bit_float;

extern float miss_distance_X_float_to_angle;
extern float miss_distance_Y_float_to_angle;

u32 SYS_Time = 0;
u32 Time = 0;

extern float X_slave_1ms_distance_sum;
extern float Z_slave_1ms_distance_sum;

extern float FW_wending_qiankui_speed_lowpass;  //方位前馈速度低通滤波
extern float FW_location_loop_error_sub;    //设定值与编码器值误差

extern float FW_moni_miss;
extern float FY_moni_miss;

//正式光纤陀螺速度
extern float x_axis_velocity_float;
extern float z_axis_velocity_float;
extern float y_axis_velocity_float;
//正式光纤陀螺姿态
extern float yaw_attitude_float;
extern float pitch_attitude_float;
extern float roll_attitude_float;

//脱靶量图形
extern float FW_send_array[10];
extern float FY_send_array[10];
extern u8 FW_send_array_select;
extern u8 FY_send_array_select;

float FW_Huiling_last = 0;
float FY_Huiling_last = 0;


//自检标志位
extern int FY_encoder_Flag;
extern int FW_encoder_Flag;
extern int GuangxianGuandao_Flag;
extern float temp1_f;
extern float temp2_f;


extern float FW_zero_degree_zheng;
extern float FY_zero_degree;
extern float FY_light_loop_revise;
extern float FW_light_loop_revise;


//与精跟踪通信
uint8_t  ControlFineTracking_Protocol[46];//总控精跟踪通信协议
uint8_t  Read_JiaoYanHouShuJu[46];//校验后存储数据
uint8_t  DaBaoFaSong_Data[41]={0};//给精跟踪发送数据打包
uint16_t Sum_Data=0;
uint16_t HeJiaoYan=0;
int Read_JiaoYan_Flag=0;
uint16_t ZhenJiShu=0;
uint16_t ZhenJiShu_GaoWei=0;
uint16_t Sum_Send_Data=0;
uint16_t Sun_Send_Data_GaoWei=0;

extern float DianLiu_FW;
extern float DianLiu_FY;
s8 DianLiu_2 = 0;
s8 DianLiu_1 = 0;

extern uint16_t fpgaUpdateFlag;
uint8_t a[41];
int Send_Flag = 0;
u8 Scan_FangShi = 0;

//参数装订
u8 PZdataWX[38];

int bbb = 0;


extern u8 step_open_flag;
extern int	FW_Scan_StartAngle;                    //方位扫描起始角度
extern int ScanAngleRang;                        //扫描范围
extern float	FW_ScanStepLength;                      //扫描步长
extern int Scan_Direction;                        //判断扫描方向
extern u8 Give_Scan_Number;

extern int Time_Long ;                            //间隔时间
int Recevie = 0;
extern int FY_Scan_StartAngle;
extern float FY_ScanAngleRang;                      //俯仰扫描范围

extern float FW_yunsu_sudu;
extern float FY_yunsu_sudu ;
extern float FW_yunsu_sudu_1;
int QIUhe_CNT = 0;

extern float SpaceSetValueFW;
extern float SpaceSetValueFY;

extern float ScanAngleRang_ly;                        //扫描范围
extern float FW_ScanStepLength_ly;                      //扫描步长
extern float FW_YunSuStepLength_ly;                      //扫描步长
float Scan_Speed_Degree_s = 0;
extern float YunSuAngleRang_ly;  

extern float KongJianZhiXiang_FW;
extern float KongJianZhiXiang_FY;
extern int WaiTongBu_ScanAngleRang;                        //扫描范围
extern float WaiTongBu_FW_ScanStepLength;   

extern float KongJianZhiXiang_FWLY;
extern float KongJianZhiXiang_FYLY;

extern float KongJianPoint_FW;
extern float KongJianPoint_FY;

extern float KongJianZhiXiang_FWLY1;
extern float KongJianZhiXiang_FYLY1;

extern int WaiTongBu_Scan_Direction;                        //判断扫描方向

extern float KongJianZhiXiang_FWYunSu;
extern float KongJianZhiXiang_FYYunSu;
extern int YunSu_Direction_ly;

//////////
extern  float   AN_1_fy;
extern  float   AN_2_fy;
extern  float   AN_3_fy;
extern  float   AN_4_fy;
extern  float   PX_fy;
extern  float   AN_1_fw;
extern  float   AN_2_fw;
extern  float   AN_3_fw;
extern  float   AN_4_fw;
extern  float   PX_fw;
extern  float   VX_fw;
extern  float   VX_fy;
extern int32_t   SO_fw;
extern int32_t   EE1_fw;
extern int32_t   EC_fw;
extern int32_t   MF_fw;
extern int32_t   SR_fw;
extern int32_t   SO_fy;
extern int32_t   EE1_fy;
extern int32_t   EC_fy;
extern int32_t   MF_fy;
extern int32_t   SR_fy;

extern float FW_pointing_degree_LY;
extern float FY_pointing_degree_LY;
extern float Point_FY_set_prepass;
u8 AB_CNT=1;

extern u8 MoShiQieHuan;

extern u8 EXIT_POINT_FLAG;
extern float WaiTongBu_step_design_set;
extern u8 WaiTongBu_Start_cnt;
extern float WaiTongBu_PanFangXiang;
extern int WaiTongBu_Flag_Com_Direction;                    //方向只赋一次值
extern u32 WaiTongBu_MaiChong_cnt;
extern u32  EXTI2_CNT;
extern u8 ZhiXiang_Complete_FlagLY;
extern u8 EXIT_LianXu_FLAG;
extern u8 EXIT_Song;
extern u8 JiTing_ZhongDuan_CNT;

u8 QieHuan_flag = 1;
extern u8 GuiLingFlag;
extern float Space_FY;
extern float Space_FW;
extern u16 ZhenPin;
u8 ly_zhenpin = 0;
int zhenpin_CNT1 = 0;
int zhenpin_CNT2 = 0;
extern float WaiTongBu_FY_ScanAngleRang;
extern u8 BeginMove_flag;
extern u8 Zhuang_Right;
extern u8 Zhuang_Left;

float AryToFloat(char c1, char c2, char c3, char c4)     //将4个字节转换成单精度浮点
{    
	union FloatToArray fta;
	fta.ary[0] = c1;    
	fta.ary[1] = c2;    
	fta.ary[2] = c3;    
	fta.ary[3] = c4;    
	return fta.f;
}
u32 tx_cnt;
void AA_Com_Read()      //①判断标志位后读取FPGA内字节
{
	AA_ADDR_tmp=AA_ADDR;
	AA_Data[5]=0;
	AA_i=0;
    while (1)
	{
		AA_Data[AA_i]=*(uint32_t*)(AA_ADDR_tmp);
		AA_ADDR_tmp = AA_ADDR_tmp+2;
		AA_RX_Len=AA_Data[5]+9;
		AA_i++;
		if(AA_i==AA_RX_Len) 
		break;
	}
	Recevie++;
	AA_DATA_();        //②判断帧头帧尾和校验
}

void AA_Com_Write()    //串口发送
{
	//union UintToArray ia;
	//ia.k = 300;
	for(AA_i=0; AA_i<AA_TX_Len; AA_i++)
	{
		*(uint32_t*)(AA_ADDR)= AA_TXData[AA_i];
	}
	Delay_us(1);
	*(uint32_t*)(AA_Com_En)= 1;
	AA_fail=0X01;
	tx_cnt++;
}

void AA_DATA_()     //②判断帧头帧尾和校验
{
	u8 ii=0,Sum2;
	u32 Sum1=0;
	if((AA_Data[0]==0x5a) & (AA_Data[1]==0x54))//帧头
	{
		if((AA_Data[AA_RX_Len-2]==0x5a)&(AA_Data[AA_RX_Len-1]==0xfe))//帧尾
		{
			for(ii=6;ii<(AA_RX_Len-3);ii++)   
			{
				Sum1+=AA_Data[ii];
			}
			Sum2=Sum1&0xff;   //sum1 = 各数据字节累加
			Sum2=256-Sum2;    //sum2 = 校验字节
			
			if(AA_Data[AA_RX_Len-3]==Sum2)//求和校验
			{
				if(AA_Data[2]==0x10&AA_Data[3]==0x17)
				{							
					 CommandNew=AA_Data[4];
					 CO_ID=AA_Data[2];
					 ST_ID=AA_Data[3];
					 SYS_Time=AA_Data[6]+AA_Data[7]*256+AA_Data[8]*256*256+AA_Data[9]*256*256*256;//系统时间。时
//                     Time=SYS_Time;
//					 SYS_Time_T=AA_Data[6];//系统时间。时
//					 SYS_Time_M=AA_Data[7];//系统时间。分
//					 SYS_Time_S=AA_Data[8];//系统时间   秒
//					 SYS_Time_mS_H=AA_Data[9];//系统时间  毫秒
//					 SYS_Time_mS_L=AA_Data[10];//系统时间 毫秒
					 
//					 Time_T=SYS_Time_T;//系统时间。时
//			         Time_M=SYS_Time_M;//系统时间。分
//					 Time_S=SYS_Time_S;//系统时间   秒
//					 Time_mS_L=SYS_Time_mS_L;//系统时间  毫秒
//					 Time_mS_H=SYS_Time_mS_H;//系统时间 毫秒
		
					 AA_Err=0;
					 AA_Data_CL();        //③帧头帧尾和校验都通过后执行这个解析程序;
				}
				else  //设备ID错误√
				{
					 AA_TXData[0]=0X5A;//帧头
					 AA_TXData[1]=0X54;//帧头
					 AA_TXData[2]=0X17;//发送设备 0x17
					 AA_TXData[3]=0X10;//接收设备0x10
					 AA_TXData[4]=0XE4;//命令码ST
					 AA_TXData[5]=0X07;//数据长度SL
					
					 AA_TXData[6]=Time&0x000000ff;//系统时间。时
			     AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
			     AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
			     AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒
					
					 /////数据
					 AA_TXData[10]=0x17;       //当前设备ID
					 AA_TXData[11]=CO_ID;      //命令源ID
					 AA_TXData[12]=ST_ID;      //目的ID
	
					 for(ii=6;ii<13;ii++)//校验
					 {
						Sum1+=AA_TXData[ii];
					 }
					 Sum2=256-Sum1&0xff;
					
					 AA_TXData[13]=256-Sum2;//校验
					 AA_TXData[14]=0X5A;
					 AA_TXData[15]=0XFE;
					 AA_TX_Len=16;	
					 AA_Com_Write();					
				
				}
			}
			else  //校验和错误√
			{
				 AA_Err=1;
				 AA_TXData[0]=0X5A;//帧头
				 AA_TXData[1]=0X54;//帧头
				 AA_TXData[2]=0X17;//发送设备 0x17
				 AA_TXData[3]=0X10;//接收设备0x10
				 AA_TXData[4]=0XE0;//命令码ST
				 AA_TXData[5]=0X07;//数据长度SL
				
				 AA_TXData[6]=Time&0x000000ff;
			   AA_TXData[7]=Time>>8&0x000000ff;
			   AA_TXData[8]=Time>>16&0x000000ff;
			   AA_TXData[9]=Time>>24&0x000000ff;
				 //数据
	       AA_TXData[10]=AA_Data[AA_RX_Len-3];    //接收到的验证码值
				 AA_TXData[11]=Sum2;                    //从站生成的验证码值
				 
				 AA_TXData[12]=0XAA;                    //备用
				 for(ii=6;ii<9;ii++)
				 {
					Sum1+=AA_TXData[ii];
				 }
				 Sum2=256-Sum1&0xff;
				 AA_TXData[13]=256-Sum2;          
				 AA_TXData[14]=0X5A;
				 AA_TXData[15]=0XFE;
	             AA_TX_Len=16;	
	             AA_Com_Write();						
			}
		}
		else  //帧尾错误
		{    
			 AA_Err=2;
			 AA_TXData[0]=0X5A;//帧头
			 AA_TXData[1]=0X54;//帧头
			 AA_TXData[2]=0X17;//发送设备 0x17
			 AA_TXData[3]=0X10;//接收设备0x10
			 AA_TXData[4]=0XE5;//命令码ST                                 ？
			 AA_TXData[5]=0X07;//数据长度SL
			
			 AA_TXData[6]=Time&0x000000ff;
			 AA_TXData[7]=Time>>8&0x000000ff;
			 AA_TXData[8]=Time>>16&0x000000ff;
			 AA_TXData[9]=Time>>24&0x000000ff;
			 /////数据
			 AA_TXData[10]=AA_Data[AA_RX_Len-3];
			 AA_TXData[11]=Sum2;
			 
			 AA_TXData[12]=0XAA;//备用
             for(ii=6;ii<(13);ii++)//校验
			 {
				Sum1+=AA_TXData[ii];
			 }
			 Sum2=256-Sum1&0xff;
			 AA_TXData[13]=Sum2;//校验
			 AA_TXData[14]=0X5A;
			 AA_TXData[15]=0XFE;
			 AA_TX_Len=16;	
			 AA_Com_Write();						
		}
	}
	else   //帧头错误
	{
		AA_Err=3;
		AA_TXData[0]=0X5A;//帧头
		AA_TXData[1]=0X54;//帧头
		AA_TXData[2]=0X17;//发送设备 0x17
		AA_TXData[3]=0X10;//接收设备0x10
		AA_TXData[4]=0XE0;//命令码ST                                ？
		AA_TXData[5]=0X03;//数据长度SL
		 /////数据
		AA_TXData[6]=AA_Data[AA_RX_Len-3];
		AA_TXData[7]=Sum2;
		 
		AA_TXData[8]=0XAA;//备用
		for(ii=6;ii<(13);ii++)//校验
		{
			Sum1+=AA_TXData[ii];
		}
		Sum2=256-Sum1&0xff;
		AA_TXData[9]=Sum2;//校验
		AA_TXData[10]=0X5A;
		AA_TXData[11]=0XFE;
		AA_TX_Len=12;	
		AA_Com_Write();						
	}

}

/////////////////////////////////////////↓↓↓解析数据包↓↓↓///////////////////////////////////////////////////////////////////
void AA_Data_CL(void)       //③帧头帧尾和校验都通过后执行这个解析程序;
{
	QIUhe_CNT++;
	u8 i=0,Sum2=0;
	u32 Sum=0;
	union UintToArray T;///
	switch(CommandNew)
	{
//自检		
		case 0x30://自检
		{
			self_check();
	        ZiJian_YingDa();
			break;
		}
//参数装订
		case 0x31://参数装订
		{
			if(Work_Mode==0x03)    //Work_Mode==0x03为待机状态，只有在待机状态下才可以参数装订
			{
				parameter_record();
				Send_Flag = 1;
				
			}
			else                   //如果不是在待机情况下收到参装订指令，返回错误代码
			{
				parameter_record_error();	    //？错误回啥			
			}
			break;
		}
//状态查询，回复一些状态量		
		case 0x32://状态查询，回复一些状态量
		{
			state_inquiry();
			
			ZhuangTaiChaXun_YingDa();
			break;
		}
//工作模式切换		
		case 0x33://工作模式切换
		{
			if(CommandOld==0x39)       //工作模式切换前必须先发急停
			{
				Work_Mode=AA_Data[10];  //工作模式
				if(Work_Mode_Old == Work_Mode)
				{
					Clearn_flag = 1;                //两次工作模式相同不清楚标志位
				}
				else
				{
					Clearn_flag = 0;               //两次工作模式不同清楚标志位
				}
				if(Clearn_flag == 0)
				{
					/////////////////////清除外同步缓存/////////////////////////////////	
			        BeginMove_flag = 0;
					Zhuang_Right = 0;
                    Zhuang_Left = 0;
					
					
					
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
				}
				Work_Mode_Old = Work_Mode;
				
				FW_position=AryToFloat(AA_Data[11],AA_Data[12],AA_Data[13],AA_Data[14]);//方位位置
				FY_position=AryToFloat(AA_Data[15],AA_Data[16],AA_Data[17],AA_Data[18]);//俯仰位置
				Scan_direction=AA_Data[19];//扫描方向
				Scan_Direction_ly = Scan_direction;
				WaiTongBu_Scan_Direction = Scan_direction;
				YunSu_Direction_ly = Scan_direction;
				Scan_times=AA_Data[20];    //  扫描次数Give_Scan_Number
				Give_Scan_Number = Scan_times;
				Scan_FW_Start_Angle=AA_Data[21]; //扫描起始方位角
				Scan_FY_Start_Angle=AA_Data[22]; //，扫描起始俯仰角
				Scan_FW_Angle=AA_Data[23]; //扫描方位角
				Scan_FY_Angle=AA_Data[24]; //，扫描俯仰角
				
				FW_Scan_Step=(AA_Data[26]*256) + AA_Data[25];     //扫描步长
				FW_Scan_Step_Float=(float)FW_Scan_Step/1000;
				
				FY_Scan_Step=(AA_Data[28]*256) + AA_Data[27];     //扫描步长
				FY_Scan_Step_Float=(float)FY_Scan_Step/1000;
//				FW_Scan_Step=ArrayToInt161(AA_Data[25],AA_Data[26]);     //扫描步长
//				FY_Scan_Step=ArrayToInt161(AA_Data[27],AA_Data[28]);     //扫描步长
				Scan_Speed=AA_Data[29];        //扫描速度
//				FW_yunsu_sudu = (float)Scan_Speed;
//				FY_yunsu_sudu = Scan_Speed;
				Scan_FangShi=AA_Data[30];     //扫描方式 
				if( (Scan_FangShi != 0 && Work_Mode == 0x01) ||(Scan_FangShi != 0 && Work_Mode == 0x05))      //内同步扫描，外同步扫描，匀速扫描
				{
					if((Scan_FW_Start_Angle<-60)||(Scan_FW_Start_Angle>60)) ////扫描方位角度判断
					{
						scan_FW_degree_judge();
						break;///					 
					}	
					if((Scan_FY_Start_Angle<-60)||(Scan_FY_Start_Angle>60))/////扫描俯仰角度判断	
					{
						scan_FY_degree_judge();
						break;///						 
					}
					//////取数
					KongJianZhiXiang_FWLY = (float)Scan_FW_Start_Angle;              
					KongJianZhiXiang_FYLY = (float)Scan_FY_Start_Angle;
					
					KongJianZhiXiang_FWLY1 = (float)Scan_FW_Start_Angle;
					KongJianZhiXiang_FYLY1 = (float)Scan_FY_Start_Angle;
					
					KongJianZhiXiang_FWYunSu = (float)Scan_FW_Start_Angle;
					KongJianZhiXiang_FYYunSu = (float)Scan_FY_Start_Angle;
					
					MoShiQieHuan = 1;
//					FW_Scan_StartAngle = (int)Scan_FW_Start_Angle - (int)FW_zero_degree_zheng;
//					FY_Scan_StartAngle = (int)Scan_FY_Start_Angle;
				}
				
				if((Scan_FangShi != 0 && Work_Mode == 0x01)||(Scan_FangShi != 0 && Work_Mode == 0x05))      //内同步扫描，外同步扫描，匀速扫描
				{
					if((Scan_FW_Angle<0)||(Scan_FW_Angle>60)) ////扫描方位角度判断
					{
						scan_FW_degree_judge_1();
						break;///					 
					}	
					if((Scan_FY_Angle<0)||(Scan_FY_Angle>60))/////扫描俯仰角度判断	
					{
						scan_FY_degree_judge_1();
						break;///						 
					}
					//////取数
					ScanAngleRang_ly = (float)Scan_FW_Angle;
					ScanAngleRang_ly = ScanAngleRang_ly * 0.5;//范围是正负走
					YunSuAngleRang_ly = (float)Scan_FW_Angle;
					
					if((Scan_FW_Angle % 2) == 0)
					{
						WaiTongBu_ScanAngleRang = (int)Scan_FW_Angle;
					}
					if((Scan_FW_Angle % 2) == 1)
					{
						WaiTongBu_ScanAngleRang = (int)Scan_FW_Angle + (int)1;					
					}
					
					

					WaiTongBu_ScanAngleRang = WaiTongBu_ScanAngleRang * 0.5; //范围是正负走
//					ScanAngleRang = (int)Scan_FW_Angle;
				}			
				
				if((Scan_FangShi != 0 && Work_Mode == 0x01) || (Scan_FangShi != 0 && Work_Mode == 0x05))   //内同步扫描，外同步扫描
				{
					if((FW_Scan_Step_Float<0.1)||(FW_Scan_Step_Float>10))////方位扫描步长判断
					{
						scan_FW_step_judge();
						break;///							 
					}
					if((FY_Scan_Step_Float<0.1)||(FY_Scan_Step_Float>10))///俯仰扫描步长判断
					{
						scan_FY_step_judge();
						break;///						 
					}
					//////取数
					FW_ScanStepLength_ly = FW_Scan_Step_Float;
					WaiTongBu_FW_ScanStepLength = FW_Scan_Step_Float;
					WaiTongBu_FY_ScanAngleRang = FY_Scan_Step_Float;
//					FW_ScanStepLength = FW_Scan_Step_Float;
//					FY_ScanAngleRang = FY_Scan_Step_Float;
				}
				if(Scan_FangShi == 0x03 && Work_Mode == 0x01)     //匀速扫描
				{  
					if((Scan_Speed<10)||(Scan_Speed>60))///扫描速度	
					{
						scan_speed();
						break;///						 
					}
				//////取数
					Scan_Speed_Degree_s = Scan_Speed * 0.001;
					FW_YunSuStepLength_ly = Scan_Speed_Degree_s;                      //度/1ms
//					FW_yunsu_sudu = (float)Scan_Speed * 0.058;
//					FW_yunsu_sudu_1 = (float)Scan_Speed * 0.058;
				}
	


				//应答报文         //正确应答
				work_mode_change_ack();
			}
			
			else   //工作模式切换前必须先发急停
			{
				work_mode_change_error();
			}
			break;//
		}
//启动指向		
		case 0x34://启动指向
		{	
			if(Work_Mode==0x03 ||Work_Mode==0x04)   //在待机模式下或人工引导模式
			{
				start_point_to();
			    ZhiXiang_YingDa();
//				system_mode = 12;       // ★    				
			}		
			else   //在非指向工作模式下，启动指向
			{
				start_point_to_error();	
			}		
			break;
		}
//启动步进扫描		
		case 0x35://启动步进扫描
		{
			if(Work_Mode==0x01|| Work_Mode==0x03||Work_Mode==0x05)// 扫描，边扫边跟踪，待机
			{
				if(Scan_FangShi == 0x01)                     //内同步扫描
				{
					start_step_scan();
					BuJinSaoMiao_YingDa();
					system_mode = 11;
//					system_mode = 5;          // ★
//					step_open_flag = 1;
				}
				if(Scan_FangShi == 0x02)                   //外同步扫描
				{
					start_step_scan();
					BuJinSaoMiao_YingDa();
					system_mode = 13;//////////////////9
				}

			}
			else
			{
				start_step_scan_error();	
			}		
			break;
		}
//启动跟踪		
		case 0x36://启动跟踪
		{
			if(Work_Mode==0x02 || Work_Mode==0x03)   //处于跟踪模式或待机
			{
				start_track();
				GenZong_YingDa();
				if(QieHuan_flag == 1)
				{
					system_mode = 7; 
				}
			}
			else                  //处于非跟踪模式
            {
				start_track_error();
			}				
			break;
		}
//启动匀速扫描
		case 0x37://启动匀速扫描
		{
			if(Work_Mode==0x01 || Work_Mode==0x03 || Work_Mode==0x05)   // 处于扫描模式、待机、边扫边跟踪
			{
//				system_mode = 8;
				system_mode = 12;
				start_constantspeed_scan();
				YunSuSaoMiao_YingDa();
			}
			else                  // 处于非匀速扫描模式
			{
				start_constantspeed_scan_error();			
			}		
			break;
		}
//归零		
		case 0x38://归零
		{
			if(CommandOld == 0x39)
			{
//				if(Work_Mode==0x03)   //处于待机模式
//				{
					GuiLingFlag = 0;
					FW_Huiling_last = FW_encoder_degrees;
					FY_Huiling_last = FY_encoder_degrees;				
					system_mode = 3; // ★
					make_zero();
					HuiLing_YingDa();
//				}
//			  else                  //处于非待机模式
//        {
//					make_zero_error();	
//		  	}				
				break;
			}
			else   //工作模式切换前必须先发急停
			{
				work_mode_change_error();
			}
			break;//
		}
//急停		
		case 0x39://急停
		{ 
			system_mode = 1;  
			scram_button();	
			JiTing_YingDa();
			JiTing_ZhongDuan_CNT = 0;
//			JiTing_Set_CNT=0;
			break;
		}
//驱动设置		
		case 0x3a://驱动设置
		{	
			if(Work_Mode==0x03)     //处于待机模式 
			{
				driver_setting();
			}
			else                    //处于非待机模式 
			{
				driver_set_error();	
			}				
			
			break;
		}
//惯导设置
		case 0x3b://惯导设置
		{
			if(Work_Mode==0x03)     //处于待机模式    
			{
				navigation_set();
			}
			else                    //处于非待机模式
            {
				navigation_set_error();	
			}				
			break;
		}	
//采集命令
		case 0x3c://采集命令
		{
			bbb++;
			//			if((Work_Mode==0x01)||(Work_Mode==0x02)||(Work_Mode==0x05))///扫描或跟踪
//			{
			collecte_command();
			CaiJiMingLing_YingDa();
		
//			}
//			else
//      {
//				collecte_command_error();	
//			}				
			break;
		}
//对时		
		case 0x3d://对时
		{	
			check_the_time();
			break;
		}
//通讯测试		
		case 0x3e://通讯测试
		{	
//			if(Work_Mode==0x03)   //待机模式下可发送通信测试命令   
//			{
				communication_test();

//			}
//			else
//      {
//				communication_test_error();
//			}				
			break;
		}
		default:
		{	 
			switch_default();
			break;
		}	
		
	}//switch反括号

}
void AA_Data_TX(void)
{
	AA_Com_Write();
}

/////////////////////////////////////////↑↑↑解析数据↑↑↑////////////////////////////////////////////////////////////////////////////
//急停
void scram_button()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //自检模式
	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
	CommandOld=CommandNew;	
}

//归零
void make_zero()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x00;          //回零模式
	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
	CommandOld=CommandNew;
}
void make_zero_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
    AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
    //数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x37;
		 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;     //校验字节
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void start_point_to()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	Point_Mode=AA_Data[10];////指向模式
	RX_Coarse_Azimuth=AryToFloat(AA_Data[11],AA_Data[12],AA_Data[13],AA_Data[14]);//粗跟踪伺服方位位置
	//SYS_Time_T=AA_Data[12];//
	//SYS_Time_M=AA_Data[13];//
	//SYS_Time_S=AA_Data[14];//
	//SYS_Time_mS_H=AA_Data[15];//
	RX_Coarse_Pitch=AryToFloat(AA_Data[15],AA_Data[16],AA_Data[17],AA_Data[18]);//粗跟踪伺服俯仰位置
	//SYS_Time_mS_L=AA_Data[16];//
	//SYS_Time_T=AA_Data[17];//
	//SYS_Time_M=AA_Data[18];//
	//SYS_Time_S=AA_Data[19];//

	RX_Precise_Azimuth=AryToFloat(AA_Data[19],AA_Data[20],AA_Data[21],AA_Data[22]);//精跟踪伺服方位位置
	//SYS_Time_mS_H=AA_Data[20];//
	//SYS_Time_mS_L=AA_Data[21];//
	//SYS_Time_T=AA_Data[22];//?
	//SYS_Time_M=AA_Data[23];//

	RX_Precise_Pitch=AryToFloat(AA_Data[23],AA_Data[24],AA_Data[25],AA_Data[26]);//精跟踪伺服俯仰位置
	//SYS_Time_S=AA_Data[24];//
	//SYS_Time_mS_H=AA_Data[25];//
	//SYS_Time_mS_L=AA_Data[26];//
	//SYS_Time_mS_L=AA_Data[27];//
	//=AA_Data[27];//备份
	
//	FW_pointing_degree_set = RX_Coarse_Azimuth - FW_zero_degree_zheng; // ★
//	FY_pointing_degree_set = RX_Coarse_Pitch - FY_zero_degree;   // ★
	
//	SpaceSetValueFW = RX_Coarse_Azimuth;
//	SpaceSetValueFY = RX_Coarse_Pitch;
	KongJianZhiXiang_FW = RX_Coarse_Azimuth;
    KongJianZhiXiang_FY = RX_Coarse_Pitch;
	if(Point_Mode==0x01)      //仅粗跟踪指向
	{
	//	system_mode=4;
	  	system_mode=14;
		
		DaBao_Send_Data();
		DaBaoFaSong_Data[35] = 0X05;                //精跟踪自检模式
		ZhenJiShu++;
		ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
		DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
		DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
		Sum_Send_Data = 0;
		for(i=2;i<38;i++)
		{
			Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
		}
		Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
		DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
		DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
		write_JingGenZong_Data();            //发给精跟踪	
	}
	
	if(Point_Mode==0x02)     //仅精跟踪指向
	{
//		system_mode=1;
		DaBao_Send_Data();
		DaBaoFaSong_Data[19] = AA_Data[19];         //精跟踪伺服方位位置
		DaBaoFaSong_Data[20] = AA_Data[20];
		DaBaoFaSong_Data[21] = AA_Data[21];
		DaBaoFaSong_Data[22] = AA_Data[22];
		
		DaBaoFaSong_Data[23] = AA_Data[23];         //精跟踪伺服俯仰位置
		DaBaoFaSong_Data[24] = AA_Data[24];
		DaBaoFaSong_Data[25] = AA_Data[25];
		DaBaoFaSong_Data[26] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X01;                //精跟踪指向模式
		
		
		ZhenJiShu++;
		ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
		DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
		DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
		Sum_Send_Data = 0;
		for(i=2;i<38;i++)
		{
			Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
		}
		Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
		DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
		DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
		write_JingGenZong_Data();            //发给精跟踪		
	}
	
  if(Point_Mode==0x03)     //粗/精跟踪指向
	{
		system_mode=14;
	//	system_mode=4;
		DaBao_Send_Data();
		DaBaoFaSong_Data[19] = AA_Data[19];         //精跟踪伺服方位位置
		DaBaoFaSong_Data[20] = AA_Data[20];
		DaBaoFaSong_Data[21] = AA_Data[21];
		DaBaoFaSong_Data[22] = AA_Data[22];
		
		DaBaoFaSong_Data[23] = AA_Data[23];         //精跟踪伺服俯仰位置
		DaBaoFaSong_Data[24] = AA_Data[24];
		DaBaoFaSong_Data[25] = AA_Data[25];
		DaBaoFaSong_Data[26] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X01;                //精跟踪指向模式
		ZhenJiShu++;
		ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
		DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
		DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
		Sum_Send_Data = 0;
		for(i=2;i<38;i++)
		{
			Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
		}
		Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
		DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
		DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
		write_JingGenZong_Data();            //发给精跟踪	
	}
	
	if(Point_Mode==0x04)     //精跟踪校准补偿
	{
		system_mode=1;
		DaBao_Send_Data();
		DaBaoFaSong_Data[11] = AA_Data[19];         //精跟踪伺服方位位置
		DaBaoFaSong_Data[12] = AA_Data[20];
		DaBaoFaSong_Data[13] = AA_Data[21];
		DaBaoFaSong_Data[14] = AA_Data[22];
		
		DaBaoFaSong_Data[15] = AA_Data[23];         //精跟踪伺服俯仰位置
		DaBaoFaSong_Data[16] = AA_Data[24];
		DaBaoFaSong_Data[17] = AA_Data[25];
		DaBaoFaSong_Data[18] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X03;                //精跟踪校准补偿
		ZhenJiShu++;
		ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
		DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
		DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
		Sum_Send_Data = 0;
		for(i=2;i<38;i++)
		{
			Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
		}
		Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
		DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
		DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
		write_JingGenZong_Data();            //发给精跟踪		
	}
	if(Point_Mode==0x05)      //粗跟踪编码器指向
	{
	  	system_mode=17;
		
		FW_pointing_degree_LY = RX_Coarse_Azimuth;
		Point_FY_set_prepass = RX_Coarse_Pitch;
		//FY_pointing_degree_set = RX_Coarse_Pitch;
		DaBao_Send_Data();
		DaBaoFaSong_Data[35] = 0X05;                //精跟踪自检模式
		ZhenJiShu++;
		ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
		DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
		DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
		Sum_Send_Data = 0;
		for(i=2;i<38;i++)
		{
			Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
		}
		Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
		DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
		DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
		write_JingGenZong_Data();            //发给精跟踪	
	}
	
		CommandOld=CommandNew;
}
void start_point_to_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
			 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x34;

	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void self_check()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //自检模式
	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
	CommandOld=CommandNew;
	
	
	
	
}
void parameter_record()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	union UintToArray T;//
	
	//数组到浮点
	RX_Coarse_Azimuth=AryToFloat(AA_Data[10],AA_Data[11],AA_Data[12],AA_Data[13]);//粗跟踪伺服方位 回零位置
	//SYS_Time_T=AA_Data[11];//
	//SYS_Time_M=AA_Data[12];//
	//SYS_Time_S=AA_Data[13];//
	//SYS_Time_mS_H=AA_Data[14];//
	RX_Coarse_Pitch=AryToFloat(AA_Data[14],AA_Data[15],AA_Data[16],AA_Data[17]);//粗跟踪伺服俯仰 回零位置
	//SYS_Time_mS_L=AA_Data[15];//
	//SYS_Time_T=AA_Data[16];//
	//SYS_Time_M=AA_Data[17];//
	//SYS_Time_S=AA_Data[18];//

	RX_Precise_Azimuth=AryToFloat(AA_Data[18],AA_Data[19],AA_Data[20],AA_Data[21]);//精跟踪伺服方位 回零位置
	//SYS_Time_mS_H=AA_Data[19];//
	//SYS_Time_mS_L=AA_Data[20];//
	//SYS_Time_T=AA_Data[21];//?
	//SYS_Time_M=AA_Data[22];//

	RX_Precise_Pitch=AryToFloat(AA_Data[22],AA_Data[23],AA_Data[24],AA_Data[25]);//精跟踪伺服俯仰 回零位置
	//SYS_Time_S=AA_Data[23];//
	//SYS_Time_mS_H=AA_Data[24];//
	//SYS_Time_mS_L=AA_Data[25];//
	//SYS_Time_mS_L=AA_Data[26];//

	RX_Coarse_Azimuth1=AryToFloat(AA_Data[26],AA_Data[27],AA_Data[28],AA_Data[29]);//安装倾斜角
	//SYS_Time_T=AA_Data[27];//
	//SYS_Time_M=AA_Data[28];//
	//SYS_Time_S=AA_Data[29];//
	//SYS_Time_mS_H=AA_Data[30];//

	RX_Coarse_Pitch1=AryToFloat(AA_Data[30],AA_Data[31],AA_Data[32],AA_Data[33]);//粗跟踪光轴补偿俯仰 角度
	//SYS_Time_mS_L=AA_Data[31];//
	//SYS_Time_T=AA_Data[32];//
	//SYS_Time_M=AA_Data[33];//
	//SYS_Time_S=AA_Data[34];//

	RX_Precise_Azimuth1=AryToFloat(AA_Data[34],AA_Data[35],AA_Data[36],AA_Data[37]);//精跟踪光轴补偿方位角度
	//SYS_Time_mS_H=AA_Data[35];//
	//SYS_Time_mS_L=AA_Data[36];//
	//SYS_Time_T=AA_Data[37];//
	//SYS_Time_M=AA_Data[38];//

	RX_Precise_Pitch1=AryToFloat(AA_Data[38],AA_Data[39],AA_Data[40],AA_Data[41]);//精跟踪光轴补偿俯仰角度
	//SYS_Time_S=AA_Data[39];//
	//SYS_Time_mS_H=AA_Data[40];//
	//SYS_Time_mS_L=AA_Data[41];//
	//SYS_Time_mS_L=AA_Data[41];//

	Step_T=AA_Data[42]+AA_Data[43]*256;//步进间隔时间
	FW_Step_Angle=AA_Data[44]+AA_Data[45]*256;//方位每次步进角
	FY_Step_Angle=AA_Data[46]+AA_Data[47]*256;//俯仰每次步进角

	T.ary[0]=AA_Data[48];//
	T.ary[1]=AA_Data[49];// 
	T.ary[2]=AA_Data[50];//
	T.ary[3]=AA_Data[51];//
	Stable_T=T.k;              //联合体ary to uint

	FW_Angle_MAX1=AA_Data[52];//扫描最大方位角1
	FW_Angle_MAX2=AA_Data[53];//扫描最大方位角2

	//
	FY_Angle_MAX1=AA_Data[54];////扫描最大俯仰角1
	FY_Angle_MAX2=AA_Data[55];////扫描最大俯仰角2

	
	////////////////////////////写FLASH参数////////////////////////////////////////////////////////////////////
	PZdataWX[0] = AA_Data[10];                 //粗跟踪伺服方位 回零位置
	PZdataWX[1] = AA_Data[11];
	PZdataWX[2] = AA_Data[12];
	PZdataWX[3] = AA_Data[13];
	
	PZdataWX[4] = AA_Data[14];                 //粗跟踪伺服俯仰 回零位置
	PZdataWX[5] = AA_Data[15];
	PZdataWX[6] = AA_Data[16];
	PZdataWX[7] = AA_Data[17];
	
	PZdataWX[8] = AA_Data[26];                 //倾斜角装订
	PZdataWX[9] = AA_Data[27];
	PZdataWX[10] = AA_Data[28];	
	PZdataWX[11] = AA_Data[29];		

	PZdataWX[12] = AA_Data[30];                //粗跟踪光轴补偿俯仰 角度
	PZdataWX[13] = AA_Data[31];
	PZdataWX[14] = AA_Data[32];	
	PZdataWX[15] = AA_Data[33];		
 	
	PZdataWX[16] = AA_Data[42];                //步进间隔时间
	PZdataWX[17] = AA_Data[43];
	
	PZdataWX[18] = AA_Data[44];	              //方位每次步进角
	PZdataWX[19] = AA_Data[45];	
	
	PZdataWX[20] = AA_Data[46];	              //俯仰每次步进角
	PZdataWX[21] = AA_Data[47];	
	
	PZdataWX[22] = AA_Data[48];	              //稳定时间
	PZdataWX[23] = AA_Data[49];	
	PZdataWX[24] = AA_Data[50];	
	PZdataWX[25] = AA_Data[51];	
	
	PZdataWX[26] = AA_Data[52];	              //扫描最大方位角1
	PZdataWX[27] = AA_Data[53];	              //扫描最大方位角2
	
	PZdataWX[28] = AA_Data[54];	              //扫描最大俯仰角1
	PZdataWX[29] = AA_Data[55];		          //扫描最大俯仰角2	
	
	PZdataWX[30] = AA_Data[18];	              //精跟踪伺服方位 回零位置
	PZdataWX[31] = AA_Data[19];		           	
	PZdataWX[32] = AA_Data[20];	              
	PZdataWX[33] = AA_Data[21];		            
	
	PZdataWX[34] = AA_Data[22];	              //精跟踪伺服俯仰 回零位置
	PZdataWX[35] = AA_Data[23];		           	
	PZdataWX[36] = AA_Data[24];	              
	PZdataWX[37] = AA_Data[25];		  
	

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	//{赋值加此处}
//	FW_zero_degree_zheng = RX_Coarse_Azimuth;            //粗跟踪伺服方位 回零位置
//	FY_zero_degree = RX_Coarse_Pitch;                    //粗跟踪伺服俯仰 回零位置
//	FY_light_loop_revise = RX_Coarse_Pitch1;             // 粗跟踪光轴补偿俯仰角度
//	FW_light_loop_revise = RX_Coarse_Azimuth1;           // 粗跟踪光轴补偿方位角度
    Time_Long	=	(int)Step_T;                      //步进间隔时间
    FW_ScanStepLength =    (float)FW_Step_Angle * (0.001);        //方位每次步进角度
    FY_ScanAngleRang  =    (float)FY_Step_Angle * (0.001);        //俯仰每次步进角度
		NewAngle = RX_Coarse_Azimuth1;
//    FW_Scan_StartAngle =    FW_zero_degree_zheng;       //扫描起始角度
//	ScanAngleRang	=	(int)FW_Angle_MAX2 - (int)FW_Angle_MAX1;//扫描范围


	
	DaBao_Send_Data();
	DaBaoFaSong_Data[3]  = AA_Data[18];                   //精跟踪伺服方位 回零位置
	DaBaoFaSong_Data[4]  = AA_Data[19];
	DaBaoFaSong_Data[5]  = AA_Data[20];
	DaBaoFaSong_Data[6]  = AA_Data[21];
	
	DaBaoFaSong_Data[7]  = AA_Data[22];                   //精跟踪伺服俯仰 回零位置
	DaBaoFaSong_Data[8]  = AA_Data[23];
	DaBaoFaSong_Data[9]  = AA_Data[24];
	DaBaoFaSong_Data[10] = AA_Data[25];

	DaBaoFaSong_Data[11]  = AA_Data[34];                   //精跟踪光轴补偿方位角度
	DaBaoFaSong_Data[12]  = AA_Data[35];
	DaBaoFaSong_Data[13]  = AA_Data[36];
	DaBaoFaSong_Data[14]  = AA_Data[37];

	DaBaoFaSong_Data[15]  = AA_Data[38];                   //精跟踪光轴补偿俯仰角度
	DaBaoFaSong_Data[16]  = AA_Data[39];
	DaBaoFaSong_Data[17]  = AA_Data[40];
	DaBaoFaSong_Data[18]  = AA_Data[41];

	DaBaoFaSong_Data[35]  = 0X04;                         //参数装订模式
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪


	
	//应答///
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X41;    //参数装订应答
	AA_TXData[5]=0X05;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
	
	AA_TXData[10]=0X00;//装订成功0XAA失败0XEE        ☆
	//AA_TXData[11]=0XAA;//备用
    //AA_TXData[12]=0XAA;//备用

	for(i=6;i<(11);i++)    //各数据字节累加（i比字节索引大1）
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;    //校验字节
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	STMFLASH_Write(FLASH_SAVE_ADDR,(u32*)PZdataWX,38);
	CommandOld=CommandNew;
}
void parameter_record_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
    AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE1;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
	 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x31;
	 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();		
}
void state_inquiry()
{
	u8 i=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          
	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
	CommandOld=CommandNew;
}
void start_step_scan()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //自检模式
	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
	CommandOld=CommandNew;
}
void start_step_scan_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	
			 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x35;
			
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void start_track()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x02;          //跟踪模式
	
	ZhenJiShu++;
  ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪		
	CommandOld=CommandNew;
}
void start_track_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
    AA_TXData[6]=Time&0x000000ff;//系统时间。时
    AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
    AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
    AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	
			 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x36;
			 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void start_constantspeed_scan()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //自检模式
	
	ZhenJiShu++;
  ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪	
		CommandOld=CommandNew;
}

void start_constantspeed_scan_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
    AA_TXData[6]=Time&0x000000ff;//系统时间。时
    AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
    AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
    AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒 
		 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x35;
		
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();			
}
void driver_setting()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	Driver_Set_Mode=AA_Data[11];///驱动器设置方式
	Driver_Set1=AA_Data[12];///伺服驱动器设置命令字
	Driver_Set2=AA_Data[13];///伺服驱动器设置命令字
	Driver_Set3=AA_Data[14];///伺服驱动器设置命令字
	Driver_Set4=AA_Data[15];///伺服驱动器设置命令字
	Driver_Set5=AA_Data[16];///伺服驱动器设置命令字
	Driver_Set6=AA_Data[17];///伺服驱动器设置命令字
	//=AA_Data[18];///备份

	///应答///
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4a;
	AA_TXData[5]=5;
	
    AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=0XAA;//备份
	for(i=6;i<(11);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;//校验
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;		
}
void driver_set_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=7;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
		 //数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x39;
		 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //校验字节
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void navigation_set()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	/*///惯导设置命令字
	Driver_Set_Mode=AA_Data[11];///驱动器设置方式
	Driver_Set1=AA_Data[12];///伺服驱动器设置命令字
	Driver_Set2=AA_Data[13];///伺服驱动器设置命令字
	Driver_Set3=AA_Data[14];///伺服驱动器设置命令字
	Driver_Set4=AA_Data[15];///伺服驱动器设置命令字
	Driver_Set5=AA_Data[16];///伺服驱动器设置命令字
	Driver_Set6=AA_Data[17];///伺服驱动器设置命令字
	//=AA_Data[18];///备份
	*/

	//惯导应答
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4b;    //惯导设置应答
	AA_TXData[5]=5;
	
    AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=0XAA;//粗跟踪状态字
	for(i=6;i<(11);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;     //校验字节
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}
void navigation_set_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=7;//数据长度SL
	
    AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
    //数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x3a;
		 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //校验字节
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void collecte_command()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	FW_Miss_distance=AryToFloat(AA_Data[10],AA_Data[11],AA_Data[12],AA_Data[13]);//方位脱靶量
	FY_Miss_distance=AryToFloat(AA_Data[14],AA_Data[15],AA_Data[16],AA_Data[17]);//俯仰脱靶量
//	RX_Servo_Steady_State=AA_Data[19];///伺服稳态

    ly_zhenpin++;
    if(ly_zhenpin == 1)
	{
		zhenpin_CNT1 = ZhenPin;
	}		
	if(ly_zhenpin == 2)
	{
		zhenpin_CNT2 = ZhenPin;
	}	
	if(ly_zhenpin > 5)
	{
		ly_zhenpin = 5;
	}
	
	
	DaBao_Send_Data();
	DaBaoFaSong_Data[27] = AA_Data[10];     //方位
	DaBaoFaSong_Data[28] = AA_Data[11];
	DaBaoFaSong_Data[29] = AA_Data[12];
	DaBaoFaSong_Data[30] = AA_Data[13];
	
	DaBaoFaSong_Data[31] = AA_Data[14];     //俯仰
	DaBaoFaSong_Data[32] = AA_Data[15];
	DaBaoFaSong_Data[33] = AA_Data[16];
	DaBaoFaSong_Data[34] = AA_Data[17];
	
	if(CommandNew == 0x36)
	{
		DaBaoFaSong_Data[35] = 0x02;          //采集模式
	}
	else
	{
		DaBaoFaSong_Data[35] = 0x06;          //采集模式
	}

	
	ZhenJiShu++;
	ZhenJiShu_GaoWei = ZhenJiShu&0xFF00;
	DaBaoFaSong_Data[36] = ZhenJiShu_GaoWei>>8;
	DaBaoFaSong_Data[37] = ZhenJiShu&0xFF;
	Sum_Send_Data = 0;
	for(i=2;i<38;i++)
	{
		Sum_Send_Data = Sum_Send_Data + DaBaoFaSong_Data[i];
	}
	Sun_Send_Data_GaoWei = Sum_Send_Data&0xFF00;
	DaBaoFaSong_Data[38] = Sun_Send_Data_GaoWei>>8;
	DaBaoFaSong_Data[39] = Sum_Send_Data&0xFF;
	write_JingGenZong_Data();            //发给精跟踪		
	CommandOld=CommandNew;
}
void collecte_command_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=7;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒		 
			 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x3b;
			 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	

}
void check_the_time()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
  if(AA_Data[10]==0xca)Time=SYS_Time;	
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4d;
	AA_TXData[5]=5;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=AA_Data[10];//粗跟踪状态字

	for(i=6;i<(11);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;    //校验字节
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}

void communication_test()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4e;     //通信测试应答
	AA_TXData[5]=5;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=0X00;    //响应标识0xaa成功发送0xee失败
//	AA_TXData[11]=0XAA;
//	AA_TXData[12]=0XAA;
	for(i=6;i<(11);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;   //校验字节
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;
	
}
void communication_test_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE1;//命令码ST
	AA_TXData[5]=7;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
	//数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0XAA;
			 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //校验字节
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void switch_default()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE1;//命令码ST
	AA_TXData[5]=7;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒
			 /////数据
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=CommandNew;
			 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void scan_FW_degree_judge()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
			 /////数据
	AA_TXData[10]=22;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FW_degree_judge_1()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
			 /////数据
	AA_TXData[10]=24;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FY_degree_judge_1()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
			 /////数据
	AA_TXData[10]=25;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FY_degree_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	
	
		 /////数据
	AA_TXData[10]=23;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_FW_step_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
	
    /////数据
	AA_TXData[10]=26;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_FY_step_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒	 
	
		 /////数据
	AA_TXData[10]=28;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_speed()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE6;//命令码ST
	AA_TXData[5]=0X07;//数据长度SL
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒 
	
				 /////数据
	AA_TXData[10]=30;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//备用
	for(i=6;i<(13);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//校验
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void work_mode_change_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;
	AA_TXData[0]=0X5A;//帧头
	AA_TXData[1]=0X54;//帧头
	AA_TXData[2]=0X17;//发送设备 0x17
	AA_TXData[3]=0X10;//接收设备0x10
	AA_TXData[4]=0XE2;//命令码ST
	AA_TXData[5]=0X03;//数据长度SL
			 /////数据
	AA_TXData[6]=AA_Data[11];
	AA_TXData[7]=CommandOld;
			 
	AA_TXData[8]=0XAA;//备用
	for(i=6;i<(9);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[9]=Sum2;//校验
	AA_TXData[10]=0X5A;
	AA_TXData[11]=0XFE;
	AA_TX_Len=12;	
	AA_Com_Write();	
}

void work_mode_change_ack()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X43;
	AA_TXData[5]=0X05;
	
	AA_TXData[6]=Time&0x000000ff;//系统时间。时
	AA_TXData[7]=Time>>8&0x000000ff;//系统时间。分
	AA_TXData[8]=Time>>16&0x000000ff;;//系统时间   秒
	AA_TXData[9]=Time>>24&0x000000ff;;//系统时间  毫秒
	
	AA_TXData[10]=0XAA;//
	for(i=6;i<(11);i++)//校验
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;//校验
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}


//读取精跟踪反馈数据
void read_JingGenZong_Data()      
{
	uint32_t ControlFPGAReadADDR=0;
	uint8_t i;
	ControlFPGAReadADDR=Control_FineComReadADDR;
	for(i=0;i<46;i++)
	{
		ControlFineTracking_Protocol[i]=*(uint32_t*)(ControlFPGAReadADDR);
		ControlFPGAReadADDR+=2;
	}		
}
extern uint8_t  DaBaoFaSong_Data[41];
//给精跟踪传数据
void write_JingGenZong_Data()
{
	uint8_t i;
	uint32_t FPGAControlWriteADDR;
	FPGAControlWriteADDR=Fine_ControlComWriteADDR;
	for(i=0;i<41;i++)
	{
		*(uint32_t*)(FPGAControlWriteADDR)=DaBaoFaSong_Data[i];
	}
	*(uint32_t*)(Fine_ControlComWriteEnable)=1;	
}	

//校验数据包
void JiaoYan_Read_Data()
{
	if(ControlFineTracking_Protocol[0]==0xEB||ControlFineTracking_Protocol[1]==0x90)
	{
		if(ControlFineTracking_Protocol[45]==0xFE)
		{
			uint8_t i_ly;
			uint8_t i_l;
			for(i_ly=2;i_ly<43;i_ly++)
			{
				Sum_Data = Sum_Data + ControlFineTracking_Protocol[i_ly];
			}
			HeJiaoYan = ControlFineTracking_Protocol[43]<<8;
			HeJiaoYan = HeJiaoYan + ControlFineTracking_Protocol[44];		
			if(Sum_Data== HeJiaoYan)
			{
				for(i_l=0;i_l<46;i_l++)
				{
					Read_JiaoYanHouShuJu[i_l] = ControlFineTracking_Protocol[i_l];
					Read_JiaoYan_Flag = 1;
					Sum_Data = 0;
					HeJiaoYan = 0;
				}
			}
			HeJiaoYan = 0;
			Sum_Data = 0;
		}
	}
}

//打包发送的数据
void DaBao_Send_Data()
{
	DaBaoFaSong_Data[0] = 0XEB;
	DaBaoFaSong_Data[1] = 0X90;
	DaBaoFaSong_Data[2] = 0X23;
	DaBaoFaSong_Data[40] = 0XFE;	
}
////////////////
void AB_Com_Write()
{
	//union UintToArray ia;
	//ia.k = 300;
	
	for(AB_i=0; AB_i<AB_TX_Len; AB_i++)
	{
		*(uint32_t*)(AB_ADDR)= AB_TXData[AB_i];
	}
	Delay_us(1);
	*(uint32_t*)(AB_Com_En)= 1;
	//AA_fail=0X01;

}
////////////////
u32 ab_tx_cnt;
float ab_f;
//void AB_TX()////数据通道
//{
//		AB_TXData[0]=0X5B;//帧头
//		AB_TXData[1]=0X54;//帧头
//		
//		AB_TXData[2]=Time&0x000000ff;//系统时间。时
//		AB_TXData[3]=Time>>8&0x000000ff;//系统时间。分
//		AB_TXData[4]=Time>>16&0x000000ff;;//系统时间   秒
//		AB_TXData[5]=Time>>24&0x000000ff;;//系统时间  毫秒
//	  AB_TXData[6]=48;
//	  AB_TXData[7]=0;
//	  AB_TXData[8]=02;
//	  if(Work_Mode == 0x03)                                            //待机
//		{
//			AB_TXData[9] = 0x00;
//		}
//		else if(Work_Mode == 0x04)                                      //人工引导
//		{
//			AB_TXData[9] = 0x01;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x03)             //匀速扫描
//		{
//			AB_TXData[9] = 0x02;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x02)            //外同步步进扫描
//		{
//			AB_TXData[9] = 0x03;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x01)          //内同步步进扫描
//		{
//			AB_TXData[9] = 0x04;
//		}
//		else if(Work_Mode == 0x02)                                  //跟踪
//		{
//			AB_TXData[9] = 0x05;
//		}
//	  
//	    Temp_Data.f=ppp;//粗跟踪伺服方位位置
//     			
//		AB_TXData[10]=Temp_Data.ary[0];//粗跟踪方位 惯性空间
//		AB_TXData[11]=Temp_Data.ary[1];//粗跟踪方位
//		AB_TXData[12]=Temp_Data.ary[2];//粗跟踪方位
//		AB_TXData[13]=Temp_Data.ary[3];//粗跟踪方位
//		
////	    TX_Coarse_Pitch=yyy;
//		Temp_Data.f=yyy;//粗跟踪伺服俯仰位置	惯性空间
//		AB_TXData[14]=Temp_Data.ary[0];//粗跟踪俯仰
//		AB_TXData[15]=Temp_Data.ary[1];//粗跟踪俯仰
//		AB_TXData[16]=Temp_Data.ary[2];//粗跟踪俯仰
//		AB_TXData[17]=Temp_Data.ary[3];//粗跟踪俯仰
//		
//		
//		Temp_Data.f=FW_encoder_degrees;//编码器方位
////		if(ab_f>360.0)ab_f=0;
////		ab_f=ab_f+0.001;
////		Temp_Data.f=ab_f;
//		AB_TXData[18]=Temp_Data.ary[0];//
//		AB_TXData[19]=Temp_Data.ary[1];//
//		AB_TXData[20]=Temp_Data.ary[2];//
//		AB_TXData[21]=Temp_Data.ary[3];//
//		
//		Temp_Data.f=FY_encoder_degrees;//编码器俯仰
////		Temp_Data.f=ab_f;
//		AB_TXData[22]=Temp_Data.ary[0];//
//		AB_TXData[23]=Temp_Data.ary[1];//
//		AB_TXData[24]=Temp_Data.ary[2];//
//		AB_TXData[25]=Temp_Data.ary[3];//	
//			
//		TX_Ins_Yaw=yaw_attitude_float;
//		Temp_Data.f=TX_Ins_Yaw;//惯性空间偏航 
//		AB_TXData[26]=Temp_Data.ary[0];//惯性偏航
//		AB_TXData[27]=Temp_Data.ary[1];//
//		AB_TXData[28]=Temp_Data.ary[2];//
//		AB_TXData[29]=Temp_Data.ary[3];//
//			
//		TX_Ins_RollAngle=roll_attitude_float;		
//		Temp_Data.f=TX_Ins_RollAngle;//惯性空间横滚
//		AB_TXData[30]=Temp_Data.ary[0];//惯性横滚
//		AB_TXData[31]=Temp_Data.ary[1];//
//		AB_TXData[32]=Temp_Data.ary[2];//
//		AB_TXData[33]=Temp_Data.ary[3];//
//			
//		TX_Ins_PitchAngle=pitch_attitude_float;
//		Temp_Data.f=TX_Ins_PitchAngle;//惯性空间俯仰
//		AB_TXData[34]=Temp_Data.ary[0];//惯性俯仰
//		AB_TXData[35]=Temp_Data.ary[1];//
//		AB_TXData[36]=Temp_Data.ary[2];//
//		AB_TXData[37]=Temp_Data.ary[3];//
//				/////////
//		if(AB_CNT==2){
//		AB_TXData[38]=2;
//	//	AB_CNT=3;
//		Temp_Data.f=AN_1_fy ;//ELMO A相电流
//		AB_TXData[39]=Temp_Data.ary[0];//
//		AB_TXData[40]=Temp_Data.ary[1];//
//		AB_TXData[41]=Temp_Data.ary[2];//
//		AB_TXData[42]=Temp_Data.ary[3];//	
//			
//		/////
//		Temp_Data.f=AN_2_fy;//ELMO B相电流
//		AB_TXData[43]=Temp_Data.ary[0];//
//		AB_TXData[44]=Temp_Data.ary[1];//
//		AB_TXData[45]=Temp_Data.ary[2];//
//		AB_TXData[46]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_3_fy;///ELMO C相电流	
//		AB_TXData[47]=Temp_Data.ary[0];//惯性横滚
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_4_fy;//ELMO 母线电压
//		AB_TXData[51]=Temp_Data.ary[0];//
//		AB_TXData[52]=Temp_Data.ary[1];//
//		AB_TXData[53]=Temp_Data.ary[2];//
//		AB_TXData[54]=Temp_Data.ary[3];//
//		
//	}
//		/////////
//		if(AB_CNT==3){
//		AB_TXData[38]=3;
//	//		AB_CNT=4;
//		Temp_Data.f=VX_fw;//ELMO 方位速度
//		AB_TXData[39]=Temp_Data.ary[0];//
//		AB_TXData[40]=Temp_Data.ary[1];//
//		AB_TXData[41]=Temp_Data.ary[2];//
//		AB_TXData[42]=Temp_Data.ary[3];//	
//			
//	
//		Temp_Data.f=VX_fy;//
//		AB_TXData[43]=Temp_Data.ary[0];//
//		AB_TXData[44]=Temp_Data.ary[1];//
//		AB_TXData[45]=Temp_Data.ary[2];//
//		AB_TXData[46]=Temp_Data.ary[3];//
//			
//			
//		Temp_Data.f=PX_fw;//ELMO方位角度
//		AB_TXData[47]=Temp_Data.ary[0];//
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//	
//		Temp_Data.f=PX_fy;//ELMO俯仰角度
//		AB_TXData[51]=Temp_Data.ary[0];//
//		AB_TXData[52]=Temp_Data.ary[1];//
//		AB_TXData[53]=Temp_Data.ary[2];//
//		AB_TXData[54]=Temp_Data.ary[3];//
//		//AB_TXData[38]=2;
//	}
//		/////////
//		if(AB_CNT==1){
//		AB_TXData[38]=1;
//	//		AB_CNT=2;
//		Temp_Data.f=AN_1_fw;//ELMO A相电流
//		AB_TXData[39]=Temp_Data.ary[0];//
//		AB_TXData[40]=Temp_Data.ary[1];//
//		AB_TXData[41]=Temp_Data.ary[2];//
//		AB_TXData[42]=Temp_Data.ary[3];//	
//			
//		/////
//		Temp_Data.f=AN_2_fw;// ELMO B相电流
//		AB_TXData[43]=Temp_Data.ary[0];//
//		AB_TXData[44]=Temp_Data.ary[1];//
//		AB_TXData[45]=Temp_Data.ary[2];//
//		AB_TXData[46]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_3_fw;////ELMO C相电流	
//		AB_TXData[47]=Temp_Data.ary[0];//惯性横滚
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_4_fw ;//ELMO 母线电压
//		AB_TXData[51]=Temp_Data.ary[0];//
//		AB_TXData[52]=Temp_Data.ary[1];//
//		AB_TXData[53]=Temp_Data.ary[2];//
//		AB_TXData[54]=Temp_Data.ary[3];//
//		
//	}
//				/////////
//		if(AB_CNT==4){
//		AB_TXData[38]=4;
////		AB_CNT=5;
//		AB_TXData[39]=EE1_fw;//
//		AB_TXData[40]=EE1_fw>>8;//
//		AB_TXData[41]=EE1_fw>>16;//
//		AB_TXData[42]=EE1_fw>>24;//	
//			

//		AB_TXData[43]=EE1_fy;//
//		AB_TXData[44]=EE1_fy>>8;//
//		AB_TXData[45]=EE1_fy>>16;//
//		AB_TXData[46]=EE1_fy>>24;//
//			
//		
//	
//		AB_TXData[47]=SO_fw;
//		AB_TXData[48]=SO_fw>>8;//
//		AB_TXData[49]=SO_fw>>16;//
//		AB_TXData[50]=SO_fw>>24;//
//			
//		
//		
//		AB_TXData[51]=SO_fy;//
//		AB_TXData[52]=SO_fy>>8;//
//		AB_TXData[53]=SO_fy>>16;//
//		AB_TXData[54]=SO_fy>>24;//
//		
//	}
//		/////////
//		if(AB_CNT==5){
//		AB_TXData[38]=5;
////		AB_CNT=6;
//		AB_TXData[39]=EC_fw;//
//		AB_TXData[40]=EC_fw>>8;//
//		AB_TXData[41]=EC_fw>>16;//
//		AB_TXData[42]=EC_fw>>24;//	
//			
//	
//		
//		AB_TXData[43]=EC_fy;//
//		AB_TXData[44]=EC_fy>>8;//
//		AB_TXData[45]=EC_fy>>16;//
//		AB_TXData[46]=EC_fy>>24;//
//			
//			
//		
//		AB_TXData[47]=MF_fw;//
//		AB_TXData[48]=MF_fw>>8;//
//		AB_TXData[49]=MF_fw>>16;//
//		AB_TXData[50]=MF_fw>>24;//
//			
//	
//		
//		AB_TXData[51]=MF_fy;
//		AB_TXData[52]=MF_fy>>8;//
//		AB_TXData[53]=MF_fy>>16;//
//		AB_TXData[54]=MF_fy>>24;//
//		
//	}
//		/////////
//		if(AB_CNT==6){
//		AB_TXData[38]=6;
////		AB_CNT=1;
//		AB_TXData[39]=SR_fw;//
//		AB_TXData[40]=SR_fw>>8;//
//		AB_TXData[41]=SR_fw>>16;//
//		AB_TXData[42]=SR_fw>>24;//	
//			
//		/////
//		
//		AB_TXData[43]=SR_fy;//
//		AB_TXData[44]=SR_fy>>8;//
//		AB_TXData[45]=SR_fy>>16;//
//		AB_TXData[46]=SR_fy>>24;//
//			
//		
//	
//		AB_TXData[47]=0xaa;//
//		AB_TXData[48]=0xaa;//
//		AB_TXData[49]=0xaa;//
//		AB_TXData[50]=0XAA;//
//			
//		
//		
//		AB_TXData[51]=0xaa;//
//		AB_TXData[52]=0xaa;//
//		AB_TXData[53]=0xaa;//
//		AB_TXData[54]=0xaa;//
//		
//	}
//		if(AB_CNT==6)AB_CNT=1;else AB_CNT++;
//			
//		/////////
//		AB_TXData[55]=0XAA;
//		AB_TXData[56]=0X5B;
//		AB_TXData[57]=0XFE;
//	  AB_TX_Len=58;
//    AB_Com_Write();
//	  ab_tx_cnt++;
//	
//	}
void AB_TX()////????
{
		AB_TXData[0]=0X5B;//??
		AB_TXData[1]=0X54;//??
		//Time=ab_tx_cnt;
		AB_TXData[2]=Time&0x000000ff;//??????
		AB_TXData[3]=Time>>8&0x000000ff;//??????
		AB_TXData[4]=Time>>16&0x000000ff;;//????   ?
		AB_TXData[5]=Time>>24&0x000000ff;;//????  ??
		AB_TXData[6]=64;
		AB_TXData[7]=0;
		AB_TXData[8]=02;
		if(Work_Mode == 0x03)                                            //??
		{
			AB_TXData[9] = 0x00;
		}
		else if(Work_Mode == 0x04)                                      //????
		{
			AB_TXData[9] = 0x01;
		}
		else if(Work_Mode == 0x01 && Scan_FangShi == 0x03)             //????
		{
			AB_TXData[9] = 0x02;
		}
		else if(Work_Mode == 0x01 && Scan_FangShi == 0x02)            //???????
		{
			AB_TXData[9] = 0x03;
		}
		else if(Work_Mode == 0x01 && Scan_FangShi == 0x01)          //???????
		{
			AB_TXData[9] = 0x04;
		}
		else if(Work_Mode == 0x02)                                  //??
		{
			AB_TXData[9] = 0x05;
		}
	  
//	    Temp_Data.f=ppp;//?????????
		Temp_Data.f=Space_FW;//?????????
//     	Temp_Data.f = SpaceSetValueFW;
		AB_TXData[10]=Temp_Data.ary[0];//????? ????
		AB_TXData[11]=Temp_Data.ary[1];//?????
		AB_TXData[12]=Temp_Data.ary[2];//?????
		AB_TXData[13]=Temp_Data.ary[3];//?????
		
//	    TX_Coarse_Pitch=yyy;
		Temp_Data.f=Space_FY;//?????????	????
//		Temp_Data.f= SpaceSetValueFY;
		AB_TXData[14]=Temp_Data.ary[0];//?????
		AB_TXData[15]=Temp_Data.ary[1];//?????
		AB_TXData[16]=Temp_Data.ary[2];//?????
		AB_TXData[17]=Temp_Data.ary[3];//?????
		
		
		Temp_Data.f=FW_encoder_degrees;//?????
//		if(ab_f>360.0)ab_f=0;
//		ab_f=ab_f+0.001;
//		Temp_Data.f=ab_f;
		AB_TXData[18]=Temp_Data.ary[0];//
		AB_TXData[19]=Temp_Data.ary[1];//
		AB_TXData[20]=Temp_Data.ary[2];//
		AB_TXData[21]=Temp_Data.ary[3];//
		
		Temp_Data.f=FY_encoder_degrees;//?????
//		Temp_Data.f=ab_f;
		AB_TXData[22]=Temp_Data.ary[0];//
		AB_TXData[23]=Temp_Data.ary[1];//
		AB_TXData[24]=Temp_Data.ary[2];//
		AB_TXData[25]=Temp_Data.ary[3];//	
			
		TX_Ins_Yaw=yaw_attitude_float;
//		TX_Ins_Yaw=z_axis_velocity_float;
		Temp_Data.f=TX_Ins_Yaw;//?????? 
		AB_TXData[26]=Temp_Data.ary[0];//????
		AB_TXData[27]=Temp_Data.ary[1];//
		AB_TXData[28]=Temp_Data.ary[2];//
		AB_TXData[29]=Temp_Data.ary[3];//
			
		TX_Ins_RollAngle=roll_attitude_float;
//        TX_Ins_RollAngle=y_axis_velocity_float;		
		Temp_Data.f=TX_Ins_RollAngle;//??????
		AB_TXData[30]=Temp_Data.ary[0];//????
		AB_TXData[31]=Temp_Data.ary[1];//
		AB_TXData[32]=Temp_Data.ary[2];//
		AB_TXData[33]=Temp_Data.ary[3];//
			
		TX_Ins_PitchAngle=pitch_attitude_float;
//        TX_Ins_PitchAngle=x_axis_velocity_float;
		Temp_Data.f=TX_Ins_PitchAngle;//??????
		AB_TXData[34]=Temp_Data.ary[0];//????
		AB_TXData[35]=Temp_Data.ary[1];//
		AB_TXData[36]=Temp_Data.ary[2];//
		AB_TXData[37]=Temp_Data.ary[3];//
		//////////////???
		//Temp_Data.f=
//		AB_TXData[38]=Temp_Data.ary[0];//????
//		AB_TXData[39]=Temp_Data.ary[1];//
//		AB_TXData[40]=Temp_Data.ary[2];//
//		AB_TXData[41]=Temp_Data.ary[3];//
//	//	Temp_Data.f
//		AB_TXData[42]=Temp_Data.ary[0];//????
//		AB_TXData[43]=Temp_Data.ary[1];//
//		AB_TXData[44]=Temp_Data.ary[2];//
//		AB_TXData[45]=Temp_Data.ary[3];//
		/////////
		//Temp_Data.f=TX_Ins_RollAngle;//?????
		AB_TXData[38]=ControlFineTracking_Protocol[15];//
		AB_TXData[39]=ControlFineTracking_Protocol[16];//
		AB_TXData[40]=ControlFineTracking_Protocol[17];//
		AB_TXData[41]=ControlFineTracking_Protocol[18];//
			
		
	//	Temp_Data.f=TX_Ins_PitchAngle;//?????
		AB_TXData[42]=ControlFineTracking_Protocol[19];//
		AB_TXData[43]=ControlFineTracking_Protocol[20];//
		AB_TXData[44]=ControlFineTracking_Protocol[21];//
		AB_TXData[45]=ControlFineTracking_Protocol[22];//
		
				
		Temp_Data.f=FW_Miss_distance;//?????
		AB_TXData[46]=Temp_Data.ary[0];//????
		AB_TXData[47]=Temp_Data.ary[1];//
		AB_TXData[48]=Temp_Data.ary[2];//
		AB_TXData[49]=Temp_Data.ary[3];//
			
	////?????
		Temp_Data.f=FY_Miss_distance;//
		AB_TXData[50]=Temp_Data.ary[0];//
		AB_TXData[51]=Temp_Data.ary[1];//
		AB_TXData[52]=Temp_Data.ary[2];//
		AB_TXData[53]=Temp_Data.ary[3];//
		
				/////////
		if(AB_CNT==2){
		AB_TXData[54]=2;
	//	AB_CNT=3;
		Temp_Data.f=AN_1_fy ;//ELMO A???
		AB_TXData[55]=Temp_Data.ary[0];//
		AB_TXData[56]=Temp_Data.ary[1];//
		AB_TXData[57]=Temp_Data.ary[2];//
		AB_TXData[58]=Temp_Data.ary[3];//	
			
		/////
		Temp_Data.f=AN_2_fy;//ELMO B???
		AB_TXData[59]=Temp_Data.ary[0];//
		AB_TXData[60]=Temp_Data.ary[1];//
		AB_TXData[61]=Temp_Data.ary[2];//
		AB_TXData[62]=Temp_Data.ary[3];//
			
		
		Temp_Data.f=AN_3_fy;///ELMO C???	
		AB_TXData[63]=Temp_Data.ary[0];//????
		AB_TXData[64]=Temp_Data.ary[1];//
		AB_TXData[65]=Temp_Data.ary[2];//
		AB_TXData[66]=Temp_Data.ary[3];//
			
		
		Temp_Data.f=AN_4_fy;//ELMO ????
		AB_TXData[67]=Temp_Data.ary[0];//
		AB_TXData[68]=Temp_Data.ary[1];//
		AB_TXData[69]=Temp_Data.ary[2];//
		AB_TXData[70]=Temp_Data.ary[3];//
		
	}
		/////////
		if(AB_CNT==3){
		AB_TXData[54]=3;
	//		AB_CNT=4;
		Temp_Data.f=VX_fw;//ELMO ????
		AB_TXData[55]=Temp_Data.ary[0];//
		AB_TXData[56]=Temp_Data.ary[1];//
		AB_TXData[57]=Temp_Data.ary[2];//
		AB_TXData[58]=Temp_Data.ary[3];//	
			
	
		Temp_Data.f=VX_fy;//
		AB_TXData[59]=Temp_Data.ary[0];//
		AB_TXData[60]=Temp_Data.ary[1];//
		AB_TXData[61]=Temp_Data.ary[2];//
		AB_TXData[62]=Temp_Data.ary[3];//
			
			
		Temp_Data.f=PX_fw;//ELMO????
		AB_TXData[63]=Temp_Data.ary[0];//
		AB_TXData[64]=Temp_Data.ary[1];//
		AB_TXData[65]=Temp_Data.ary[2];//
		AB_TXData[66]=Temp_Data.ary[3];//
			
	
		Temp_Data.f=PX_fy;//ELMO????
		AB_TXData[67]=Temp_Data.ary[0];//
		AB_TXData[68]=Temp_Data.ary[1];//
		AB_TXData[69]=Temp_Data.ary[2];//
		AB_TXData[70]=Temp_Data.ary[3];//
		//AB_TXData[38]=2;
	}
		/////////
		if(AB_CNT==1){
		AB_TXData[54]=1;
	//		AB_CNT=2;
		Temp_Data.f=AN_1_fw;//ELMO A???
		AB_TXData[55]=Temp_Data.ary[0];//
		AB_TXData[56]=Temp_Data.ary[1];//
		AB_TXData[57]=Temp_Data.ary[2];//
		AB_TXData[58]=Temp_Data.ary[3];//	
			
		/////
		Temp_Data.f=AN_2_fw;// ELMO B???
		AB_TXData[59]=Temp_Data.ary[0];//
		AB_TXData[60]=Temp_Data.ary[1];//
		AB_TXData[61]=Temp_Data.ary[2];//
		AB_TXData[62]=Temp_Data.ary[3];//
			
		
		Temp_Data.f=AN_3_fw;////ELMO C???	
		AB_TXData[63]=Temp_Data.ary[0];//????
		AB_TXData[64]=Temp_Data.ary[1];//
		AB_TXData[65]=Temp_Data.ary[2];//
		AB_TXData[66]=Temp_Data.ary[3];//
			
		
		Temp_Data.f=AN_4_fw ;//ELMO ????
		AB_TXData[67]=Temp_Data.ary[0];//
		AB_TXData[68]=Temp_Data.ary[1];//
		AB_TXData[69]=Temp_Data.ary[2];//
		AB_TXData[70]=Temp_Data.ary[3];//
		
	}
				/////////
		if(AB_CNT==4){
		AB_TXData[54]=4;
//		AB_CNT=5;
		AB_TXData[55]=EE1_fw;//
		AB_TXData[56]=EE1_fw>>8;//
		AB_TXData[57]=EE1_fw>>16;//
		AB_TXData[58]=EE1_fw>>24;//	
			

		AB_TXData[59]=EE1_fy;//
		AB_TXData[60]=EE1_fy>>8;//
		AB_TXData[61]=EE1_fy>>16;//
		AB_TXData[62]=EE1_fy>>24;//
			
		
	
		AB_TXData[63]=SO_fw;
		AB_TXData[64]=SO_fw>>8;//
		AB_TXData[65]=SO_fw>>16;//
		AB_TXData[66]=SO_fw>>24;//
			
		
		
		AB_TXData[67]=SO_fy;//
		AB_TXData[68]=SO_fy>>8;//
		AB_TXData[69]=SO_fy>>16;//
		AB_TXData[70]=SO_fy>>24;//
		
	}
		/////////
		if(AB_CNT==5){
		AB_TXData[54]=5;
//		AB_CNT=6;
		AB_TXData[55]=EC_fw;//
		AB_TXData[56]=EC_fw>>8;//
		AB_TXData[57]=EC_fw>>16;//
		AB_TXData[58]=EC_fw>>24;//	
			
	
		
		AB_TXData[59]=EC_fy;//
		AB_TXData[60]=EC_fy>>8;//
		AB_TXData[61]=EC_fy>>16;//
		AB_TXData[62]=EC_fy>>24;//
			
			
		
		AB_TXData[63]=MF_fw;//
		AB_TXData[64]=MF_fw>>8;//
		AB_TXData[65]=MF_fw>>16;//
		AB_TXData[66]=MF_fw>>24;//
			
	
		
		AB_TXData[67]=MF_fy;
		AB_TXData[68]=MF_fy>>8;//
		AB_TXData[69]=MF_fy>>16;//
		AB_TXData[70]=MF_fy>>24;//
		
	}
		/////////
		if(AB_CNT==6){
		AB_TXData[54]=6;
//		AB_CNT=1;
		AB_TXData[55]=SR_fw;//
		AB_TXData[56]=SR_fw>>8;//
		AB_TXData[57]=SR_fw>>16;//
		AB_TXData[58]=SR_fw>>24;//	
			
		/////
		
		AB_TXData[59]=SR_fy;//
		AB_TXData[60]=SR_fy>>8;//
		AB_TXData[61]=SR_fy>>16;//
		AB_TXData[62]=SR_fy>>24;//
			
		
	
		AB_TXData[63]=0xaa;//
		AB_TXData[64]=0xaa;//
		AB_TXData[65]=0xaa;//
		AB_TXData[66]=0XAA;//
			
		
		
		AB_TXData[67]=0xaa;//
		AB_TXData[68]=0xaa;//
		AB_TXData[69]=0xaa;//
		AB_TXData[70]=0xaa;//
		
	}
		if(AB_CNT==6)AB_CNT=1;else AB_CNT++;
			
		/////////
		AB_TXData[71]=0XAA;
		AB_TXData[72]=0X5B;
		AB_TXData[73]=0XFE;
		AB_TX_Len=74;
		AB_Com_Write();
		ab_tx_cnt++;
	
	}
//////////
	s8 CJ_Time=0;
	void CJ()///数据通道
{
	
		
			if(CJ_Time ==0 ) 
		    {
				AB_TX();
				CJ_Time=100;
				//PC_Data_TX();
		    }
		  CJ_Time--;
}



/////////////////////////////////
////////
////////////////
u8 AB_Data[50];
void AB_Com_Read()      //①判断标志位后读取FPGA内字节机上惯导
{
	AA_ADDR_tmp=AB_ADDR;
//	AB_Data[5]=0;
//	AA_i=0;
   for(AB_i=0; AB_i<48; AB_i++)
	{
		AB_Data[AB_i]=*(uint32_t*)(AA_ADDR_tmp);
		AA_ADDR_tmp = AA_ADDR_tmp+2;
//		AA_RX_Len=AA_Data[5]+9;
//		AA_i++;
//		if(AA_i==AA_RX_Len) 
//		break;
	}
	AB_DATA_RX();        //②判断帧头帧尾和校验
}

u32 UTS_Time,UTS_Time1;
u16 HH,MM,SS,sss;

s32 Lattitude_int,Longitude_int,Altitude_int;
float Lattitude_f,Longitude_f,Altitude_f;
s16 Yaw, Pitch, Roll;
float Yaw_f, Pitch_f, Roll_f;
s16 N_V,E_V,SKEY_V,X_Palstance,Y_Palstance,Z_Palstance,X_Acceleration,Y_Acceleration,Z_Acceleration;
float N_V_f,E_V_f,SKEY_V_f,X_Palstance_f,Y_Palstance_f,Z_Palstance_f,X_Acceleration_f,Y_Acceleration_f,Z_Acceleration_f;
s16 Status;
u32 ab_rx_cnt;
void AB_DATA_RX()
{
u8 ii=0,Sum2;
	u32 Sum1=0;
	ab_rx_cnt++;
	if((AB_Data[0]==0x99) & (AB_Data[1]==0x66))//帧头
	{
//		if((AA_Data[AA_RX_Len-2]==0x5a)&(AA_Data[AA_RX_Len-1]==0xfe))//帧尾
//		{
			for(ii=2;ii<(45);ii++)   
			{
				Sum1+=AB_Data[ii];
			}
			Sum2=Sum1&0xff;   //sum1 = 各数据字节累加
			Sum2=256-Sum2;    //sum2 = 校验字节
			
			if(AB_Data[45]==Sum2)//求和校验
			{
				
				UTS_Time1=AB_Data[3]+AB_Data[4]*256+AB_Data[5]*256*256+AB_Data[6]*256*256*256;//   时间
				Lattitude_int=AB_Data[7]+AB_Data[8]*256+AB_Data[9]*256*256+AB_Data[10]*256*256*256;  //纬度
				Altitude_int=AB_Data[11]+AB_Data[12]*256+AB_Data[13]*256*256+AB_Data[14]*256*256*256;////高度
				Longitude_int=AB_Data[15]+AB_Data[16]*256+AB_Data[17]*256*256+AB_Data[18]*256*256*256;  ///经度
				N_V=AB_Data[19]+AB_Data[20]*256;              ///北向速度
				SKEY_V=AB_Data[21]+AB_Data[22]*256;            ///天向速度
				E_V=AB_Data[23]+AB_Data[24]*256;             //东向速度
				Roll=AB_Data[25]+AB_Data[26]*256;            ///横滚角
				Yaw=AB_Data[27]+AB_Data[28]*256;              ////航向角
				Pitch=AB_Data[29]+AB_Data[30]*256;            ///俯仰角
				X_Palstance=AB_Data[31]+AB_Data[32]*256;      //X轴角速度
				Y_Palstance=AB_Data[33]+AB_Data[34]*256;      //Y轴角速度   
				Z_Palstance=AB_Data[35]+AB_Data[36]*256;       //Z轴角速度
				X_Acceleration=AB_Data[37]+AB_Data[38]*256;   //X轴加速度
				Y_Acceleration=AB_Data[39]+AB_Data[40]*256;  //Y轴加速度
				Z_Acceleration=AB_Data[41]+AB_Data[42]*256;  //Z轴加速度
				Status=AB_Data[43]+AB_Data[44]*256;   //////状态字
// 
        Lattitude_f= (float)Lattitude_int*180.0/2147483646.0; //纬度
				Longitude_f= (float)Longitude_int*180.0/2147483646.0;
				Altitude_f = (float)Altitude_int*16382.0/2147483646.0;
				Yaw_f   = Yaw*180.0/32767.0;
				Pitch_f = Pitch*180.0/32767.0;
				Roll_f  = Roll*180.0/32767.0;
				N_V_f   = N_V*256/32767.0;
				E_V_f   = E_V*256/32767.0;
				SKEY_V_f= SKEY_V*256/32767.0;
				X_Palstance_f=X_Palstance*300/32767.0;
				Y_Palstance_f=Y_Palstance*300/32767.0;
				Z_Palstance_f=Z_Palstance*300/32767.0;
				X_Acceleration_f= X_Acceleration*100/32767.0;
				Y_Acceleration_f= Y_Acceleration*100/32767.0;
				Z_Acceleration_f= Z_Acceleration*100/32767.0;
				 UTS_Time=UTS_Time1;
				 sss=UTS_Time%1000;
				 UTS_Time=UTS_Time/1000;
         SS=UTS_Time%100;
				  UTS_Time=UTS_Time/100;
          MM=UTS_Time%100;		 
				  HH=UTS_Time/100;
//          AB_Data[46]	
//          AB_Data[47]			
					
				}
		//	}
		}


}

