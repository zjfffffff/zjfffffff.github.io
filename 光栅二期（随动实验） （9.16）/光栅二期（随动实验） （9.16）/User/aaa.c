#include "stm32f4xx.h"//
#include "aaa.h"
#include "stmflash.h"
#include "ioinit.h"
#include "AttitudeAlgorithm.h"
/////////////600000=400ms
//#define CJ_T 25
//#define CJ_Len 8
//#define AA_ADDR_422        ((u32)(0x64006000))     //���ڶ�ȡ��ַ              422��ַ
//#define AA_Com_En_422      ((u32)(0x64006800))     //���ڷ���ʹ�ܵ�ַ          422��ַ
#define AB_ADDR        ((u32)(0x6400B100))

#define AA_ADDR        ((u32)(0x64004000))     //���ڶ�ȡ��ַ                LVDS��ַ
#define AA_Com_En      ((u32)(0x64004800))     //���ڷ���ʹ�ܵ�ַ            LVDSʹ��
#define AB_Com_En      ((u32)(0x64000000))     //���ڷ���ʹ�ܵ�ַ  //0x6400B900
//#define AA_ADDR        ((u32)(0x64006000))     //���ڶ�ȡ��ַ                422��ַ
//#define AA_Com_En      ((u32)(0x64006800))     //���ڷ���ʹ�ܵ�ַ            422ʹ��

#define Control_FineComReadADDR          ((u32)(0x64003000))//������Э���ȡ��ַ
#define Fine_ControlComWriteADDR         ((u32)(0x64003000))//������Э��д���ַ
#define Fine_ControlComWriteEnable       ((u32)(0x64003800))//������Э��д��ʹ��


u8 AB_i=0,AB_TX_Len;
u8 AB_TXData[100];     //��������
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
float TX_Coarse_Azimuth=0;//�ָ����ŷ���λ
float TX_Coarse_Pitch=0;//�ָ����ŷ�����
float TX_Precise_Azimuth=0;//�������ŷ���λ
float TX_Precise_Pitch=0;//�������ŷ�����

float RX_Coarse_Azimuth=0;//�ָ����ŷ���λ
float RX_Coarse_Pitch=0;//�ָ����ŷ�����
float RX_Precise_Azimuth=0;//�������ŷ���λ
float RX_Precise_Pitch=0;//�������ŷ�����

float FW_Miss_distance;//��λ�Ѱ���
float FY_Miss_distance;//�����Ѱ���

float RX_Coarse_Azimuth1=0;//�ָ��ٹ��Ჹ����λ�Ƕ�
float RX_Coarse_Pitch1=0;//�ָ��ٹ��Ჹ������ �Ƕ�
float RX_Precise_Azimuth1=0;//�����ٹ��Ჹ����λ�Ƕ�
float RX_Precise_Pitch1=0;//�����ٹ��Ჹ�������Ƕ�
/*
float TX_Coarse_Azimuth1=0;//�ָ����ŷ���λ
float TX_Coarse_Pitch1=0;//�ָ����ŷ�����
float TX_Precise_Azimuth1=0;//�������ŷ���λ
float TX_Precise_Pitch1=0;//�������ŷ�����
*/
//�ߵ��ĸ����Ƕ���Ϣ��InsPitchAngle��������Ƕ���Ϣ��InsRollAngle��
float TX_Ins_RollAngle=0;//���Կռ���
float TX_Ins_PitchAngle=0;//���Կռ丩��
float TX_Ins_Yaw=0;//���Կռ�ƫ��
/////�ɼ�����
float TX_Ins_Attitude_FW=0;//�ߵ���̬��λ
float TX_Ins_Attitude_FY=0;//�ߵ���̬����
float TX_Ins_Attitude_Roll=0;//�ߵ���̬���
float TX_Ins_Speed_FW=0;//�ߵ����ٶȷ�λ
float TX_Ins_Speed_FY=0;//�ߵ����ٶȸ���
float TX_Ins_Speed_Roll=0;//�ߵ����ٶȺ��

float TX_Encoder_FW=0;//��������λ
float TX_Encoder_FY=0;//����������
float TX_Miss_FY=0;//�Ѱ�����λ
float TX_Miss_FW=0;//�Ѱ�������
float TX_Fast_Reflection_Mirror_FW=0;//�췴����λ
float TX_Fast_Reflection_Mirror_FY=0;//�췴������


//float TX_Precise_Pitch=0;//�������ŷ�����
float FW_position;//��λλ��
float FY_position;//����λ��

u8 Scan_direction,Scan_times;//ɨ�跽��ɨ�����
s8 Scan_FW_Angle,Scan_FY_Angle;//ɨ�跽λ�ǣ�ɨ�踩����
s8 Scan_FW_Start_Angle;
s8 Scan_FY_Start_Angle;
int16_t FW_Scan_Step; //��λɨ�貽��
int16_t FY_Scan_Step; // ����ɨ�貽��
float FW_Scan_Step_Float=0;
float FY_Scan_Step_Float=0;

u8 Scan_Speed; //ɨ���ٶ�
u8 RX_Servo_Steady_State;//�ŷ���̬
u8 TX_Servo_Steady_State;//�ŷ���̬
u8 TX_Coarse_State;  ///�ָ���״̬��
u8 TX_Precise_State; ///������״̬��
u8 Point_Mode; //ָ��ģʽ
u16 Step_T;//�������ʱ�䣬
u16 FW_Step_Angle;//��λÿ�β����ǣ�
u16 FY_Step_Angle;//����ÿ�β�����
u32 Stable_T; ///�ȶ�ʱ��
s8 FW_Angle_MAX1,FY_Angle_MAX1;//ɨ�����λ������1
int16_t FW_Angle_MAX2,FY_Angle_MAX2;//ɨ�����λ ������2
u8 Primary_Mirror_Work_Mode;//�����侵����ģʽ
u8 Fast_Mirror_Work_Mode;//�췴������ģʽ

u32 AA_Time=0;
u8 AA_Flage=0;
u8 AA_i=0;
u8 AA_frame;
u8 AA_TX_Len,AA_RX_Len;//���ݳ���
u32 AA_ADDR_tmp;       //
u8 AA_TXData[100];     //��������
u8 AA_Data[100];       //��������
u8 AA_Err,AA_fail;   
u8 CommandNew,CommandOld,CO_ID,ST_ID,Work_Mode=0X03;//���� �豸ID ����ģʽ
u8 Time_T=10,Time_M=7,Time_S=1,Time_mS_H=1,Time_mS_L=10;//��ϵͳʱ��
u8 SYS_Time_T,SYS_Time_M,SYS_Time_S,SYS_Time_mS_H,SYS_Time_mS_L; //  ϵͳʱ��
u8 Driver_Set1,Driver_Set2,Driver_Set3,Driver_Set4,Driver_Set5,Driver_Set6;//�ŷ���������������
u8 Driver_Set_Mode;///�ŷ����������÷�ʽ

u8 Work_Mode_Old =0;
u8 Clearn_flag = 0;

//��λ��ʹ�ò���
extern u8 system_mode;
extern float FW_pointing_degree_set;
extern float FY_pointing_degree_set; 

extern float FY_encoder_degrees;  //����������ת��Ϊ�Ƕ�
extern float FW_encoder_degrees;


extern float miss_distance_X_float;     
extern float miss_distance_Y_float;

extern float FY_miss_distance_sub;
extern float FW_miss_distance_sub;


extern float yaw_angle_float_slave;      //С�ߵ�ƫ��

extern float pitch_angle_float_slave;    //С�ߵ�����

extern float roll_angle_float_slave;     //С�ߵ����


extern float X_axis_palstance_float;     //С�ߵ����ٶ�
extern float Y_axis_palstance_float;     //С�ߵ����ٶ�
extern float Z_axis_palstance_float;     //С�ߵ����ٶ�

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
extern float FY_location_loop_error;      //�趨ֵ - ������ֵ 

extern float simulated_target_motion;

extern float IMU_bit_float;

extern float miss_distance_X_float_to_angle;
extern float miss_distance_Y_float_to_angle;

u32 SYS_Time = 0;
u32 Time = 0;

extern float X_slave_1ms_distance_sum;
extern float Z_slave_1ms_distance_sum;

extern float FW_wending_qiankui_speed_lowpass;  //��λǰ���ٶȵ�ͨ�˲�
extern float FW_location_loop_error_sub;    //�趨ֵ�������ֵ���

extern float FW_moni_miss;
extern float FY_moni_miss;

//��ʽ���������ٶ�
extern float x_axis_velocity_float;
extern float z_axis_velocity_float;
extern float y_axis_velocity_float;
//��ʽ����������̬
extern float yaw_attitude_float;
extern float pitch_attitude_float;
extern float roll_attitude_float;

//�Ѱ���ͼ��
extern float FW_send_array[10];
extern float FY_send_array[10];
extern u8 FW_send_array_select;
extern u8 FY_send_array_select;

float FW_Huiling_last = 0;
float FY_Huiling_last = 0;


//�Լ��־λ
extern int FY_encoder_Flag;
extern int FW_encoder_Flag;
extern int GuangxianGuandao_Flag;
extern float temp1_f;
extern float temp2_f;


extern float FW_zero_degree_zheng;
extern float FY_zero_degree;
extern float FY_light_loop_revise;
extern float FW_light_loop_revise;


//�뾫����ͨ��
uint8_t  ControlFineTracking_Protocol[46];//�ܿؾ�����ͨ��Э��
uint8_t  Read_JiaoYanHouShuJu[46];//У���洢����
uint8_t  DaBaoFaSong_Data[41]={0};//�������ٷ������ݴ��
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

//����װ��
u8 PZdataWX[38];

int bbb = 0;


extern u8 step_open_flag;
extern int	FW_Scan_StartAngle;                    //��λɨ����ʼ�Ƕ�
extern int ScanAngleRang;                        //ɨ�跶Χ
extern float	FW_ScanStepLength;                      //ɨ�貽��
extern int Scan_Direction;                        //�ж�ɨ�跽��
extern u8 Give_Scan_Number;

extern int Time_Long ;                            //���ʱ��
int Recevie = 0;
extern int FY_Scan_StartAngle;
extern float FY_ScanAngleRang;                      //����ɨ�跶Χ

extern float FW_yunsu_sudu;
extern float FY_yunsu_sudu ;
extern float FW_yunsu_sudu_1;
int QIUhe_CNT = 0;

extern float SpaceSetValueFW;
extern float SpaceSetValueFY;

extern float ScanAngleRang_ly;                        //ɨ�跶Χ
extern float FW_ScanStepLength_ly;                      //ɨ�貽��
extern float FW_YunSuStepLength_ly;                      //ɨ�貽��
float Scan_Speed_Degree_s = 0;
extern float YunSuAngleRang_ly;  

extern float KongJianZhiXiang_FW;
extern float KongJianZhiXiang_FY;
extern int WaiTongBu_ScanAngleRang;                        //ɨ�跶Χ
extern float WaiTongBu_FW_ScanStepLength;   

extern float KongJianZhiXiang_FWLY;
extern float KongJianZhiXiang_FYLY;

extern float KongJianPoint_FW;
extern float KongJianPoint_FY;

extern float KongJianZhiXiang_FWLY1;
extern float KongJianZhiXiang_FYLY1;

extern int WaiTongBu_Scan_Direction;                        //�ж�ɨ�跽��

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
extern int WaiTongBu_Flag_Com_Direction;                    //����ֻ��һ��ֵ
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

float AryToFloat(char c1, char c2, char c3, char c4)     //��4���ֽ�ת���ɵ����ȸ���
{    
	union FloatToArray fta;
	fta.ary[0] = c1;    
	fta.ary[1] = c2;    
	fta.ary[2] = c3;    
	fta.ary[3] = c4;    
	return fta.f;
}
u32 tx_cnt;
void AA_Com_Read()      //���жϱ�־λ���ȡFPGA���ֽ�
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
	AA_DATA_();        //���ж�֡ͷ֡β��У��
}

void AA_Com_Write()    //���ڷ���
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

void AA_DATA_()     //���ж�֡ͷ֡β��У��
{
	u8 ii=0,Sum2;
	u32 Sum1=0;
	if((AA_Data[0]==0x5a) & (AA_Data[1]==0x54))//֡ͷ
	{
		if((AA_Data[AA_RX_Len-2]==0x5a)&(AA_Data[AA_RX_Len-1]==0xfe))//֡β
		{
			for(ii=6;ii<(AA_RX_Len-3);ii++)   
			{
				Sum1+=AA_Data[ii];
			}
			Sum2=Sum1&0xff;   //sum1 = �������ֽ��ۼ�
			Sum2=256-Sum2;    //sum2 = У���ֽ�
			
			if(AA_Data[AA_RX_Len-3]==Sum2)//���У��
			{
				if(AA_Data[2]==0x10&AA_Data[3]==0x17)
				{							
					 CommandNew=AA_Data[4];
					 CO_ID=AA_Data[2];
					 ST_ID=AA_Data[3];
					 SYS_Time=AA_Data[6]+AA_Data[7]*256+AA_Data[8]*256*256+AA_Data[9]*256*256*256;//ϵͳʱ�䡣ʱ
//                     Time=SYS_Time;
//					 SYS_Time_T=AA_Data[6];//ϵͳʱ�䡣ʱ
//					 SYS_Time_M=AA_Data[7];//ϵͳʱ�䡣��
//					 SYS_Time_S=AA_Data[8];//ϵͳʱ��   ��
//					 SYS_Time_mS_H=AA_Data[9];//ϵͳʱ��  ����
//					 SYS_Time_mS_L=AA_Data[10];//ϵͳʱ�� ����
					 
//					 Time_T=SYS_Time_T;//ϵͳʱ�䡣ʱ
//			         Time_M=SYS_Time_M;//ϵͳʱ�䡣��
//					 Time_S=SYS_Time_S;//ϵͳʱ��   ��
//					 Time_mS_L=SYS_Time_mS_L;//ϵͳʱ��  ����
//					 Time_mS_H=SYS_Time_mS_H;//ϵͳʱ�� ����
		
					 AA_Err=0;
					 AA_Data_CL();        //��֡ͷ֡β��У�鶼ͨ����ִ�������������;
				}
				else  //�豸ID�����
				{
					 AA_TXData[0]=0X5A;//֡ͷ
					 AA_TXData[1]=0X54;//֡ͷ
					 AA_TXData[2]=0X17;//�����豸 0x17
					 AA_TXData[3]=0X10;//�����豸0x10
					 AA_TXData[4]=0XE4;//������ST
					 AA_TXData[5]=0X07;//���ݳ���SL
					
					 AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
			     AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
			     AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
			     AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����
					
					 /////����
					 AA_TXData[10]=0x17;       //��ǰ�豸ID
					 AA_TXData[11]=CO_ID;      //����ԴID
					 AA_TXData[12]=ST_ID;      //Ŀ��ID
	
					 for(ii=6;ii<13;ii++)//У��
					 {
						Sum1+=AA_TXData[ii];
					 }
					 Sum2=256-Sum1&0xff;
					
					 AA_TXData[13]=256-Sum2;//У��
					 AA_TXData[14]=0X5A;
					 AA_TXData[15]=0XFE;
					 AA_TX_Len=16;	
					 AA_Com_Write();					
				
				}
			}
			else  //У��ʹ����
			{
				 AA_Err=1;
				 AA_TXData[0]=0X5A;//֡ͷ
				 AA_TXData[1]=0X54;//֡ͷ
				 AA_TXData[2]=0X17;//�����豸 0x17
				 AA_TXData[3]=0X10;//�����豸0x10
				 AA_TXData[4]=0XE0;//������ST
				 AA_TXData[5]=0X07;//���ݳ���SL
				
				 AA_TXData[6]=Time&0x000000ff;
			   AA_TXData[7]=Time>>8&0x000000ff;
			   AA_TXData[8]=Time>>16&0x000000ff;
			   AA_TXData[9]=Time>>24&0x000000ff;
				 //����
	       AA_TXData[10]=AA_Data[AA_RX_Len-3];    //���յ�����֤��ֵ
				 AA_TXData[11]=Sum2;                    //��վ���ɵ���֤��ֵ
				 
				 AA_TXData[12]=0XAA;                    //����
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
		else  //֡β����
		{    
			 AA_Err=2;
			 AA_TXData[0]=0X5A;//֡ͷ
			 AA_TXData[1]=0X54;//֡ͷ
			 AA_TXData[2]=0X17;//�����豸 0x17
			 AA_TXData[3]=0X10;//�����豸0x10
			 AA_TXData[4]=0XE5;//������ST                                 ��
			 AA_TXData[5]=0X07;//���ݳ���SL
			
			 AA_TXData[6]=Time&0x000000ff;
			 AA_TXData[7]=Time>>8&0x000000ff;
			 AA_TXData[8]=Time>>16&0x000000ff;
			 AA_TXData[9]=Time>>24&0x000000ff;
			 /////����
			 AA_TXData[10]=AA_Data[AA_RX_Len-3];
			 AA_TXData[11]=Sum2;
			 
			 AA_TXData[12]=0XAA;//����
             for(ii=6;ii<(13);ii++)//У��
			 {
				Sum1+=AA_TXData[ii];
			 }
			 Sum2=256-Sum1&0xff;
			 AA_TXData[13]=Sum2;//У��
			 AA_TXData[14]=0X5A;
			 AA_TXData[15]=0XFE;
			 AA_TX_Len=16;	
			 AA_Com_Write();						
		}
	}
	else   //֡ͷ����
	{
		AA_Err=3;
		AA_TXData[0]=0X5A;//֡ͷ
		AA_TXData[1]=0X54;//֡ͷ
		AA_TXData[2]=0X17;//�����豸 0x17
		AA_TXData[3]=0X10;//�����豸0x10
		AA_TXData[4]=0XE0;//������ST                                ��
		AA_TXData[5]=0X03;//���ݳ���SL
		 /////����
		AA_TXData[6]=AA_Data[AA_RX_Len-3];
		AA_TXData[7]=Sum2;
		 
		AA_TXData[8]=0XAA;//����
		for(ii=6;ii<(13);ii++)//У��
		{
			Sum1+=AA_TXData[ii];
		}
		Sum2=256-Sum1&0xff;
		AA_TXData[9]=Sum2;//У��
		AA_TXData[10]=0X5A;
		AA_TXData[11]=0XFE;
		AA_TX_Len=12;	
		AA_Com_Write();						
	}

}

/////////////////////////////////////////�������������ݰ�������///////////////////////////////////////////////////////////////////
void AA_Data_CL(void)       //��֡ͷ֡β��У�鶼ͨ����ִ�������������;
{
	QIUhe_CNT++;
	u8 i=0,Sum2=0;
	u32 Sum=0;
	union UintToArray T;///
	switch(CommandNew)
	{
//�Լ�		
		case 0x30://�Լ�
		{
			self_check();
	        ZiJian_YingDa();
			break;
		}
//����װ��
		case 0x31://����װ��
		{
			if(Work_Mode==0x03)    //Work_Mode==0x03Ϊ����״̬��ֻ���ڴ���״̬�²ſ��Բ���װ��
			{
				parameter_record();
				Send_Flag = 1;
				
			}
			else                   //��������ڴ���������յ���װ��ָ����ش������
			{
				parameter_record_error();	    //�������ɶ			
			}
			break;
		}
//״̬��ѯ���ظ�һЩ״̬��		
		case 0x32://״̬��ѯ���ظ�һЩ״̬��
		{
			state_inquiry();
			
			ZhuangTaiChaXun_YingDa();
			break;
		}
//����ģʽ�л�		
		case 0x33://����ģʽ�л�
		{
			if(CommandOld==0x39)       //����ģʽ�л�ǰ�����ȷ���ͣ
			{
				Work_Mode=AA_Data[10];  //����ģʽ
				if(Work_Mode_Old == Work_Mode)
				{
					Clearn_flag = 1;                //���ι���ģʽ��ͬ�������־λ
				}
				else
				{
					Clearn_flag = 0;               //���ι���ģʽ��ͬ�����־λ
				}
				if(Clearn_flag == 0)
				{
					/////////////////////�����ͬ������/////////////////////////////////	
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
				
				FW_position=AryToFloat(AA_Data[11],AA_Data[12],AA_Data[13],AA_Data[14]);//��λλ��
				FY_position=AryToFloat(AA_Data[15],AA_Data[16],AA_Data[17],AA_Data[18]);//����λ��
				Scan_direction=AA_Data[19];//ɨ�跽��
				Scan_Direction_ly = Scan_direction;
				WaiTongBu_Scan_Direction = Scan_direction;
				YunSu_Direction_ly = Scan_direction;
				Scan_times=AA_Data[20];    //  ɨ�����Give_Scan_Number
				Give_Scan_Number = Scan_times;
				Scan_FW_Start_Angle=AA_Data[21]; //ɨ����ʼ��λ��
				Scan_FY_Start_Angle=AA_Data[22]; //��ɨ����ʼ������
				Scan_FW_Angle=AA_Data[23]; //ɨ�跽λ��
				Scan_FY_Angle=AA_Data[24]; //��ɨ�踩����
				
				FW_Scan_Step=(AA_Data[26]*256) + AA_Data[25];     //ɨ�貽��
				FW_Scan_Step_Float=(float)FW_Scan_Step/1000;
				
				FY_Scan_Step=(AA_Data[28]*256) + AA_Data[27];     //ɨ�貽��
				FY_Scan_Step_Float=(float)FY_Scan_Step/1000;
//				FW_Scan_Step=ArrayToInt161(AA_Data[25],AA_Data[26]);     //ɨ�貽��
//				FY_Scan_Step=ArrayToInt161(AA_Data[27],AA_Data[28]);     //ɨ�貽��
				Scan_Speed=AA_Data[29];        //ɨ���ٶ�
//				FW_yunsu_sudu = (float)Scan_Speed;
//				FY_yunsu_sudu = Scan_Speed;
				Scan_FangShi=AA_Data[30];     //ɨ�跽ʽ 
				if( (Scan_FangShi != 0 && Work_Mode == 0x01) ||(Scan_FangShi != 0 && Work_Mode == 0x05))      //��ͬ��ɨ�裬��ͬ��ɨ�裬����ɨ��
				{
					if((Scan_FW_Start_Angle<-60)||(Scan_FW_Start_Angle>60)) ////ɨ�跽λ�Ƕ��ж�
					{
						scan_FW_degree_judge();
						break;///					 
					}	
					if((Scan_FY_Start_Angle<-60)||(Scan_FY_Start_Angle>60))/////ɨ�踩���Ƕ��ж�	
					{
						scan_FY_degree_judge();
						break;///						 
					}
					//////ȡ��
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
				
				if((Scan_FangShi != 0 && Work_Mode == 0x01)||(Scan_FangShi != 0 && Work_Mode == 0x05))      //��ͬ��ɨ�裬��ͬ��ɨ�裬����ɨ��
				{
					if((Scan_FW_Angle<0)||(Scan_FW_Angle>60)) ////ɨ�跽λ�Ƕ��ж�
					{
						scan_FW_degree_judge_1();
						break;///					 
					}	
					if((Scan_FY_Angle<0)||(Scan_FY_Angle>60))/////ɨ�踩���Ƕ��ж�	
					{
						scan_FY_degree_judge_1();
						break;///						 
					}
					//////ȡ��
					ScanAngleRang_ly = (float)Scan_FW_Angle;
					ScanAngleRang_ly = ScanAngleRang_ly * 0.5;//��Χ��������
					YunSuAngleRang_ly = (float)Scan_FW_Angle;
					
					if((Scan_FW_Angle % 2) == 0)
					{
						WaiTongBu_ScanAngleRang = (int)Scan_FW_Angle;
					}
					if((Scan_FW_Angle % 2) == 1)
					{
						WaiTongBu_ScanAngleRang = (int)Scan_FW_Angle + (int)1;					
					}
					
					

					WaiTongBu_ScanAngleRang = WaiTongBu_ScanAngleRang * 0.5; //��Χ��������
//					ScanAngleRang = (int)Scan_FW_Angle;
				}			
				
				if((Scan_FangShi != 0 && Work_Mode == 0x01) || (Scan_FangShi != 0 && Work_Mode == 0x05))   //��ͬ��ɨ�裬��ͬ��ɨ��
				{
					if((FW_Scan_Step_Float<0.1)||(FW_Scan_Step_Float>10))////��λɨ�貽���ж�
					{
						scan_FW_step_judge();
						break;///							 
					}
					if((FY_Scan_Step_Float<0.1)||(FY_Scan_Step_Float>10))///����ɨ�貽���ж�
					{
						scan_FY_step_judge();
						break;///						 
					}
					//////ȡ��
					FW_ScanStepLength_ly = FW_Scan_Step_Float;
					WaiTongBu_FW_ScanStepLength = FW_Scan_Step_Float;
					WaiTongBu_FY_ScanAngleRang = FY_Scan_Step_Float;
//					FW_ScanStepLength = FW_Scan_Step_Float;
//					FY_ScanAngleRang = FY_Scan_Step_Float;
				}
				if(Scan_FangShi == 0x03 && Work_Mode == 0x01)     //����ɨ��
				{  
					if((Scan_Speed<10)||(Scan_Speed>60))///ɨ���ٶ�	
					{
						scan_speed();
						break;///						 
					}
				//////ȡ��
					Scan_Speed_Degree_s = Scan_Speed * 0.001;
					FW_YunSuStepLength_ly = Scan_Speed_Degree_s;                      //��/1ms
//					FW_yunsu_sudu = (float)Scan_Speed * 0.058;
//					FW_yunsu_sudu_1 = (float)Scan_Speed * 0.058;
				}
	


				//Ӧ����         //��ȷӦ��
				work_mode_change_ack();
			}
			
			else   //����ģʽ�л�ǰ�����ȷ���ͣ
			{
				work_mode_change_error();
			}
			break;//
		}
//����ָ��		
		case 0x34://����ָ��
		{	
			if(Work_Mode==0x03 ||Work_Mode==0x04)   //�ڴ���ģʽ�»��˹�����ģʽ
			{
				start_point_to();
			    ZhiXiang_YingDa();
//				system_mode = 12;       // ��    				
			}		
			else   //�ڷ�ָ����ģʽ�£�����ָ��
			{
				start_point_to_error();	
			}		
			break;
		}
//��������ɨ��		
		case 0x35://��������ɨ��
		{
			if(Work_Mode==0x01|| Work_Mode==0x03||Work_Mode==0x05)// ɨ�裬��ɨ�߸��٣�����
			{
				if(Scan_FangShi == 0x01)                     //��ͬ��ɨ��
				{
					start_step_scan();
					BuJinSaoMiao_YingDa();
					system_mode = 11;
//					system_mode = 5;          // ��
//					step_open_flag = 1;
				}
				if(Scan_FangShi == 0x02)                   //��ͬ��ɨ��
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
//��������		
		case 0x36://��������
		{
			if(Work_Mode==0x02 || Work_Mode==0x03)   //���ڸ���ģʽ�����
			{
				start_track();
				GenZong_YingDa();
				if(QieHuan_flag == 1)
				{
					system_mode = 7; 
				}
			}
			else                  //���ڷǸ���ģʽ
            {
				start_track_error();
			}				
			break;
		}
//��������ɨ��
		case 0x37://��������ɨ��
		{
			if(Work_Mode==0x01 || Work_Mode==0x03 || Work_Mode==0x05)   // ����ɨ��ģʽ����������ɨ�߸���
			{
//				system_mode = 8;
				system_mode = 12;
				start_constantspeed_scan();
				YunSuSaoMiao_YingDa();
			}
			else                  // ���ڷ�����ɨ��ģʽ
			{
				start_constantspeed_scan_error();			
			}		
			break;
		}
//����		
		case 0x38://����
		{
			if(CommandOld == 0x39)
			{
//				if(Work_Mode==0x03)   //���ڴ���ģʽ
//				{
					GuiLingFlag = 0;
					FW_Huiling_last = FW_encoder_degrees;
					FY_Huiling_last = FY_encoder_degrees;				
					system_mode = 3; // ��
					make_zero();
					HuiLing_YingDa();
//				}
//			  else                  //���ڷǴ���ģʽ
//        {
//					make_zero_error();	
//		  	}				
				break;
			}
			else   //����ģʽ�л�ǰ�����ȷ���ͣ
			{
				work_mode_change_error();
			}
			break;//
		}
//��ͣ		
		case 0x39://��ͣ
		{ 
			system_mode = 1;  
			scram_button();	
			JiTing_YingDa();
			JiTing_ZhongDuan_CNT = 0;
//			JiTing_Set_CNT=0;
			break;
		}
//��������		
		case 0x3a://��������
		{	
			if(Work_Mode==0x03)     //���ڴ���ģʽ 
			{
				driver_setting();
			}
			else                    //���ڷǴ���ģʽ 
			{
				driver_set_error();	
			}				
			
			break;
		}
//�ߵ�����
		case 0x3b://�ߵ�����
		{
			if(Work_Mode==0x03)     //���ڴ���ģʽ    
			{
				navigation_set();
			}
			else                    //���ڷǴ���ģʽ
            {
				navigation_set_error();	
			}				
			break;
		}	
//�ɼ�����
		case 0x3c://�ɼ�����
		{
			bbb++;
			//			if((Work_Mode==0x01)||(Work_Mode==0x02)||(Work_Mode==0x05))///ɨ������
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
//��ʱ		
		case 0x3d://��ʱ
		{	
			check_the_time();
			break;
		}
//ͨѶ����		
		case 0x3e://ͨѶ����
		{	
//			if(Work_Mode==0x03)   //����ģʽ�¿ɷ���ͨ�Ų�������   
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
		
	}//switch������

}
void AA_Data_TX(void)
{
	AA_Com_Write();
}

/////////////////////////////////////////�������������ݡ�����////////////////////////////////////////////////////////////////////////////
//��ͣ
void scram_button()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //�Լ�ģʽ
	
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
	write_JingGenZong_Data();            //����������	
	CommandOld=CommandNew;	
}

//����
void make_zero()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x00;          //����ģʽ
	
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
	write_JingGenZong_Data();            //����������	
	CommandOld=CommandNew;
}
void make_zero_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
    AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
    //����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x37;
		 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;     //У���ֽ�
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void start_point_to()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	Point_Mode=AA_Data[10];////ָ��ģʽ
	RX_Coarse_Azimuth=AryToFloat(AA_Data[11],AA_Data[12],AA_Data[13],AA_Data[14]);//�ָ����ŷ���λλ��
	//SYS_Time_T=AA_Data[12];//
	//SYS_Time_M=AA_Data[13];//
	//SYS_Time_S=AA_Data[14];//
	//SYS_Time_mS_H=AA_Data[15];//
	RX_Coarse_Pitch=AryToFloat(AA_Data[15],AA_Data[16],AA_Data[17],AA_Data[18]);//�ָ����ŷ�����λ��
	//SYS_Time_mS_L=AA_Data[16];//
	//SYS_Time_T=AA_Data[17];//
	//SYS_Time_M=AA_Data[18];//
	//SYS_Time_S=AA_Data[19];//

	RX_Precise_Azimuth=AryToFloat(AA_Data[19],AA_Data[20],AA_Data[21],AA_Data[22]);//�������ŷ���λλ��
	//SYS_Time_mS_H=AA_Data[20];//
	//SYS_Time_mS_L=AA_Data[21];//
	//SYS_Time_T=AA_Data[22];//?
	//SYS_Time_M=AA_Data[23];//

	RX_Precise_Pitch=AryToFloat(AA_Data[23],AA_Data[24],AA_Data[25],AA_Data[26]);//�������ŷ�����λ��
	//SYS_Time_S=AA_Data[24];//
	//SYS_Time_mS_H=AA_Data[25];//
	//SYS_Time_mS_L=AA_Data[26];//
	//SYS_Time_mS_L=AA_Data[27];//
	//=AA_Data[27];//����
	
//	FW_pointing_degree_set = RX_Coarse_Azimuth - FW_zero_degree_zheng; // ��
//	FY_pointing_degree_set = RX_Coarse_Pitch - FY_zero_degree;   // ��
	
//	SpaceSetValueFW = RX_Coarse_Azimuth;
//	SpaceSetValueFY = RX_Coarse_Pitch;
	KongJianZhiXiang_FW = RX_Coarse_Azimuth;
    KongJianZhiXiang_FY = RX_Coarse_Pitch;
	if(Point_Mode==0x01)      //���ָ���ָ��
	{
	//	system_mode=4;
	  	system_mode=14;
		
		DaBao_Send_Data();
		DaBaoFaSong_Data[35] = 0X05;                //�������Լ�ģʽ
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
		write_JingGenZong_Data();            //����������	
	}
	
	if(Point_Mode==0x02)     //��������ָ��
	{
//		system_mode=1;
		DaBao_Send_Data();
		DaBaoFaSong_Data[19] = AA_Data[19];         //�������ŷ���λλ��
		DaBaoFaSong_Data[20] = AA_Data[20];
		DaBaoFaSong_Data[21] = AA_Data[21];
		DaBaoFaSong_Data[22] = AA_Data[22];
		
		DaBaoFaSong_Data[23] = AA_Data[23];         //�������ŷ�����λ��
		DaBaoFaSong_Data[24] = AA_Data[24];
		DaBaoFaSong_Data[25] = AA_Data[25];
		DaBaoFaSong_Data[26] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X01;                //������ָ��ģʽ
		
		
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
		write_JingGenZong_Data();            //����������		
	}
	
  if(Point_Mode==0x03)     //��/������ָ��
	{
		system_mode=14;
	//	system_mode=4;
		DaBao_Send_Data();
		DaBaoFaSong_Data[19] = AA_Data[19];         //�������ŷ���λλ��
		DaBaoFaSong_Data[20] = AA_Data[20];
		DaBaoFaSong_Data[21] = AA_Data[21];
		DaBaoFaSong_Data[22] = AA_Data[22];
		
		DaBaoFaSong_Data[23] = AA_Data[23];         //�������ŷ�����λ��
		DaBaoFaSong_Data[24] = AA_Data[24];
		DaBaoFaSong_Data[25] = AA_Data[25];
		DaBaoFaSong_Data[26] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X01;                //������ָ��ģʽ
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
		write_JingGenZong_Data();            //����������	
	}
	
	if(Point_Mode==0x04)     //������У׼����
	{
		system_mode=1;
		DaBao_Send_Data();
		DaBaoFaSong_Data[11] = AA_Data[19];         //�������ŷ���λλ��
		DaBaoFaSong_Data[12] = AA_Data[20];
		DaBaoFaSong_Data[13] = AA_Data[21];
		DaBaoFaSong_Data[14] = AA_Data[22];
		
		DaBaoFaSong_Data[15] = AA_Data[23];         //�������ŷ�����λ��
		DaBaoFaSong_Data[16] = AA_Data[24];
		DaBaoFaSong_Data[17] = AA_Data[25];
		DaBaoFaSong_Data[18] = AA_Data[26];
    
		DaBaoFaSong_Data[35] = 0X03;                //������У׼����
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
		write_JingGenZong_Data();            //����������		
	}
	if(Point_Mode==0x05)      //�ָ��ٱ�����ָ��
	{
	  	system_mode=17;
		
		FW_pointing_degree_LY = RX_Coarse_Azimuth;
		Point_FY_set_prepass = RX_Coarse_Pitch;
		//FY_pointing_degree_set = RX_Coarse_Pitch;
		DaBao_Send_Data();
		DaBaoFaSong_Data[35] = 0X05;                //�������Լ�ģʽ
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
		write_JingGenZong_Data();            //����������	
	}
	
		CommandOld=CommandNew;
}
void start_point_to_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
			 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x34;

	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
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
	DaBaoFaSong_Data[35] = 0x05;          //�Լ�ģʽ
	
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
	write_JingGenZong_Data();            //����������	
	CommandOld=CommandNew;
	
	
	
	
}
void parameter_record()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	union UintToArray T;//
	
	//���鵽����
	RX_Coarse_Azimuth=AryToFloat(AA_Data[10],AA_Data[11],AA_Data[12],AA_Data[13]);//�ָ����ŷ���λ ����λ��
	//SYS_Time_T=AA_Data[11];//
	//SYS_Time_M=AA_Data[12];//
	//SYS_Time_S=AA_Data[13];//
	//SYS_Time_mS_H=AA_Data[14];//
	RX_Coarse_Pitch=AryToFloat(AA_Data[14],AA_Data[15],AA_Data[16],AA_Data[17]);//�ָ����ŷ����� ����λ��
	//SYS_Time_mS_L=AA_Data[15];//
	//SYS_Time_T=AA_Data[16];//
	//SYS_Time_M=AA_Data[17];//
	//SYS_Time_S=AA_Data[18];//

	RX_Precise_Azimuth=AryToFloat(AA_Data[18],AA_Data[19],AA_Data[20],AA_Data[21]);//�������ŷ���λ ����λ��
	//SYS_Time_mS_H=AA_Data[19];//
	//SYS_Time_mS_L=AA_Data[20];//
	//SYS_Time_T=AA_Data[21];//?
	//SYS_Time_M=AA_Data[22];//

	RX_Precise_Pitch=AryToFloat(AA_Data[22],AA_Data[23],AA_Data[24],AA_Data[25]);//�������ŷ����� ����λ��
	//SYS_Time_S=AA_Data[23];//
	//SYS_Time_mS_H=AA_Data[24];//
	//SYS_Time_mS_L=AA_Data[25];//
	//SYS_Time_mS_L=AA_Data[26];//

	RX_Coarse_Azimuth1=AryToFloat(AA_Data[26],AA_Data[27],AA_Data[28],AA_Data[29]);//��װ��б��
	//SYS_Time_T=AA_Data[27];//
	//SYS_Time_M=AA_Data[28];//
	//SYS_Time_S=AA_Data[29];//
	//SYS_Time_mS_H=AA_Data[30];//

	RX_Coarse_Pitch1=AryToFloat(AA_Data[30],AA_Data[31],AA_Data[32],AA_Data[33]);//�ָ��ٹ��Ჹ������ �Ƕ�
	//SYS_Time_mS_L=AA_Data[31];//
	//SYS_Time_T=AA_Data[32];//
	//SYS_Time_M=AA_Data[33];//
	//SYS_Time_S=AA_Data[34];//

	RX_Precise_Azimuth1=AryToFloat(AA_Data[34],AA_Data[35],AA_Data[36],AA_Data[37]);//�����ٹ��Ჹ����λ�Ƕ�
	//SYS_Time_mS_H=AA_Data[35];//
	//SYS_Time_mS_L=AA_Data[36];//
	//SYS_Time_T=AA_Data[37];//
	//SYS_Time_M=AA_Data[38];//

	RX_Precise_Pitch1=AryToFloat(AA_Data[38],AA_Data[39],AA_Data[40],AA_Data[41]);//�����ٹ��Ჹ�������Ƕ�
	//SYS_Time_S=AA_Data[39];//
	//SYS_Time_mS_H=AA_Data[40];//
	//SYS_Time_mS_L=AA_Data[41];//
	//SYS_Time_mS_L=AA_Data[41];//

	Step_T=AA_Data[42]+AA_Data[43]*256;//�������ʱ��
	FW_Step_Angle=AA_Data[44]+AA_Data[45]*256;//��λÿ�β�����
	FY_Step_Angle=AA_Data[46]+AA_Data[47]*256;//����ÿ�β�����

	T.ary[0]=AA_Data[48];//
	T.ary[1]=AA_Data[49];// 
	T.ary[2]=AA_Data[50];//
	T.ary[3]=AA_Data[51];//
	Stable_T=T.k;              //������ary to uint

	FW_Angle_MAX1=AA_Data[52];//ɨ�����λ��1
	FW_Angle_MAX2=AA_Data[53];//ɨ�����λ��2

	//
	FY_Angle_MAX1=AA_Data[54];////ɨ���������1
	FY_Angle_MAX2=AA_Data[55];////ɨ���������2

	
	////////////////////////////дFLASH����////////////////////////////////////////////////////////////////////
	PZdataWX[0] = AA_Data[10];                 //�ָ����ŷ���λ ����λ��
	PZdataWX[1] = AA_Data[11];
	PZdataWX[2] = AA_Data[12];
	PZdataWX[3] = AA_Data[13];
	
	PZdataWX[4] = AA_Data[14];                 //�ָ����ŷ����� ����λ��
	PZdataWX[5] = AA_Data[15];
	PZdataWX[6] = AA_Data[16];
	PZdataWX[7] = AA_Data[17];
	
	PZdataWX[8] = AA_Data[26];                 //��б��װ��
	PZdataWX[9] = AA_Data[27];
	PZdataWX[10] = AA_Data[28];	
	PZdataWX[11] = AA_Data[29];		

	PZdataWX[12] = AA_Data[30];                //�ָ��ٹ��Ჹ������ �Ƕ�
	PZdataWX[13] = AA_Data[31];
	PZdataWX[14] = AA_Data[32];	
	PZdataWX[15] = AA_Data[33];		
 	
	PZdataWX[16] = AA_Data[42];                //�������ʱ��
	PZdataWX[17] = AA_Data[43];
	
	PZdataWX[18] = AA_Data[44];	              //��λÿ�β�����
	PZdataWX[19] = AA_Data[45];	
	
	PZdataWX[20] = AA_Data[46];	              //����ÿ�β�����
	PZdataWX[21] = AA_Data[47];	
	
	PZdataWX[22] = AA_Data[48];	              //�ȶ�ʱ��
	PZdataWX[23] = AA_Data[49];	
	PZdataWX[24] = AA_Data[50];	
	PZdataWX[25] = AA_Data[51];	
	
	PZdataWX[26] = AA_Data[52];	              //ɨ�����λ��1
	PZdataWX[27] = AA_Data[53];	              //ɨ�����λ��2
	
	PZdataWX[28] = AA_Data[54];	              //ɨ���������1
	PZdataWX[29] = AA_Data[55];		          //ɨ���������2	
	
	PZdataWX[30] = AA_Data[18];	              //�������ŷ���λ ����λ��
	PZdataWX[31] = AA_Data[19];		           	
	PZdataWX[32] = AA_Data[20];	              
	PZdataWX[33] = AA_Data[21];		            
	
	PZdataWX[34] = AA_Data[22];	              //�������ŷ����� ����λ��
	PZdataWX[35] = AA_Data[23];		           	
	PZdataWX[36] = AA_Data[24];	              
	PZdataWX[37] = AA_Data[25];		  
	

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	//{��ֵ�Ӵ˴�}
//	FW_zero_degree_zheng = RX_Coarse_Azimuth;            //�ָ����ŷ���λ ����λ��
//	FY_zero_degree = RX_Coarse_Pitch;                    //�ָ����ŷ����� ����λ��
//	FY_light_loop_revise = RX_Coarse_Pitch1;             // �ָ��ٹ��Ჹ�������Ƕ�
//	FW_light_loop_revise = RX_Coarse_Azimuth1;           // �ָ��ٹ��Ჹ����λ�Ƕ�
    Time_Long	=	(int)Step_T;                      //�������ʱ��
    FW_ScanStepLength =    (float)FW_Step_Angle * (0.001);        //��λÿ�β����Ƕ�
    FY_ScanAngleRang  =    (float)FY_Step_Angle * (0.001);        //����ÿ�β����Ƕ�
		NewAngle = RX_Coarse_Azimuth1;
//    FW_Scan_StartAngle =    FW_zero_degree_zheng;       //ɨ����ʼ�Ƕ�
//	ScanAngleRang	=	(int)FW_Angle_MAX2 - (int)FW_Angle_MAX1;//ɨ�跶Χ


	
	DaBao_Send_Data();
	DaBaoFaSong_Data[3]  = AA_Data[18];                   //�������ŷ���λ ����λ��
	DaBaoFaSong_Data[4]  = AA_Data[19];
	DaBaoFaSong_Data[5]  = AA_Data[20];
	DaBaoFaSong_Data[6]  = AA_Data[21];
	
	DaBaoFaSong_Data[7]  = AA_Data[22];                   //�������ŷ����� ����λ��
	DaBaoFaSong_Data[8]  = AA_Data[23];
	DaBaoFaSong_Data[9]  = AA_Data[24];
	DaBaoFaSong_Data[10] = AA_Data[25];

	DaBaoFaSong_Data[11]  = AA_Data[34];                   //�����ٹ��Ჹ����λ�Ƕ�
	DaBaoFaSong_Data[12]  = AA_Data[35];
	DaBaoFaSong_Data[13]  = AA_Data[36];
	DaBaoFaSong_Data[14]  = AA_Data[37];

	DaBaoFaSong_Data[15]  = AA_Data[38];                   //�����ٹ��Ჹ�������Ƕ�
	DaBaoFaSong_Data[16]  = AA_Data[39];
	DaBaoFaSong_Data[17]  = AA_Data[40];
	DaBaoFaSong_Data[18]  = AA_Data[41];

	DaBaoFaSong_Data[35]  = 0X04;                         //����װ��ģʽ
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
	write_JingGenZong_Data();            //����������


	
	//Ӧ��///
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X41;    //����װ��Ӧ��
	AA_TXData[5]=0X05;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
	
	AA_TXData[10]=0X00;//װ���ɹ�0XAAʧ��0XEE        ��
	//AA_TXData[11]=0XAA;//����
    //AA_TXData[12]=0XAA;//����

	for(i=6;i<(11);i++)    //�������ֽ��ۼӣ�i���ֽ�������1��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;    //У���ֽ�
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
    AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE1;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
	 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x31;
	 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
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
	write_JingGenZong_Data();            //����������	
	CommandOld=CommandNew;
}
void start_step_scan()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	DaBao_Send_Data();	
	DaBaoFaSong_Data[35] = 0x05;          //�Լ�ģʽ
	
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
	write_JingGenZong_Data();            //����������	
	CommandOld=CommandNew;
}
void start_step_scan_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	
			 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x35;
			
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
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
	DaBaoFaSong_Data[35] = 0x02;          //����ģʽ
	
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
	write_JingGenZong_Data();            //����������		
	CommandOld=CommandNew;
}
void start_track_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
    AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
    AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
    AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
    AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	
			 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x36;
			 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
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
	DaBaoFaSong_Data[35] = 0x05;          //�Լ�ģʽ
	
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
	write_JingGenZong_Data();            //����������	
		CommandOld=CommandNew;
}

void start_constantspeed_scan_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
    AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
    AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
    AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
    AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ���� 
		 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x35;
		
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();			
}
void driver_setting()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;
	Driver_Set_Mode=AA_Data[11];///���������÷�ʽ
	Driver_Set1=AA_Data[12];///�ŷ�����������������
	Driver_Set2=AA_Data[13];///�ŷ�����������������
	Driver_Set3=AA_Data[14];///�ŷ�����������������
	Driver_Set4=AA_Data[15];///�ŷ�����������������
	Driver_Set5=AA_Data[16];///�ŷ�����������������
	Driver_Set6=AA_Data[17];///�ŷ�����������������
	//=AA_Data[18];///����

	///Ӧ��///
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

	AA_TXData[10]=0XAA;//����
	for(i=6;i<(11);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;//У��
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
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=7;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;
		 //����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x39;
		 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //У���ֽ�
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void navigation_set()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	/*///�ߵ�����������
	Driver_Set_Mode=AA_Data[11];///���������÷�ʽ
	Driver_Set1=AA_Data[12];///�ŷ�����������������
	Driver_Set2=AA_Data[13];///�ŷ�����������������
	Driver_Set3=AA_Data[14];///�ŷ�����������������
	Driver_Set4=AA_Data[15];///�ŷ�����������������
	Driver_Set5=AA_Data[16];///�ŷ�����������������
	Driver_Set6=AA_Data[17];///�ŷ�����������������
	//=AA_Data[18];///����
	*/

	//�ߵ�Ӧ��
	AA_TXData[0]=0X5A;
	AA_TXData[1]=0X54;
	AA_TXData[2]=0X17;
	AA_TXData[3]=0X10;
	AA_TXData[4]=0X4b;    //�ߵ�����Ӧ��
	AA_TXData[5]=5;
	
    AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=0XAA;//�ָ���״̬��
	for(i=6;i<(11);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;     //У���ֽ�
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
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=7;//���ݳ���SL
	
    AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
    //����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x3a;
		 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //У���ֽ�
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}
void collecte_command()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	FW_Miss_distance=AryToFloat(AA_Data[10],AA_Data[11],AA_Data[12],AA_Data[13]);//��λ�Ѱ���
	FY_Miss_distance=AryToFloat(AA_Data[14],AA_Data[15],AA_Data[16],AA_Data[17]);//�����Ѱ���
//	RX_Servo_Steady_State=AA_Data[19];///�ŷ���̬

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
	DaBaoFaSong_Data[27] = AA_Data[10];     //��λ
	DaBaoFaSong_Data[28] = AA_Data[11];
	DaBaoFaSong_Data[29] = AA_Data[12];
	DaBaoFaSong_Data[30] = AA_Data[13];
	
	DaBaoFaSong_Data[31] = AA_Data[14];     //����
	DaBaoFaSong_Data[32] = AA_Data[15];
	DaBaoFaSong_Data[33] = AA_Data[16];
	DaBaoFaSong_Data[34] = AA_Data[17];
	
	if(CommandNew == 0x36)
	{
		DaBaoFaSong_Data[35] = 0x02;          //�ɼ�ģʽ
	}
	else
	{
		DaBaoFaSong_Data[35] = 0x06;          //�ɼ�ģʽ
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
	write_JingGenZong_Data();            //����������		
	CommandOld=CommandNew;
}
void collecte_command_error()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=7;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����		 
			 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0x3b;
			 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
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

	AA_TXData[10]=AA_Data[10];//�ָ���״̬��

	for(i=6;i<(11);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;    //У���ֽ�
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
	AA_TXData[4]=0X4e;     //ͨ�Ų���Ӧ��
	AA_TXData[5]=5;
	
	AA_TXData[6]=Time&0x000000ff;
	AA_TXData[7]=Time>>8&0x000000ff;
	AA_TXData[8]=Time>>16&0x000000ff;
	AA_TXData[9]=Time>>24&0x000000ff;

	AA_TXData[10]=0X00;    //��Ӧ��ʶ0xaa�ɹ�����0xeeʧ��
//	AA_TXData[11]=0XAA;
//	AA_TXData[12]=0XAA;
	for(i=6;i<(11);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;   //У���ֽ�
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
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE1;//������ST
	AA_TXData[5]=7;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;
    AA_TXData[7]=Time>>8&0x000000ff;
    AA_TXData[8]=Time>>16&0x000000ff;
    AA_TXData[9]=Time>>24&0x000000ff;
	//����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=0XAA;
			 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;    //У���ֽ�
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void switch_default()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE1;//������ST
	AA_TXData[5]=7;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����
			 /////����
	AA_TXData[10]=Work_Mode;
	AA_TXData[11]=CommandNew;
			 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();	
}

void scan_FW_degree_judge()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
			 /////����
	AA_TXData[10]=22;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FW_degree_judge_1()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
			 /////����
	AA_TXData[10]=24;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FY_degree_judge_1()
{
  u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
			 /////����
	AA_TXData[10]=25;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();

}

void scan_FY_degree_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	
	
		 /////����
	AA_TXData[10]=23;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_FW_step_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
	
    /////����
	AA_TXData[10]=26;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_FY_step_judge()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;		
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����	 
	
		 /////����
	AA_TXData[10]=28;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void scan_speed()
{
	u8 i=0,Sum2=0;
	u32 Sum=0;	
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE6;//������ST
	AA_TXData[5]=0X07;//���ݳ���SL
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ���� 
	
				 /////����
	AA_TXData[10]=30;
	AA_TXData[11]=0XAA;
				 
	AA_TXData[12]=0XAA;//����
	for(i=6;i<(13);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[13]=Sum2;//У��
	AA_TXData[14]=0X5A;
	AA_TXData[15]=0XFE;
	AA_TX_Len=16;	
	AA_Com_Write();
}

void work_mode_change_error()
{
    u8 i=0,Sum2=0;
	u32 Sum=0;
	AA_TXData[0]=0X5A;//֡ͷ
	AA_TXData[1]=0X54;//֡ͷ
	AA_TXData[2]=0X17;//�����豸 0x17
	AA_TXData[3]=0X10;//�����豸0x10
	AA_TXData[4]=0XE2;//������ST
	AA_TXData[5]=0X03;//���ݳ���SL
			 /////����
	AA_TXData[6]=AA_Data[11];
	AA_TXData[7]=CommandOld;
			 
	AA_TXData[8]=0XAA;//����
	for(i=6;i<(9);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[9]=Sum2;//У��
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
	
	AA_TXData[6]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
	AA_TXData[7]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
	AA_TXData[8]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
	AA_TXData[9]=Time>>24&0x000000ff;;//ϵͳʱ��  ����
	
	AA_TXData[10]=0XAA;//
	for(i=6;i<(11);i++)//У��
	{
		Sum+=AA_TXData[i];
	}
	Sum2=256-Sum&0xff;
	AA_TXData[11]=Sum2;//У��
	AA_TXData[12]=0X5A;
	AA_TXData[13]=0XFE;
	AA_TX_Len=14;	
	AA_Com_Write();
	CommandOld=CommandNew;	
}


//��ȡ�����ٷ�������
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
//�������ٴ�����
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

//У�����ݰ�
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

//������͵�����
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
//void AB_TX()////����ͨ��
//{
//		AB_TXData[0]=0X5B;//֡ͷ
//		AB_TXData[1]=0X54;//֡ͷ
//		
//		AB_TXData[2]=Time&0x000000ff;//ϵͳʱ�䡣ʱ
//		AB_TXData[3]=Time>>8&0x000000ff;//ϵͳʱ�䡣��
//		AB_TXData[4]=Time>>16&0x000000ff;;//ϵͳʱ��   ��
//		AB_TXData[5]=Time>>24&0x000000ff;;//ϵͳʱ��  ����
//	  AB_TXData[6]=48;
//	  AB_TXData[7]=0;
//	  AB_TXData[8]=02;
//	  if(Work_Mode == 0x03)                                            //����
//		{
//			AB_TXData[9] = 0x00;
//		}
//		else if(Work_Mode == 0x04)                                      //�˹�����
//		{
//			AB_TXData[9] = 0x01;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x03)             //����ɨ��
//		{
//			AB_TXData[9] = 0x02;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x02)            //��ͬ������ɨ��
//		{
//			AB_TXData[9] = 0x03;
//		}
//		else if(Work_Mode == 0x01 && Scan_FangShi == 0x01)          //��ͬ������ɨ��
//		{
//			AB_TXData[9] = 0x04;
//		}
//		else if(Work_Mode == 0x02)                                  //����
//		{
//			AB_TXData[9] = 0x05;
//		}
//	  
//	    Temp_Data.f=ppp;//�ָ����ŷ���λλ��
//     			
//		AB_TXData[10]=Temp_Data.ary[0];//�ָ��ٷ�λ ���Կռ�
//		AB_TXData[11]=Temp_Data.ary[1];//�ָ��ٷ�λ
//		AB_TXData[12]=Temp_Data.ary[2];//�ָ��ٷ�λ
//		AB_TXData[13]=Temp_Data.ary[3];//�ָ��ٷ�λ
//		
////	    TX_Coarse_Pitch=yyy;
//		Temp_Data.f=yyy;//�ָ����ŷ�����λ��	���Կռ�
//		AB_TXData[14]=Temp_Data.ary[0];//�ָ��ٸ���
//		AB_TXData[15]=Temp_Data.ary[1];//�ָ��ٸ���
//		AB_TXData[16]=Temp_Data.ary[2];//�ָ��ٸ���
//		AB_TXData[17]=Temp_Data.ary[3];//�ָ��ٸ���
//		
//		
//		Temp_Data.f=FW_encoder_degrees;//��������λ
////		if(ab_f>360.0)ab_f=0;
////		ab_f=ab_f+0.001;
////		Temp_Data.f=ab_f;
//		AB_TXData[18]=Temp_Data.ary[0];//
//		AB_TXData[19]=Temp_Data.ary[1];//
//		AB_TXData[20]=Temp_Data.ary[2];//
//		AB_TXData[21]=Temp_Data.ary[3];//
//		
//		Temp_Data.f=FY_encoder_degrees;//����������
////		Temp_Data.f=ab_f;
//		AB_TXData[22]=Temp_Data.ary[0];//
//		AB_TXData[23]=Temp_Data.ary[1];//
//		AB_TXData[24]=Temp_Data.ary[2];//
//		AB_TXData[25]=Temp_Data.ary[3];//	
//			
//		TX_Ins_Yaw=yaw_attitude_float;
//		Temp_Data.f=TX_Ins_Yaw;//���Կռ�ƫ�� 
//		AB_TXData[26]=Temp_Data.ary[0];//����ƫ��
//		AB_TXData[27]=Temp_Data.ary[1];//
//		AB_TXData[28]=Temp_Data.ary[2];//
//		AB_TXData[29]=Temp_Data.ary[3];//
//			
//		TX_Ins_RollAngle=roll_attitude_float;		
//		Temp_Data.f=TX_Ins_RollAngle;//���Կռ���
//		AB_TXData[30]=Temp_Data.ary[0];//���Ժ��
//		AB_TXData[31]=Temp_Data.ary[1];//
//		AB_TXData[32]=Temp_Data.ary[2];//
//		AB_TXData[33]=Temp_Data.ary[3];//
//			
//		TX_Ins_PitchAngle=pitch_attitude_float;
//		Temp_Data.f=TX_Ins_PitchAngle;//���Կռ丩��
//		AB_TXData[34]=Temp_Data.ary[0];//���Ը���
//		AB_TXData[35]=Temp_Data.ary[1];//
//		AB_TXData[36]=Temp_Data.ary[2];//
//		AB_TXData[37]=Temp_Data.ary[3];//
//				/////////
//		if(AB_CNT==2){
//		AB_TXData[38]=2;
//	//	AB_CNT=3;
//		Temp_Data.f=AN_1_fy ;//ELMO A�����
//		AB_TXData[39]=Temp_Data.ary[0];//
//		AB_TXData[40]=Temp_Data.ary[1];//
//		AB_TXData[41]=Temp_Data.ary[2];//
//		AB_TXData[42]=Temp_Data.ary[3];//	
//			
//		/////
//		Temp_Data.f=AN_2_fy;//ELMO B�����
//		AB_TXData[43]=Temp_Data.ary[0];//
//		AB_TXData[44]=Temp_Data.ary[1];//
//		AB_TXData[45]=Temp_Data.ary[2];//
//		AB_TXData[46]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_3_fy;///ELMO C�����	
//		AB_TXData[47]=Temp_Data.ary[0];//���Ժ��
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_4_fy;//ELMO ĸ�ߵ�ѹ
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
//		Temp_Data.f=VX_fw;//ELMO ��λ�ٶ�
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
//		Temp_Data.f=PX_fw;//ELMO��λ�Ƕ�
//		AB_TXData[47]=Temp_Data.ary[0];//
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//	
//		Temp_Data.f=PX_fy;//ELMO�����Ƕ�
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
//		Temp_Data.f=AN_1_fw;//ELMO A�����
//		AB_TXData[39]=Temp_Data.ary[0];//
//		AB_TXData[40]=Temp_Data.ary[1];//
//		AB_TXData[41]=Temp_Data.ary[2];//
//		AB_TXData[42]=Temp_Data.ary[3];//	
//			
//		/////
//		Temp_Data.f=AN_2_fw;// ELMO B�����
//		AB_TXData[43]=Temp_Data.ary[0];//
//		AB_TXData[44]=Temp_Data.ary[1];//
//		AB_TXData[45]=Temp_Data.ary[2];//
//		AB_TXData[46]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_3_fw;////ELMO C�����	
//		AB_TXData[47]=Temp_Data.ary[0];//���Ժ��
//		AB_TXData[48]=Temp_Data.ary[1];//
//		AB_TXData[49]=Temp_Data.ary[2];//
//		AB_TXData[50]=Temp_Data.ary[3];//
//			
//		
//		Temp_Data.f=AN_4_fw ;//ELMO ĸ�ߵ�ѹ
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
	void CJ()///����ͨ��
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
void AB_Com_Read()      //���жϱ�־λ���ȡFPGA���ֽڻ��Ϲߵ�
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
	AB_DATA_RX();        //���ж�֡ͷ֡β��У��
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
	if((AB_Data[0]==0x99) & (AB_Data[1]==0x66))//֡ͷ
	{
//		if((AA_Data[AA_RX_Len-2]==0x5a)&(AA_Data[AA_RX_Len-1]==0xfe))//֡β
//		{
			for(ii=2;ii<(45);ii++)   
			{
				Sum1+=AB_Data[ii];
			}
			Sum2=Sum1&0xff;   //sum1 = �������ֽ��ۼ�
			Sum2=256-Sum2;    //sum2 = У���ֽ�
			
			if(AB_Data[45]==Sum2)//���У��
			{
				
				UTS_Time1=AB_Data[3]+AB_Data[4]*256+AB_Data[5]*256*256+AB_Data[6]*256*256*256;//   ʱ��
				Lattitude_int=AB_Data[7]+AB_Data[8]*256+AB_Data[9]*256*256+AB_Data[10]*256*256*256;  //γ��
				Altitude_int=AB_Data[11]+AB_Data[12]*256+AB_Data[13]*256*256+AB_Data[14]*256*256*256;////�߶�
				Longitude_int=AB_Data[15]+AB_Data[16]*256+AB_Data[17]*256*256+AB_Data[18]*256*256*256;  ///����
				N_V=AB_Data[19]+AB_Data[20]*256;              ///�����ٶ�
				SKEY_V=AB_Data[21]+AB_Data[22]*256;            ///�����ٶ�
				E_V=AB_Data[23]+AB_Data[24]*256;             //�����ٶ�
				Roll=AB_Data[25]+AB_Data[26]*256;            ///�����
				Yaw=AB_Data[27]+AB_Data[28]*256;              ////�����
				Pitch=AB_Data[29]+AB_Data[30]*256;            ///������
				X_Palstance=AB_Data[31]+AB_Data[32]*256;      //X����ٶ�
				Y_Palstance=AB_Data[33]+AB_Data[34]*256;      //Y����ٶ�   
				Z_Palstance=AB_Data[35]+AB_Data[36]*256;       //Z����ٶ�
				X_Acceleration=AB_Data[37]+AB_Data[38]*256;   //X����ٶ�
				Y_Acceleration=AB_Data[39]+AB_Data[40]*256;  //Y����ٶ�
				Z_Acceleration=AB_Data[41]+AB_Data[42]*256;  //Z����ٶ�
				Status=AB_Data[43]+AB_Data[44]*256;   //////״̬��
// 
        Lattitude_f= (float)Lattitude_int*180.0/2147483646.0; //γ��
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

