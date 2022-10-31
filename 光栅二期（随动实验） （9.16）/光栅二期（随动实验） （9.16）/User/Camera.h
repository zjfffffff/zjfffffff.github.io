#ifndef CAMERA_H
#define CAMERA_H
#include "stm32f4xx.h"
#include "ASCII.h"
#include "FSMC.h"
#include "delay.h"

/*设置参数地址*/
#define Camera_Restart_Addr 									((u16)(0x1200))//重启相机
#define Save_Configuration_Addr								((u16)(0x1201))//保存当前配置
#define Reset_Addr														((u16)(0x1202))//恢复出厂设置
#define Exposure_Time_Addr1										((u16)(0x1205))//曝光时间低
#define Exposure_Time_Addr2										((u16)(0x1206))//曝光时间高
#define X_Migration_Addr											((u16)(0x120B))//X偏移
#define Y_Migration_Addr											((u16)(0x120C))//Y偏移
#define X_Length															((u16)(0x120D))//X长度
#define Y_Length															((u16)(0x120E))//Y长度
#define ADC_Gain															((u16)(0x1218))//ADC增益
#define PGA_Gain															((u16)(0x1220))//PGA增益
#define Tracking_Algorithm_Addr								((u16)(0x1302))//跟踪算法
#define Small_Window_Switch										((u16)(0x1303))//小窗口切换
#define Threshold_Addr												((u16)(0x1306))//阈值设置


#define Window_L															((u16)(0x0000))//大窗口
#define Window_S															((u16)(0x0001))//小窗口
#define Barycentre														((u16)(0x0000))//质心
#define Centroid															((u16)(0x0001))//形心

#endif


extern u32 Exposure_Time;//曝光时间
extern u16 X_Migration_Setting;//设置X偏移
extern u16 Y_Migration_Setting;//设置Y偏移
extern u16 X_Length_Setting;//设置X长度
extern u16 Y_Length_Setting;//设置Y长度
extern u8 Windows_Switch;//切换窗口
extern u16 Threshold;//阈值
extern u8 Tracing_Algorithm;//跟踪算法
///////////////////////
extern u16 X_Migration_Setting1;//设置X偏移
extern u32 Exposure_Time1;//曝光时间
extern u16 X_Length_Setting1;//设置X长度
extern u16 Y_Length_Setting1;//设置Y长度
extern u16 Y_Migration_Setting1;//设置Y偏移
extern u16 Threshold1;//阈值
extern u8 Tracing_Algorithm1;//跟踪算法
extern u8 Windows_Switch1;//切换窗口

////////////////////
extern u16 Image_Threshold;//跟踪图像阈值
extern u8 Image_Gray_Value;//图像平均灰度值
extern u8 Background_Gray_Value;//背景平均灰度值
extern u8 Spot_Gray_Value;//光斑平均灰度值
extern u8 Camera_Window;//相机窗口状态
extern u8 Coarse_Image_Symbol;//图像粗跟踪标志
extern u8 Precise_Image_Symbol;//图像精跟踪标志
extern u8 Image_Algorithm;//图像算法
extern u32 Integral_Time;//积分时间

extern u8 Camera_Error,Camera_ADD_Error;
/////////////////////
extern u16 Image_Threshold1;//跟踪图像阈值
extern u8 Image_Gray_Value1;//图像平均灰度值
extern u8 Background_Gray_Value1;//背景平均灰度值
extern u8 Spot_Gray_Value1;//光斑平均灰度值
extern u8 Camera_Window1;//相机窗口状态
extern u8 Coarse_Image_Symbol1;//图像粗跟踪标志
extern u8 Precise_Image_Symbol1;//图像精跟踪标志
extern u8 Image_Algorithm1;//图像算法
extern u32 Integral_Time1;//积分时间

extern u8 Camera_Error1,Camera_ADD_Error1;
extern u8 Camera_Step,Camera_TX_falg;

////////////////



void Camera_Param_Set1(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer1(u32 ADDR,u8 Data[]);
void Camera_Answer_Param1(u16 Param_Addr,u16 Param_Data);
void Restore_Factory_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Save_Configuration_Settings1(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);

void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer(u32 ADDR,u8 Data[]);
void Camera_Answer_Param(u16 Param_Addr,u16 Param_Data);
void Restore_Factory_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Save_Configuration_Settings(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Set_chu(u32 ADDR,u8 Data[]);
void Camera_Param_Set2(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Set2(u32 ADDR,u8 Data[],u8 n,u32 Enable_ADDR);
void Camera_Answer2(u32 ADDR,u8 Data[]);
void Camera_Answer_Param2(u16 Param_Addr,u16 Param_Data);
