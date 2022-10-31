#include "stm32f4xx.h"
#include "ASCII.h"

#define CAM_Com_Flag		0x0010					//相机串口接收标志位
#define CAM_ADDR            ((u32)(0x64002800))
#define CAM_Com_En          ((u32)(0x64002c00))
/*设置参数地址*/
#define Camera_Restart_Addr 								((u16)(0x1200))//重启相机
#define Save_Configuration_Addr								((u16)(0x1201))//保存当前配置
#define Reset_Addr											((u16)(0x1202))//恢复出厂设置
#define Exposure_Time_Addr1									((u16)(0x1205))//曝光时间低
#define Exposure_Time_Addr2									((u16)(0x1206))//曝光时间高
#define X_Migration_Addr									((u16)(0x120B))//X偏移
#define Y_Migration_Addr									((u16)(0x120C))//Y偏移
#define X_Length											((u16)(0x120D))//X长度
#define Y_Length											((u16)(0x120E))//Y长度
#define ADC_Gain											((u16)(0x1218))//ADC增益
#define PGA_Gain											((u16)(0x1220))//PGA增益
#define Tracking_Algorithm_Addr								((u16)(0x1302))//跟踪算法
#define Small_Window_Switch									((u16)(0x1303))//小窗口切换
#define Threshold_Addr										((u16)(0x1306))//阈值设置
#define Window_L											((u16)(0x0000))//大窗口
#define Window_S											((u16)(0x0001))//小窗口
#define Barycentre											((u16)(0x0000))//质心
#define Centroid											((u16)(0x0001))//形心

void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[]);
void Camera_Write(u8 Data[],u8 n);
void Restore_Factory_Settings();
void Save_Configuration_Settings();
