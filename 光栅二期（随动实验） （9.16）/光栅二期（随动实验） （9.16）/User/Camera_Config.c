#include "Camera_Config.h"
extern u8 CAMERA_Gray_Value;				//从相机读回的灰度值
u8 CAM_Data[30],CAM_Answer_Data[30];
u16 Threshold;//阈值
u8 Windows_Switch;
u8 Camera_Step;
u16 X_Length_Setting,Y_Length_Setting;
u32 Exposure_Time;//曝光时间

u32 Exposure_Time_temp;
u16 Threshold_temp=0;
u16 Gray_temp=0;
u8 Windows_Switch_temp=0;
u8 Camera_Step_temp=0;
u16 X_Length_Setting_temp=0,Y_Length_Setting_temp=0;
float X_Migration = 0;
float Y_Migration = 0;

void Camera_Param_Set(u16 Param_Addr,u16 Param_Data,u8 Data[])
{
	u8 Check=0;
	Data[0]=0x7E;//帧头
	/*帧长度*/
	Data[1]=0x30;
	Data[2]=0x45;
	/*设备地址*/
	Data[3]=0x36;
	Data[4]=0x30;
	/*设备参数地址*/ 
	Data[8]=Char_To_Hex((u8)(Param_Addr&0x000F));
	Data[7]=Char_To_Hex((u8)((Param_Addr&0x00F0)>>4));
	Data[6]=Char_To_Hex((u8)((Param_Addr&0x0F00)>>8));
	Data[5]=Char_To_Hex((u8)((Param_Addr&0xF000)>>12));
	/*设置参数数据*/
	Data[12]=Char_To_Hex((u8)(Param_Data&0x000F));
	Data[11]=Char_To_Hex((u8)((Param_Data&0x00F0)>>4));
	Data[10]=Char_To_Hex((u8)((Param_Data&0x0F00)>>8));
	Data[9]=Char_To_Hex((u8)((Param_Data&0xF000)>>12));
	Data[15]=0x3E;//帧尾
}
void Camera_Write(u8 Data[],u8 n)
{
	u8 i;	
	for(i=0;i<n;i++)
	{
		*(uint32_t*)(CAM_ADDR)=Data[i];
	}
	*(uint32_t*)CAM_Com_En=1;
}
void Restore_Factory_Settings()
{
	Camera_Param_Set(Camera_Restart_Addr,0,CAM_Data);
	Camera_Write(CAM_Data,16);
}
void Save_Configuration_Settings()
{
	Camera_Param_Set(Save_Configuration_Addr,0,CAM_Data);
	Camera_Write(CAM_Data,16);
}

























