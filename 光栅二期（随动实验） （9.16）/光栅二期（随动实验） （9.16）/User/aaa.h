


#ifndef __Caaa_H
#define __Caaa_H

void AA_Com_Read();
void AA_Com_Write();
void AA_DATA_(void);
void AA_Data_TX(void);
void AA_Data_CL(void);
void read_JingGenZong_Data();
void JiaoYan_Read_Data();
void DaBao_Send_Data();
void write_JingGenZong_Data();
void AB_TX();
void AB_DATA_RX();
void AB_Com_Read();  

union FloatToArray{
	float f;    
	unsigned char ary[4];
};

union IntToArray{
	int k;    
	unsigned char ary[4];
};

union UintToArray{
	unsigned int k;    
	unsigned char ary[4];
};
///*
union Int16ToArray1{
	signed short k;    
	unsigned char ary[2];
};

void self_check();                 //自检
void parameter_record();           //参数装订
void parameter_record_error();     //参数装订错误
void state_inquiry();              //状态查询
void start_point_to();             //启动指向
void start_point_to_error();       //启动指向错误
void start_step_scan();            //启动步进扫描
void start_step_scan_error();      //启动步进扫描错误
void start_track();                //启动跟踪
void start_track_error();          //启动跟踪错误
void start_constantspeed_scan();   //启动匀速扫描
void start_constantspeed_scan_error();  //启动匀速扫描错误
void make_zero();                  //归零
void make_zero_error();            //归零错误
void scram_button();               //急停
void driver_setting();             //驱动设置
void driver_set_error();           //驱动设置错误
void navigation_set();             //惯导设置
void navigation_set_error();       //惯导设置错误
void collecte_command();           //采集命令
void collecte_command_error();     //采集命令错误
void check_the_time();             //对时
void communication_test();         //通信测试
void communication_test_error();   //通信测试错误
void switch_default();             //switch其它

void scan_FW_degree_judge();
void scan_FY_degree_judge();
void scan_FW_step_judge();
void scan_FY_step_judge();
void scan_speed();
void work_mode_change_error();
void work_mode_change_ack();
void scan_FY_degree_judge_1();
void scan_FW_degree_judge_1();
void ZiJian_YingDa();
void CaiJiMingLing_YingDa();
void ZhuangTaiChaXun_YingDa();
void ZhiXiang_YingDa();
void BuJinSaoMiao_YingDa();
void GenZong_YingDa();
void YunSuSaoMiao_YingDa();
void HuiLing_YingDa();
void JiTing_YingDa();
float AryToFloat(char c1, char c2, char c3, char c4);
//*/
#endif