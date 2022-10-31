


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

void self_check();                 //�Լ�
void parameter_record();           //����װ��
void parameter_record_error();     //����װ������
void state_inquiry();              //״̬��ѯ
void start_point_to();             //����ָ��
void start_point_to_error();       //����ָ�����
void start_step_scan();            //��������ɨ��
void start_step_scan_error();      //��������ɨ�����
void start_track();                //��������
void start_track_error();          //�������ٴ���
void start_constantspeed_scan();   //��������ɨ��
void start_constantspeed_scan_error();  //��������ɨ�����
void make_zero();                  //����
void make_zero_error();            //�������
void scram_button();               //��ͣ
void driver_setting();             //��������
void driver_set_error();           //�������ô���
void navigation_set();             //�ߵ�����
void navigation_set_error();       //�ߵ����ô���
void collecte_command();           //�ɼ�����
void collecte_command_error();     //�ɼ��������
void check_the_time();             //��ʱ
void communication_test();         //ͨ�Ų���
void communication_test_error();   //ͨ�Ų��Դ���
void switch_default();             //switch����

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