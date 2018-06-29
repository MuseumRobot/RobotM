#pragma once
#include "stdafx.h"
#include "SerialCom.h"
#include <math.h>

#define Drobot_wheel_spacing 359//�ּ��

struct robotinfo 
{
	double pi;
	int Drobot;
	int zuolunfangxiang;
	int youlunfangxiang;
	int zuolunjuli;
	int youlunjuli;
	double pianzhuan;
	float pointrox;
	float pointroy;
	double pian;
	float pointrox_corrected;
	float pointroy_corrected;
	float pianzhuan_corrected;
	int scene_length;
	int scene_width;
	double pianzhuan_stargazer;
	float pointrox_stargazer;
	float pointroy_stargazer;
	
	//	float Drobot_wheel_spacing;
};

class CMotor:public CSerialCom
{
public:

	volatile int lift_num ;//�������ݴ洢
	volatile int right_num ;//�������ݴ洢
	volatile int zhong_num;//�������ݴ洢
	volatile int motor_timer ;//ʱ�����ݴ洢

	int encoder_l; //���������դ��
	int encoder_r; //�ұ�������դ��
	int encoder_z; //�б�������դ��
	//extern int timer; //ʱ��


	int timer ;  //ʱ���¼
	int timerold ; //��һ��ʱ��
	int timer_dif ; //ʱ���

	int time_hour; //Сʱ
	int time_minute; //����
	int time_second; //��
	int time_msec; //����

	int errornum ;
	int speed_stated; //����
	float speed_l; //����ʵʱ�ٶ�
	float speed_r ; //����ʵʱ�ٶ�
	float speed_z ; //����ʵʱ�ٶ�

public:

	void Parse(BYTE inData);//���ڽ��ղ������߳�
	char m_cmdToSend[512];//���ݻ�����

	//recv
	BYTE m_recvbuf[50]; //���ݰ�����

	//parse
	char m_ParHeader; //���ݰ��Ŀ�ͷ��ʽ
	char m_ParEnd;  //���ݰ�������ʽ
	char ldata_star;
	char rdata_star;
	char zdata_star;
	char Time_start;

	char m_lastChar;   //�洢��һ�����յ����ַ�
	bool m_bFrameStart; //���ݰ����տ�ʼ��־λ��TRUEΪ�ѿ�ʼ���գ�FLASEΪû�п�ʼ
	bool m_right;//���ݰ���ʼ��ȷ
	int m_nFrameLen;    //�洢���ݰ��ĳ���
	int m_nRecvindex; //����д��λ��m_recvbuf

	char* m_databuf; //���ݴ洢
	int* m_distVal;  //���ݽ�������洢
	CMotor(void);
	~CMotor(void);
	char motor_str[20];//
	int Go_status;//ǰ��״̬
	int Go_status_old;//ǰ��״̬
	int move_lsp;
	int move_rsp;
	int move_zsp;

	//ǰ������ Lspeed �����ٶ� Rspeed �����ٶ�  ��λcm/s
	bool gomotor(int Lspeed, int Rspeed, int Zspeed);
	//ֹͣ
	bool stop();
	// ���ȿ��ƺ���
	bool Velocity_control(float linear_velocity, float angular_velocity);//linear_velocity ���ٶ� angular_velocity ���ٶ� ������ͬ����ͬ
	//void VectorMove(double inAngle,float inLV,float inPSpeed); //����ǰ���ƶ����ƣ����������ֱ�������������ϵ��ȫ������ϵ��ˮƽ����ļнǣ��������˶������ٶȣ��������˶��Ľ��ٶ�
	void VectorMove(float inLV, float inPSpeed,float pianzhuan,float Robot_X,float Robot_Y); //����ǰ���ƶ����ƣ����������ֱ�������������ϵ��ȫ������ϵ��ˮƽ����ļнǣ��������˶������ٶȣ��������˶��Ľ��ٶ�
	
	int CMotor::m_CalAngle(int angle1, int angle2);//����Ƕ�

	//�򿪵������
	bool open_com_motor(int CCommport);
	void m_ParseFrame(void);//�������ݽ���

	//������λ�˼���
	void RobotPositionCompute(float distancedif_l,float distancedif_r,float distancedif_z,robotinfo&);
	
};

//struct robotinfo 
//{
//	double pi;
//	int Drobot;
//	int zuolunfangxiang;
//	int youlunfangxiang;
//	int zuolunjuli;
//	int youlunjuli;
//	double pianzhuan;
//	float pointrox;
//	float pointroy;
//	double pian;
//	float pointrox_corrected;
//	float pointroy_corrected;
//	float pianzhuan_corrected;
////	float Drobot_wheel_spacing;
//};
