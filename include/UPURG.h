// UPURG.h: interface for the CUPURG class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_UPURG_H__450564AC_462A_4B7B_8206_579EB9A55BD8__INCLUDED_)
#define AFX_UPURG_H__450564AC_462A_4B7B_8206_579EB9A55BD8__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "SerialCom.h"

extern int m_distVal_temp_temp[1000];
extern int m_nValPoint_temp;//m_distVal�ĳ���


#define URG_RECVBUF_LEN 3072
struct Beam
{
	double left_angle, right_angle;//������ʵ�ʽǶ�
	double width_of_angle;//right_angle-left_angle
	double left_dis,right_dis,min_distance;
	double width;
	int m_Obstacle;//=1ʱ,����Ϊ�ϰ���
	
};
/***                 ***/
class CUPURG : public CSerialCom  //����CUPURG��̳�CSerialCom
{
public:
	void SwitchOff(); //�Ͽ�ָ��
	void SwitchOn();  //����ָ��
	void SCIP20();
	void GetDataByGD(int inStart,int inEnd,int inClusterCnt);
	void Parse(BYTE inData); //�������ݲ��洢
public:
	CUPURG();              //���캯��
	virtual ~CUPURG();     //��������

protected:
	long urg_decode(const char* data, int data_byte); //���ݽ�������֮һ

public:
	int m_DirFlag; 
	int m_nTurnRightFlag; //ת���־λ
	int m_nTurnLeftFlag;  //ת���־λ

	int m_nObstacleOnLeft;//����ϰ���������־
	int m_nObstacleOnRight;//����ϰ������Ҳ��־
	int m_nDangerDir;     //�ж��ϰ���������֮һ
	CEvent wait_laser; //�����߳��жϿ���
	bool key; //���ݴ洢���ƿ���,false--m_distVal_temp_test[0]����;ture--m_distVal_temp_test[1]����

	
	void fileter(int * p);
	


	int *m_distVal_temp;
	double m_distVal_temp_test[2][1000];

	double robotPosition_x;
	double robotPosition_y;

	int     target_angle;  //the angle between robot and target 0-72 ,correspond to m_vfh[72]
	
	int     target_flag;   //���Ŀ��Ƕ�������������ĸ�����
	
	double robotsize;

protected:
	void m_ParseFrame(); //�Խ��յ����ݰ����н���
	//send
	char m_cmdToSend[512];

	//recv
	char* m_recvbuf; //���ݰ�����

	//parse
	char m_ParHeader[3]; //���ݰ��Ŀ�ͷ��ʽ
	char m_ParEnd[2];  //���ݰ�������ʽ
	char m_lastChar;   //�洢��һ�����յ����ַ�
	bool m_bFrameStart; //���ݰ����տ�ʼ��־λ��TRUEΪ�ѿ�ʼ���գ�FLASEΪû�п�ʼ
	int m_nFrameLen;    //�洢���ݰ��ĳ���
	int m_nRecvindex; //����д��λ��m_recvbuf

	char* m_databuf; //���ݴ洢
	int* m_distVal;  //���ݽ�������洢
	int * m_disPreVal;//*****��ʱû�õ�
	int m_nValPoint;//m_distVal�ĳ���

	//display
	CString m_strTmp; //

	CRect m_FrameRect;  //CRect��Ϊһ�����Σ��൱�ڶ���һ������
};

#endif // !defined(AFX_UPURG_H__450564AC_462A_4B7B_8206_579EB9A55BD8__INCLUDED_)
