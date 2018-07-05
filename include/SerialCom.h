// SerialCom.h: interface for the CSerialCom class.
// For Vorager 2.0
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_SERIALCOM_H__78D38E89_1031_495D_8967_84842B40DF86__INCLUDED_)
#define AFX_SERIALCOM_H__78D38E89_1031_495D_8967_84842B40DF86__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
#include <afx.h>
#include <WinSock2.h>

//����ָ��Ļ����б��Ա
struct CMDBUF 
{
	UCHAR * pCmdBuf;	//ָ������ָ��
	UINT nLen;			//ָ���
};
/***************************************************/
//����ͨ����
/***************************************************/
class CSerialCom 
{
public:
	CSerialCom();             //���캯��                                        
	virtual ~CSerialCom();	  //��������
public:
	//void m_PrintToMoveList(CString inStr);                  //�б����º���
	//void m_PrintToList(CString inStr);                      //�б����º���
	virtual void Parse(BYTE inData){};
	void SetBaudRate(int inBaud);                           //���ò�����
	BOOL Create(int inCom);									//��һ�����ڶ˿�
	void Close();											//�رյ�ǰ�򿪵Ķ˿�
	void Send(const void *pBuffer, const int iLength);		//ͨ�������б����ָ��ͣ���׼�ӿڣ�
	void ComSend(const void *pBuffer, const int iLength);	//ֱ�ӷ���ָ��


	CPtrList m_cmdlist;										//����ָ��Ļ����б�
	CMDBUF * m_pTempCmd;									//����ָ����Ա��ʱָ��
	HANDLE m_hCom;											//�Ѵ򿪵Ĵ��ھ��
	BOOL bSending;											//���ͱ��
	BOOL IsRuning;                                          //�жϴ����Ƿ�򿪱�־

	//show //CListBox���ṩWindows�б��Ĺ��ܡ�
	//CListBox* pDispList; 
//	CListBox * pMoveList;


protected:
	//��������
	int m_com;			//���ں�									
	int m_baudrate;     //������
	BYTE m_bytesize;   // �ֽ���
	BYTE m_parity;   //У�鷽ʽ
	BYTE m_stopbits;  //ֹͣλ

private:
	
};

#endif // !defined(AFX_SERIALCOM_H__78D38E89_1031_495D_8967_84842B40DF86__INCLUDED_)
