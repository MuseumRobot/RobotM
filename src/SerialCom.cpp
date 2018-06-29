// SerialCom.cpp: implementation of the CSerialCom class.
// For Vorager 2.0
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SerialCom.h"
#include <afxwin.h>
#include <conio.h> //���ÿ���̨

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//���ڽ����߳�
UINT ComMonitor(LPVOID pParam)
{
	OVERLAPPED m_os={0};
	m_os.hEvent=NULL;//ָ��һ��I/O������ɺ󴥷����¼�
	CSerialCom * pCom=(CSerialCom *)pParam;
	DWORD dwMaskFlag;
	DWORD dwCommEvent;
	DWORD dwByteToRead;
	DWORD dwRes;
	BYTE rechar[1]={0xff};//���ڱ���������ݵ�һ��������
	bool bWaitingOnStatusHandle = false;

	dwMaskFlag=EV_RXCHAR | EV_CTS;//׼�����ӵĴ����¼�����
	if (!SetCommMask(pCom->m_hCom,dwMaskFlag))//����Ҫͨ���¼�������,ʧ���򷵻�
	{
		return 0;
	}
	m_os.hEvent=CreateEvent(NULL,true,false,NULL);//�ú�������һ��Eventͬ������,�����ظö����Handle   
	if (m_os.hEvent==NULL)//����ʧ���򷵻�0
		return 0;

	while (true)
	{
		//Ϊһ����ָ��ͨ���豸�ȴ�һ���¼�����,��������ɹ������ط���ֵ�����򷵻�0.
		if (!WaitCommEvent(pCom->m_hCom,&dwCommEvent,&m_os))  

		{
			if (GetLastError() == ERROR_IO_PENDING)//����õ��Ĵ�����Ϣ�����������Ϊ���ڵȴ�״̬
			{
				bWaitingOnStatusHandle = true;
			}
            else
				break;
		}
		else
		{
			bWaitingOnStatusHandle=false;
		}
		if (bWaitingOnStatusHandle)
		{
			dwRes = WaitForSingleObject(m_os.hEvent,/*2000*/ INFINITE);//�����߳�Ը����Զ�ȴ���ȥ��������ʱ�䣩
			GetCommMask(pCom->m_hCom,&dwCommEvent);
			if ((dwCommEvent & EV_RXCHAR)==EV_RXCHAR)
			{
				do
				{
					COMSTAT comstat;
					DWORD dwError = 0;
					//�˺������Ӳ����ͨѶ�����Լ���ȡͨѶ�豸�ĵ�ǰ״̬���õ�comstat��ֵ
					ClearCommError(pCom->m_hCom,&dwError,&comstat);//��鴮�ڽ��ջ������е����ݸ���
					if (comstat.cbInQue==0)//�������е�ǰ���е������ֽڸ���Ϊ0
						break;
					ReadFile(pCom->m_hCom,rechar,1,&dwByteToRead,&m_os);//�Ӵ���m_hCom�ж�ȡ���ݵ�������rechar��
					if (pCom!=NULL)
						pCom->Parse(*(BYTE*)rechar);//���ղ��Ҵ洢rechar����
				}
				while (true);
			}
			if (dwRes==WAIT_OBJECT_0)
			{
				//�ú��������ص������Ľ���������ж��첽�����Ƿ���ɣ�����ͨ��m_os�ṹ�е�hEvent�Ƿ���λ���ж�
				if (!GetOverlappedResult(pCom->m_hCom, &m_os, &dwByteToRead, false))
				{
					do
					{
						COMSTAT comstat;
						DWORD dwError = 0;
						
						ClearCommError(pCom->m_hCom,&dwError,&comstat);
						if (comstat.cbInQue==0)
							break;
						ReadFile(pCom->m_hCom,rechar,1,&dwByteToRead,&m_os);
						if (dwByteToRead>0)
						{
							if (pCom=NULL)
								pCom->Parse(*(BYTE*)rechar);
						}
						else
						{
							CloseHandle(m_os.hEvent);
							return 0;
						}

					}
					while (true);
				}
			}
		}
	}
	CloseHandle(m_os.hEvent);
	return 0;
}

//���ڷ����߳�
UINT SendingThread(LPVOID pParam)
{
	CSerialCom * pCom = (CSerialCom *) pParam;
	POSITION pos;

	while (pCom->IsRuning)
	{
		pos = pCom->m_cmdlist.GetHeadPosition();
		if (pos != NULL)
		{
			CMDBUF * pCbuf = (CMDBUF *)pCom->m_cmdlist.GetNext(pos);

			//������ڷ��ͣ���ȴ�
			while (pCom->bSending == TRUE)
			{
				;
			}

			//���ͻ����б����ָ��
			pCom->ComSend(pCbuf->pCmdBuf,pCbuf->nLen);

			//�ͷŻ����б����ѷ���ָ��ռ�õ���Դ
			pCom->m_cmdlist.RemoveHead();
			delete []pCbuf->pCmdBuf;
			delete pCbuf;	
		}
		Sleep(5);
	}
	return 0;
}



CSerialCom::CSerialCom() //���캯��
{
	//Ĭ�ϴ�������
	//m_baudrate=CBR_19200;
	m_baudrate=CBR_115200;
	m_bytesize=8;
	m_stopbits=ONESTOPBIT;
	m_parity=NOPARITY;
	
	bSending = FALSE;
	IsRuning = FALSE;

	//pDispList = NULL;
	//pMoveList = NULL;
}

CSerialCom::~CSerialCom() //��������
{
	IsRuning = FALSE;  

	//��������б��������
	POSITION pos = m_cmdlist.GetHeadPosition();
	while (pos != NULL)
	{
		m_pTempCmd = (CMDBUF *)m_cmdlist.GetNext(pos);
		delete []m_pTempCmd->pCmdBuf;
		delete m_pTempCmd;
	}
	m_cmdlist.RemoveAll();

	Sleep(50);
}

BOOL CSerialCom::Create(int inCom) //���ݴ��ں�(inCom)��ָ������
{
	
	if (TRUE == IsRuning)
	{
		if (inCom == m_com)
		{	//�ظ���ͬһ����
			return FALSE;
		} 
		else
		{	//δ�ر�ԭ���Ĵ���
			CloseHandle(m_hCom);
		}		
	}
	m_com = inCom;

	//��������
	//����CString �����൱���ַ����飬MFC�а�����װ��һ������
	CString strCom;
	strCom.Format("COM%d",inCom); //����һ�����ں��ַ���
	
	//�ô��ں��ַ����򿪴��ڲ����ش򿪵Ĵ��ھ��
	m_hCom=CreateFile(strCom,GENERIC_READ | GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_FLAG_OVERLAPPED,NULL);

	if (m_hCom==INVALID_HANDLE_VALUE)		//�򿪴���ʧ�ܵĴ���
	{
	
		m_hCom=NULL; //m_hCom��NULL,�Ժ���Ը���m_hCom�Ƿ����NULL�жϴ����Ƿ��
		return false; 
	}

	DCB dcb;			          //�豸���ö��������ǶԴ��ڵ�һ������

	COMMTIMEOUTS m_CommTimeouts;      //�涨��/д�����ĳ�ʱʱ��,��λms
	m_CommTimeouts.ReadIntervalTimeout = 1000;           // �������ʱ 
	m_CommTimeouts.ReadTotalTimeoutMultiplier = 1000;    // ��ʱ��ϵ�� 
	m_CommTimeouts.ReadTotalTimeoutConstant = 1000;      // ��ʱ�䳣��
	m_CommTimeouts.WriteTotalTimeoutMultiplier = 1000;   // дʱ��ϵ�� 
	m_CommTimeouts.WriteTotalTimeoutConstant = 1000;     // дʱ�䳣�� 


    //SetCommTimeouts windowsϵͳ���ô˺����趨ͨѶ�豸��дʱ�ĳ�ʱ����
	if (!SetCommTimeouts(m_hCom, &m_CommTimeouts))
	{
		//SetCommTimeouts��������ʧ�ܽ���
		CloseHandle(m_hCom); //�ر��Ѵ򿪴���
		m_hCom=NULL;
		return false; //return 0;
	}

	GetCommState(m_hCom,&dcb);			//�����ö������õ��򿪵Ĵ���
	dcb.BaudRate=m_baudrate;			//���ô��ڲ�����
	dcb.ByteSize=m_bytesize;			//���ô�����ֽ���
	dcb.StopBits=m_stopbits;			//����ֹͣλ
	dcb.Parity=m_parity;				//����У�鷽ʽ
	
	//�����ö������õ��򿪵Ĵ��ڣ������ʧ�ܣ��������رմ���
	if (!SetCommState(m_hCom,&dcb))		
	{
		CloseHandle(m_hCom);
		m_hCom=NULL;
		return false;
	}

	IsRuning = TRUE;

	CString info;
	info.Format("COM %d �򿪳ɹ���",inCom);
//	m_PrintToList(info);

	//�������ڽ����߳�
	AfxBeginThread(ComMonitor,(LPVOID)this,THREAD_PRIORITY_NORMAL);
	
	//�������ڷ����߳�
	AfxBeginThread(SendingThread,(LPVOID)this,THREAD_PRIORITY_NORMAL);

	


	return true;
}

void CSerialCom::Send(const void *pBuffer, const int iLength) //ͨ�������б����ָ��ͣ���׼�ӿڣ�
{
	m_pTempCmd = new CMDBUF;
	m_pTempCmd->pCmdBuf = new UCHAR[iLength];
	memcpy(m_pTempCmd->pCmdBuf,pBuffer,iLength); //�����ַ���
	m_pTempCmd->nLen = iLength;

	m_cmdlist.AddTail(m_pTempCmd); //������β��������m_pTempCmd���ݣ��������ݸ�����1�������µ�����βλ��

}
int num;
void CSerialCom::ComSend(const void *pBuffer, const int iLength) //ֱ�ӷ���ָ��
{
	if (m_hCom==NULL)   //����û�д��򷵻�
		return;

	bSending = TRUE;    //��������λ��TRUE,����1

	OVERLAPPED m_os={0};
	DWORD dwByteWrite;  //DWORD����unsigned long���Ѿ�д����ֽ���
	//_cprintf("send\n"); //�����̨д��
	num++;
	if (!WriteFile(m_hCom,pBuffer,iLength,&dwByteWrite,&m_os)) //�ӻ������򴮿�д����
	{
		DWORD iErr=GetLastError();
	}
	Sleep(10);						//�������100ms

	DWORD dwNumberOfBytesTransferred;//�������ɴ����ֽ�������һ������ 
//	_cprintf("send %d\n",num); 
	GetOverlappedResult(m_hCom,&m_os,&dwNumberOfBytesTransferred,TRUE);//�ȴ�д���ݽ�����ͨ��m_os�ж�
	//_cprintf("send ok\n");
	bSending = FALSE;
}

void CSerialCom::Close() //�رմ��ں���
{
	//�رմ���
	if (m_hCom!=NULL)  //�����Ϊ����˵�����˴��ڣ�ΪNULL˵��û�д򿪴��ڲ���ִ�����溯��
		CloseHandle(m_hCom); //�ر�m_hCom�������
	
	IsRuning = FALSE; 
	m_hCom=NULL;  //�رպ�NULLֵ
}

void CSerialCom::SetBaudRate(int inBaud) //���ò�����
{
	m_baudrate = inBaud;
}


//void CSerialCom::m_PrintToList(CString inStr) //�б��pDispList,����������������ʾ�ַ���
//{
//	if (pDispList != NULL) //���б�����ʱ
//	{
//		pDispList->AddString(inStr);   //���һ���ַ������б����
//		//GetCount���������ַ����������������ַ���
//		pDispList->SetCurSel(pDispList->GetCount()-1); ////ʹ��ӵ��ַ����ɼ�������������Ͽ���б����ѡ��һ���ַ�������Ҫʱ�б����������ʹ���ַ������б�Ŀ������� 
//	}
//}

//void CSerialCom::m_PrintToMoveList(CString inStr) //�б��pMoveList,����������������ʾ�ַ���
//{
//	if (pMoveList != NULL)
//	{
//		pMoveList->AddString(inStr);  //���һ���ַ������б����
//		pMoveList->SetCurSel(pMoveList->GetCount()-1);  //ʹ��ӵ��ַ����ɼ�
//		//pMoveList->GetCount()��ȡ�б���������Ŀ��
//		//SetCurSel()��������Ͽ���б����ѡ��һ���ַ�����
//        //��Ҫʱ�б����������ʹ���ַ������б�Ŀ������� ���б��ǿɼ���ʱ ����
//
//	}
//}
