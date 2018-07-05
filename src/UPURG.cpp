// UPURG.cpp: implementation of the CUPURG class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "UPURG.h"
#include "math.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif



int m_distVal_temp_temp[1000];
int m_nValPoint_temp;//m_distVal�ĳ���
// FILE* URG;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


CUPURG::CUPURG() //���캯��
{
	m_lastChar = 0;
	m_ParHeader[0] = 0x47; //G
	m_ParHeader[1] = 0x44; //D
	m_ParHeader[2] = '\0';

	m_ParEnd[0] = '\n';
	m_ParEnd[1] = '\n';

	m_recvbuf = new char[URG_RECVBUF_LEN]; //new��̬�����ڴ�,���ݰ�����
	m_databuf = new char[URG_RECVBUF_LEN]; //���ݴ洢
	m_distVal = new int[URG_RECVBUF_LEN/3];//���ݽ�������洢
	m_disPreVal = new int[URG_RECVBUF_LEN/3];//

	m_nRecvindex = 0;       //����д��λ��m_recvbuf
	m_nFrameLen = 0;        //�洢���ݰ��ĳ���
	m_bFrameStart = false;  //���ݰ����տ�ʼ��־λ��TRUEΪ�ѿ�ʼ���գ�FLASEΪû�п�ʼ
	m_nValPoint = 0;        //m_distVal�ĳ���

	key = false;

	for (int i = 0;i<1024;i++)  //URG_RECVBUF_LEN/3=1024
	{
		m_disPreVal[i] = 0;
	}
	m_nTurnLeftFlag = 0;
	m_nTurnRightFlag = 0;
	m_DirFlag = 0;

	///////////////////////////////////
	for (int i=0;i<1000;i++)
	{
		m_distVal_temp_temp[i]=100;
	}
	///////////////////////////////////
	
}

CUPURG::~CUPURG() //��������
{
	delete []m_recvbuf; //�ͷ�new����ռ�
	delete []m_databuf;
	/////////////////////////////��������´���
	delete []m_distVal; 
	delete []m_disPreVal;

}

void CUPURG::Parse(BYTE inData) //�������ݲ��洢
{
	if (m_bFrameStart == false) //��û�п�ʼ�������ݣ�����Ҫ����Ƿ������ݰ�ͷ���֣����ݰ����տ�ʼ��־λ��TRUEΪ�ѿ�ʼ���գ�FLASEΪû�п�ʼ
	{
		//�ж��Ƿ�Ϊ��ͷ
		if (inData == m_ParHeader[1] && m_lastChar == m_ParHeader[0])//���ݰ�ͷ������ʼ��������
		{
			m_bFrameStart = true;
			memcpy(m_recvbuf,m_ParHeader,2);//д���ͷ���Ѱ�ͷд�뵽���ݰ��洢
			m_nRecvindex = 1;//����д��λ
			//debug
		//	m_strTmp.Format("���յ� %s ���ݰ�",m_ParHeader);//����һ�����ݰ�ͷ�ַ���
		//	m_PrintToList(m_strTmp);//������ʾ�ַ���
		}
	}
	else//���Ѿ���ʼ��������
	{
		//������
		m_nRecvindex ++;//����д��λ��1
		m_recvbuf[m_nRecvindex] = inData;
		//Ѱ�Ұ�β
		if (inData == m_ParEnd[1] && m_lastChar == m_ParEnd[0])//�����ݺ���һ�����յ����ַ���Ϊ���ݰ�������ʽ
		{
			m_nFrameLen = m_nRecvindex + 1;//�洢���ݰ��ĳ��ȵ�������д��λ��1
			m_ParseFrame();//�Խ��յ����ݰ����н���
			m_bFrameStart = false;
			m_nRecvindex = 0;
		}
	}
	m_lastChar = inData;//����һ�����յ����ַ����ĳ�inData

}

long CUPURG::urg_decode(const char *data, int data_byte) //���ݽ���
{	
	long value = 0;
	for (int i = 0; i < data_byte; ++i)
	{
		value <<= 6;

		value &= ~0x3f;
		value |= data[i] - 0x30;
	}
  return value;
}

void CUPURG::GetDataByGD(int inStart, int inEnd, int inClusterCnt) //�����������յ� GDGS �������ᷴ�����µĲ������ݵ�����
{
	sprintf(m_cmdToSend, "GD%04d%04d%02d\n", inStart, inEnd, inClusterCnt);//�Ѹ�ʽ��������д�뻺����m_cmdToSend
	Send(m_cmdToSend,13);//ͨ�������б����ָ��ͣ���׼�ӿڣ�
}

void CUPURG::SCIP20()     //ʹ������SCIP2.0�л�����ı䴫����ģʽ
{
	sprintf(m_cmdToSend, "SCIP2.0\n");
	Send(m_cmdToSend,8);
}

void CUPURG::SwitchOn()    //��� BM �����ʹ��������ʹ�ܲ���
{
	//BM cmd
	sprintf(m_cmdToSend, "BM\n");
	Send(m_cmdToSend,3);
}

void CUPURG::SwitchOff()    //���QT����رռ��⣬���ò���״̬��
{
	//QT cmd	
	sprintf(m_cmdToSend, "QT\n");
	Send(m_cmdToSend,3);
}

void CUPURG::m_ParseFrame() //�Խ��յ����ݰ����н���
{
// 	m_strTmp.Format("����%d",m_nFrameLen);
// 	m_PrintToList(m_strTmp);
	
	
//	fprintf(out,"gggggg   \n");
	//�������ݿ鳤��
	int nData = m_nFrameLen - 23/*cmd*/ - 1/*���һ��LF*/;
	int nBlock = nData/66;
	int nLeft = nData%66 - 2/*sum+LF*/;

	char* pDataBuf = m_recvbuf+23; //��ȡ����ָ�룬����23���̶���cmd�ַ�
//	URG = fopen(".\\urg.txt","w+");
	//data block
	for (int i=0;i<nBlock;i++)
	{
		memcpy((m_databuf+i*64),pDataBuf,64); //�����ݴ洢���Ƶ������б���
		pDataBuf += 66;
	}

	//left
	if (nLeft>0)
	{
		memcpy((m_databuf+nBlock*64),pDataBuf,nLeft);
	}

	//����ɾ���ֵ //���ݽ���
	m_nValPoint = (nBlock*64+nLeft)/3;
	pDataBuf = m_databuf;
	CString str;
	//for (int i = 0; i <1000;i++)
	m_distVal[0] = 100; 
	
	m_nValPoint_temp=m_nValPoint;
//fprintf(out,"gggggg  %d  %d   \n",m_nValPoint_temp,m_nValPoint);

	for (int i=1;i<m_nValPoint;i++)
	{
		int temp;
		int j = 0;
		temp = urg_decode(pDataBuf,3); //���ݽ���
	
		m_distVal[i] = temp;//������������õ�ÿ���Ƕȵľ���

		pDataBuf += 3;
	}

//	fileter(m_distVal);//���˻�ȡ������
	for (int i=0;i<m_nValPoint;i++)
	{
	/*	if (m_distVal[i]<50)
		{
			m_distVal[i] = 10000;
		}*/

				for (int i=0;i<100;i++)
			{
				m_distVal[i]=100;//////////////////////////100mm����ʾ�������������ϰ���
			}
			for (int i=666;i<769;i++)
			{
				m_distVal[i]=100;
			}

			for (int i=100;i<666;i++)
			{
				if (m_distVal[i]<50)
				{
					m_distVal[i]=10000;/////////////////////////////����������û�в������ϰ���趨�䷵�ؽ��Ϊ10000mm
				}
	
			}
		m_distVal_temp_temp[i]=m_distVal[i];
		m_distVal_temp_test[key][i]=(double)m_distVal[i];
		
	}
	key = !key;

	wait_laser.SetEvent();

}



void CUPURG::fileter(int * p)
{
	for (int i=0;i<100;i++)
	{
		p[i]=100;//////////////////////////100mm����ʾ�������������ϰ���
	}
	for (int i=666;i<769;i++)
	{
		p[i]=100;
	}

	for (int i=100;i<666;i++)
	{
		if (p[i]<50)
		{
			p[i]=10000;/////////////////////////////����������û�в������ϰ���趨�䷵�ؽ��Ϊ10000mm
		}
	
	}

}