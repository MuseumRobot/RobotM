// Pic.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Pic.h"
#include "afxdialogex.h"


// CPic �Ի���

IMPLEMENT_DYNAMIC(CPic, CDialogEx)

CPic::CPic(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPic::IDD, pParent)
{

}

CPic::~CPic()
{
}

void CPic::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CPic, CDialogEx)
END_MESSAGE_MAP()


// CPic ��Ϣ�������
