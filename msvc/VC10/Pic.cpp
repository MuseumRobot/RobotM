// Pic.cpp : 实现文件
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Pic.h"
#include "afxdialogex.h"


// CPic 对话框

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


// CPic 消息处理程序
