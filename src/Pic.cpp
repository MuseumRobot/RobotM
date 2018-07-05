// ../../src/Pic.cpp : 实现文件
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Pic.h"
#include "afxdialogex.h"
#include "Voice.h"
CPic *p_ER;
extern CVoice *p_DR;

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
	DDX_Control(pDX, IDC_PIC_STATIC, m_pic1);
}


BEGIN_MESSAGE_MAP(CPic, CDialogEx)
	ON_BN_CLICKED(IDOK, &CPic::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CPic::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON1, &CPic::OnBnClickedButton1)
	
END_MESSAGE_MAP()


// CPic 消息处理程序


void CPic::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	
	
}


BOOL CPic::OnInitDialog()
{
	CDialogEx::OnInitDialog();

			
	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


void CPic::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();
	
	
}


void CPic::OnBnClickedButton1()
{
	 CBitmap bitmap;  // CBitmap对象，用于加载位图   
    HBITMAP hBmp;    // 保存CBitmap加载的位图的句柄   
  
    bitmap.LoadBitmap(IDB_BITMAP1);  // 将位图IDB_BITMAP1加载到bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // 获取bitmap加载位图的句柄   
    m_pic1.SetBitmap(hBmp);    //
}





