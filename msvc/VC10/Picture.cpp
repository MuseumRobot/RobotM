// Picture.cpp : 实现文件
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Picture.h"
#include "Voice.h"
#include "afxdialogex.h"
CPicture *p_ER;
extern CVoice *p_DR;

// CPicture 对话框

IMPLEMENT_DYNAMIC(CPicture, CDialogEx)

CPicture::CPicture(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPicture::IDD, pParent)
{

}

CPicture::~CPicture()
{
}

void CPicture::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PIC_STATIC, m_pic1);
}


BEGIN_MESSAGE_MAP(CPicture, CDialogEx)
	ON_BN_CLICKED(IDOK, &CPic::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CPic::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON1, &CPic::OnBnClickedButton1)
END_MESSAGE_MAP()


// CPicture 消息处理程序
void CPicture::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();
	
	
}
void CPic::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	
	
}

void CPic::OnBnClickedButton1()
{
	 CBitmap bitmap;  // CBitmap对象，用于加载位图   
    HBITMAP hBmp;    // 保存CBitmap加载的位图的句柄   
    bitmap.LoadBitmap(IDB_BITMAP1);  // 将位图IDB_BITMAP1加载到bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // 获取bitmap加载位图的句柄   
    m_pic1.SetBitmap(hBmp);    //
}
