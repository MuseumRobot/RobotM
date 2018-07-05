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
	ON_BN_CLICKED(IDOK, &CPicture::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON1, &CPicture::OnBnClickedButton1)
	ON_BN_CLICKED(IDCANCEL, &CPicture::OnBnClickedCancel)
END_MESSAGE_MAP()


// CPicture 消息处理程序


void CPicture::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnOK();
}


void CPicture::OnBnClickedButton1()
{
	// TODO: 在此添加控件通知处理程序代码
				 CBitmap bitmap;  // CBitmap对象，用于加载位图   
    HBITMAP hBmp;    // 保存CBitmap加载的位图的句柄   
	CString str1;
	CString strTemp;
	p_DR->GetDlgItem(IDC_COMBO1)->GetWindowText(str1);
	if(str1==""){
    bitmap.LoadBitmap(IDB_BITMAP1);  // 将位图IDB_BITMAP1加载到bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // 获取bitmap加载位图的句柄   
    m_pic1.SetBitmap(hBmp);    //
	
	}
	if(str1=="黑龙江省博物馆有哪些活动？"){
    bitmap.LoadBitmap(IDB_BITMAP17);  // 将位图IDB_BITMAP1加载到bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // 获取bitmap加载位图的句柄   
    m_pic1.SetBitmap(hBmp);    //
	
	}
		if(str1=="黑龙江省博物馆镇馆之宝有哪些？"){
    bitmap.LoadBitmap(IDB_BITMAP18);  // 将位图IDB_BITMAP1加载到bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // 获取bitmap加载位图的句柄   
    m_pic1.SetBitmap(hBmp);    //
	
	}

}


void CPicture::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();

}


BOOL CPicture::OnInitDialog()
{
	CDialogEx::OnInitDialog();


	// TODO:  在此添加额外的初始化

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}
