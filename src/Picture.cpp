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
	ON_WM_PAINT()
	ON_BN_CLICKED(IDC_MFCBUTTON1, &CPicture::OnBnClickedMfcbutton1)
	ON_BN_CLICKED(IDC_MFCBUTTON2, &CPicture::OnBnClickedMfcbutton2)
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
				

}


void CPicture::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	

}


BOOL CPicture::OnInitDialog()
{
	CDialogEx::OnInitDialog();


	// TODO:  在此添加额外的初始化

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


void CPicture::OnPaint()
{

		//CDialog::OnPaint(); 
		//添加代码对话框背景贴图
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //获取对话框长宽      
		CDC   dcBmp;             //定义并创建一个内存设备环境
		dcBmp.CreateCompatibleDC(&dc);             //创建兼容性DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP20);    //载入资源中图片
		BITMAP   m_bitmap;                         //图片变量               
		bmpBackground.GetBitmap(&m_bitmap);       //将图片载入位图中
		//将位图选入临时内存设备环境
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//调用函数显示图片StretchBlt显示形状可变
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
			
	// TODO: 在此处添加消息处理程序代码
	// 不为绘图消息调用 CDialogEx::OnPaint()
}


void CPicture::OnBnClickedMfcbutton1()
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


void CPicture::OnBnClickedMfcbutton2()
{
	// TODO: 在此添加控件通知处理程序代码
CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();

}
