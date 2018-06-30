// Voice.cpp : 实现文件
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Voice.h"
#include "afxdialogex.h"
#include "ASR_Recorder_ExampleDlg.h"


//////////////////xhy
extern CRecorder_ExampleDlg *p_CR;
CVoice *p_DR;
////////////////
// CVoice 对话框

IMPLEMENT_DYNAMIC(CVoice, CDialogEx)

CVoice::CVoice(CWnd* pParent /*=NULL*/)
	: CDialogEx(CVoice::IDD, pParent)
{

	
}

CVoice::~CVoice()
{
}

void CVoice::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);


	//  DDX_Control(pDX, IDCANCEL, m_btnCancel);
	DDX_Control(pDX, IDC_MFCBUTTON1, m_mfcbtn1);
	DDX_Control(pDX, IDC_MFCBUTTON2, m_btn2);

	DDX_Control(pDX, IDC_MFCBUTTON3, m_btn3);
//	DDX_Control(pDX, IDC_EDIT1, m_edit1);
}


BEGIN_MESSAGE_MAP(CVoice, CDialogEx)
	ON_WM_PAINT()
	ON_BN_CLICKED(IDCANCEL, &CVoice::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_MFCBUTTON3, &CVoice::OnBnClickedMfcbutton3)
	ON_BN_CLICKED(IDC_MFCBUTTON2, &CVoice::OnBnClickedMfcbutton2)
	ON_BN_CLICKED(IDC_MFCBUTTON4, &CVoice::OnBnClickedMfcbutton4)
	
	ON_CBN_SELCHANGE(IDC_COMBO1, &CVoice::OnCbnSelchangeCombo1)
	ON_BN_CLICKED(IDC_MFCBUTTON1, &CVoice::OnBnClickedMfcbutton1)
END_MESSAGE_MAP()


// CVoice 消息处理程序


void CVoice::OnPaint()
{

	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, h_icon1);
	}
	else
	{
		//CDialog::OnPaint(); 
		//添加代码对话框背景贴图
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //获取对话框长宽      
		CDC   dcBmp;             //定义并创建一个内存设备环境
		dcBmp.CreateCompatibleDC(&dc);             //创建兼容性DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP8);    //载入资源中图片
		BITMAP   m_bitmap;                         //图片变量               
		bmpBackground.GetBitmap(&m_bitmap);       //将图片载入位图中
		//将位图选入临时内存设备环境
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//调用函数显示图片StretchBlt显示形状可变
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
			}
	// TODO: 在此处添加消息处理程序代码
	// 不为绘图消息调用 CDialogEx::OnPaint()
}


BOOL CVoice::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	m_mfcbtn1.SetFaceColor(RGB(195,209,216));
	m_btn2.SetFaceColor(RGB(195,209,216));
	m_btn3.SetFaceColor(RGB(195,209,216));
	((CComboBox*)GetDlgItem(IDC_COMBO1))->ResetContent();
	((CComboBox*)GetDlgItem(IDC_COMBO1))->AddString(_T("世事难料"));
	((CComboBox*)GetDlgItem(IDC_COMBO1))->AddString(_T("人间无常"));
	// TODO:  在此添加额外的初始化

	//////////////////xhy
	p_DR=(CVoice*)this;
	//////////////////

	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}


void CVoice::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}


void CVoice::OnBnClickedMfcbutton3()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}


void CVoice::OnBnClickedMfcbutton2()
{
	
	if(p_CR!=NULL)
	{
		p_CR->OnBnClickedButton9();
	}
}
void CVoice::OnBnClickedMfcbutton4()
{
	
	if(p_CR!=NULL)
	{
		p_CR->OnBnClickedButton11();
	}
	
}
void CVoice::OnCbnSelchangeCombo1()
{	CString strTemp;
	((CComboBox*)GetDlgItem(IDC_COMBO1))->GetWindowText(strTemp);
}
void CVoice::OnCbnSelchangeCombo2()
{	
}






void CVoice::OnBnClickedMfcbutton1()
{
	// TODO: 在此添加控件通知处理程序代码
	p_CR->OnBnClickedButton12();
}
