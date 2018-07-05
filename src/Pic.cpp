// ../../src/Pic.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Pic.h"
#include "afxdialogex.h"
#include "Voice.h"
CPic *p_ER;
extern CVoice *p_DR;

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
	DDX_Control(pDX, IDC_PIC_STATIC, m_pic1);
}


BEGIN_MESSAGE_MAP(CPic, CDialogEx)
	ON_BN_CLICKED(IDOK, &CPic::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CPic::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON1, &CPic::OnBnClickedButton1)
	
END_MESSAGE_MAP()


// CPic ��Ϣ�������


void CPic::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	
	
}


BOOL CPic::OnInitDialog()
{
	CDialogEx::OnInitDialog();

			
	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣: OCX ����ҳӦ���� FALSE
}


void CPic::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();
	
	
}


void CPic::OnBnClickedButton1()
{
	 CBitmap bitmap;  // CBitmap�������ڼ���λͼ   
    HBITMAP hBmp;    // ����CBitmap���ص�λͼ�ľ��   
  
    bitmap.LoadBitmap(IDB_BITMAP1);  // ��λͼIDB_BITMAP1���ص�bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // ��ȡbitmap����λͼ�ľ��   
    m_pic1.SetBitmap(hBmp);    //
}





