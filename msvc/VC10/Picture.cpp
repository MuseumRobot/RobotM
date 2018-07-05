// Picture.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "Picture.h"
#include "Voice.h"
#include "afxdialogex.h"
CPicture *p_ER;
extern CVoice *p_DR;

// CPicture �Ի���

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


// CPicture ��Ϣ�������
void CPicture::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();
	
	
}
void CPic::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	
	
}

void CPic::OnBnClickedButton1()
{
	 CBitmap bitmap;  // CBitmap�������ڼ���λͼ   
    HBITMAP hBmp;    // ����CBitmap���ص�λͼ�ľ��   
    bitmap.LoadBitmap(IDB_BITMAP1);  // ��λͼIDB_BITMAP1���ص�bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // ��ȡbitmap����λͼ�ľ��   
    m_pic1.SetBitmap(hBmp);    //
}
