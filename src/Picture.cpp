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
	ON_BN_CLICKED(IDOK, &CPicture::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BUTTON1, &CPicture::OnBnClickedButton1)
	ON_BN_CLICKED(IDCANCEL, &CPicture::OnBnClickedCancel)
END_MESSAGE_MAP()


// CPicture ��Ϣ�������


void CPicture::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnOK();
}


void CPicture::OnBnClickedButton1()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
				 CBitmap bitmap;  // CBitmap�������ڼ���λͼ   
    HBITMAP hBmp;    // ����CBitmap���ص�λͼ�ľ��   
	CString str1;
	CString strTemp;
	p_DR->GetDlgItem(IDC_COMBO1)->GetWindowText(str1);
	if(str1==""){
    bitmap.LoadBitmap(IDB_BITMAP1);  // ��λͼIDB_BITMAP1���ص�bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // ��ȡbitmap����λͼ�ľ��   
    m_pic1.SetBitmap(hBmp);    //
	
	}
	if(str1=="������ʡ���������Щ���"){
    bitmap.LoadBitmap(IDB_BITMAP17);  // ��λͼIDB_BITMAP1���ص�bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // ��ȡbitmap����λͼ�ľ��   
    m_pic1.SetBitmap(hBmp);    //
	
	}
		if(str1=="������ʡ��������֮������Щ��"){
    bitmap.LoadBitmap(IDB_BITMAP18);  // ��λͼIDB_BITMAP1���ص�bitmap   
    hBmp = (HBITMAP)bitmap.GetSafeHandle();  // ��ȡbitmap����λͼ�ľ��   
    m_pic1.SetBitmap(hBmp);    //
	
	}

}


void CPicture::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();

}


BOOL CPicture::OnInitDialog()
{
	CDialogEx::OnInitDialog();


	// TODO:  �ڴ���Ӷ���ĳ�ʼ��

	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣: OCX ����ҳӦ���� FALSE
}
