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
	ON_WM_PAINT()
	ON_BN_CLICKED(IDC_MFCBUTTON1, &CPicture::OnBnClickedMfcbutton1)
	ON_BN_CLICKED(IDC_MFCBUTTON2, &CPicture::OnBnClickedMfcbutton2)
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
				

}


void CPicture::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	

}


BOOL CPicture::OnInitDialog()
{
	CDialogEx::OnInitDialog();


	// TODO:  �ڴ���Ӷ���ĳ�ʼ��

	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣: OCX ����ҳӦ���� FALSE
}


void CPicture::OnPaint()
{

		//CDialog::OnPaint(); 
		//��Ӵ���Ի��򱳾���ͼ
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //��ȡ�Ի��򳤿�      
		CDC   dcBmp;             //���岢����һ���ڴ��豸����
		dcBmp.CreateCompatibleDC(&dc);             //����������DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP20);    //������Դ��ͼƬ
		BITMAP   m_bitmap;                         //ͼƬ����               
		bmpBackground.GetBitmap(&m_bitmap);       //��ͼƬ����λͼ��
		//��λͼѡ����ʱ�ڴ��豸����
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//���ú�����ʾͼƬStretchBlt��ʾ��״�ɱ�
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
			
	// TODO: �ڴ˴������Ϣ����������
	// ��Ϊ��ͼ��Ϣ���� CDialogEx::OnPaint()
}


void CPicture::OnBnClickedMfcbutton1()
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


void CPicture::OnBnClickedMfcbutton2()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
CDialogEx::OnCancel();
	p_DR->OnBnClickedOk();

}
