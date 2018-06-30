// Voice.cpp : ʵ���ļ�
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
// CVoice �Ի���

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


// CVoice ��Ϣ�������


void CVoice::OnPaint()
{

	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, h_icon1);
	}
	else
	{
		//CDialog::OnPaint(); 
		//��Ӵ���Ի��򱳾���ͼ
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //��ȡ�Ի��򳤿�      
		CDC   dcBmp;             //���岢����һ���ڴ��豸����
		dcBmp.CreateCompatibleDC(&dc);             //����������DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP8);    //������Դ��ͼƬ
		BITMAP   m_bitmap;                         //ͼƬ����               
		bmpBackground.GetBitmap(&m_bitmap);       //��ͼƬ����λͼ��
		//��λͼѡ����ʱ�ڴ��豸����
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//���ú�����ʾͼƬStretchBlt��ʾ��״�ɱ�
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
			}
	// TODO: �ڴ˴������Ϣ����������
	// ��Ϊ��ͼ��Ϣ���� CDialogEx::OnPaint()
}


BOOL CVoice::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	m_mfcbtn1.SetFaceColor(RGB(195,209,216));
	m_btn2.SetFaceColor(RGB(195,209,216));
	m_btn3.SetFaceColor(RGB(195,209,216));
	((CComboBox*)GetDlgItem(IDC_COMBO1))->ResetContent();
	((CComboBox*)GetDlgItem(IDC_COMBO1))->AddString(_T("��������"));
	((CComboBox*)GetDlgItem(IDC_COMBO1))->AddString(_T("�˼��޳�"));
	// TODO:  �ڴ���Ӷ���ĳ�ʼ��

	//////////////////xhy
	p_DR=(CVoice*)this;
	//////////////////

	return TRUE;  // return TRUE unless you set the focus to a control
	// �쳣: OCX ����ҳӦ���� FALSE
}


void CVoice::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnCancel();
}


void CVoice::OnBnClickedMfcbutton3()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	p_CR->OnBnClickedButton12();
}
