#pragma once
#include "afxwin.h"


// CVoice �Ի���

class CVoice : public CDialogEx
{
	DECLARE_DYNAMIC(CVoice)

public:
	CVoice(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CVoice();

// �Ի�������
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnPaint();
	HICON h_icon1;
	virtual BOOL OnInitDialog();
//	CMFCButton m_btn1;
//	CMFCButton m_btn2;
//	CButton m_btn3;
//	CButton m_btnCancel;
	afx_msg void OnBnClickedCancel();
	CMFCButton m_mfcbtn1;
	CMFCButton m_btn2;
	CButton m_btnCancel;
	afx_msg void OnBnClickedMfcbutton3();
	CMFCButton m_btn3;
	afx_msg void OnBnClickedMfcbutton2();
	afx_msg void OnEnChangeMfceditbrowse1();
	afx_msg void OnEnChangeEdit1();
	CComboBox m_xiala;
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnBnClickedMfcbutton4();
	afx_msg void OnLbnSelchangeList1();
	afx_msg void OnCbnSelchangeCombo2();
	afx_msg void OnBnClickedMfcbutton1();
	afx_msg void OnBnClickedMfcbutton5();
	afx_msg void OnBnClickedOk();
};
