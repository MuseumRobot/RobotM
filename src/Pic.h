#pragma once
#include "afxwin.h"


// CPic �Ի���

class CPic : public CDialogEx
{
	DECLARE_DYNAMIC(CPic)

public:
	CPic(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CPic();

// �Ի�������
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	virtual BOOL OnInitDialog();
	
	CStatic m_pic1;
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedButton1();

};
