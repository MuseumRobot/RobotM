#pragma once


// CPicture �Ի���

class CPicture : public CDialogEx
{
	DECLARE_DYNAMIC(CPicture)

public:
	CPicture(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CPicture();

// �Ի�������
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedCancel();
	CStatic m_pic1;
	virtual BOOL OnInitDialog();
};
