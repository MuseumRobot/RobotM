#pragma once


// CPicture 对话框

class CPicture : public CDialogEx
{
	DECLARE_DYNAMIC(CPicture)

public:
	CPicture(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPicture();

// 对话框数据
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedCancel();
	CStatic m_pic1;
	virtual BOOL OnInitDialog();
};
