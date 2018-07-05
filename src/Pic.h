#pragma once
#include "afxwin.h"





// CPic 对话框

class CPic : public CDialogEx
{
	DECLARE_DYNAMIC(CPic)

public:
	CPic(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPic();

// 对话框数据
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CStatic m_pic1;
};
