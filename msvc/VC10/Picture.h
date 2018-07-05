#pragma once
#include "afxwin.h"





// CPic 对话框

class CPicture : public CDialogEx
{
	DECLARE_DYNAMIC(CPic)

public:
	CPicture(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPicture();

// 对话框数据
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CStatic m_pic1;
};