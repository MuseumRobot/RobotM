#pragma once
#include "afxwin.h"





// CPic �Ի���

class CPicture : public CDialogEx
{
	DECLARE_DYNAMIC(CPic)

public:
	CPicture(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CPicture();

// �Ի�������
	enum { IDD = IDD_DIALOG2 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CStatic m_pic1;
};