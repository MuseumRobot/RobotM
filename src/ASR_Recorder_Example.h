
// Recorder_Example.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CRecorder_ExampleApp:
// �йش����ʵ�֣������ Recorder_Example.cpp
//

class CRecorder_ExampleApp : public CWinAppEx
{
public:
	CRecorder_ExampleApp();

// ��д
	public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
//	virtual int ExitInstance();
};

extern CRecorder_ExampleApp theApp;