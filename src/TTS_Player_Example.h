
// TTSPlayer_Example.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CTTSPlayer_ExampleApp:
// �йش����ʵ�֣������ TTSPlayer_Example.cpp
//

class CTTSPlayer_ExampleApp : public CWinApp
{
public:
	CTTSPlayer_ExampleApp();

// ��д
	public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CTTSPlayer_ExampleApp theApp;