
// TTSPlayer_ExampleDlg.h : 头文件
//

#pragma once

#include <string>
#include "hci_tts_player.h"
using std::string;

// CTTSPlayer_ExampleDlg 对话框
class CTTSPlayer_ExampleDlg : public CDialog
{
// 构造
public:
	CTTSPlayer_ExampleDlg(CWnd* pParent = NULL);	// 标准构造函数
	~CTTSPlayer_ExampleDlg(){ Uninit(); }

// 对话框数据
	enum { IDD = IDD_TTSPLAYER_EXAMPLE_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg LRESULT OnShowStatus( WPARAM wParam, LPARAM lParam );
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnPlay();
	afx_msg void OnBnClickedBtnPause();
	afx_msg void OnBnClickedBtnStop();
    void ShowBtnStatus( BOOL bStartPlay, BOOL bPause, BOOL bStop );
	bool Init(void);
	bool Uninit(void);

protected:
    static void HCIAPI CB_EventChange(
        _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
        _OPT_ _IN_ void * pUsrParam);

    static void HCIAPI CB_ProgressChange (
        _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
        _MUST_ _IN_ int nStart,
        _MUST_ _IN_ int nStop,
        _OPT_ _IN_ void * pUsrParam);

    static void HCIAPI CB_SdkErr( _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
        _MUST_ _IN_ HCI_ERR_CODE eErrorCode,
        _OPT_ _IN_ void * pUsrParam );
};
