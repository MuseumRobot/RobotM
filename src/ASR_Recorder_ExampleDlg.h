#pragma once
#include "resource.h"
#include "common/FileReader.h"
#include "hci_asr_recorder.h"
#include <string>
#include "afxwin.h"
#include "hci_tts_player.h"
using std::string;
typedef enum _tag_AsrRecogType{
    kRecogTypeUnkown = -1,      //δ֪����
    kRecogTypeCloud = 0,        //�ƶ�ʶ��
    kRecogTypeLocal,            //����ʶ��
}AsrRecogType;
typedef enum _tag_AsrRecogMode{
    kRecogModeUnkown = -1,      //δ֪����
    kRecogModeFreetalk = 0,     //����˵
	kRecogModeDialog = 2,        //�Ի�
    kRecogModeGrammar,          //�﷨ʶ��
}AsrRecogMode;
// CRecorder_ExampleDlg �Ի���
class CRecorder_ExampleDlg : public CDialog{
// ����
public:
	CRecorder_ExampleDlg(CWnd* pParent = NULL);	// ��׼���캯��
	~CRecorder_ExampleDlg(){ 
		// ������¼�����Ļص��У�ʹ����Windows���ڣ����������������з���ʼ��
		// �����ڴ�����Чʱ�����з���ʼ��
		//Uninit(); 
	}
// �Ի�������
	enum { IDD = IDD_RECORDER_EXAMPLE_DIALOG };
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��

// ʵ��
protected:
    HICON m_hIcon;
	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg LRESULT OnShowStatus( WPARAM wParam, LPARAM lParam );
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnStartRecord();
	afx_msg void OnBnClickedBtnCancelRecord();
	bool Init();
	bool Uninit(void);
	afx_msg void OnBnClickedOk();
	void CRecorder_ExampleDlg::DrawItem(int nIDCtl,LPDRAWITEMSTRUCT lpDrawItemStruct);
	void  CRecorder_ExampleDlg::ChangeSize(CWnd * pWnd, int cx, int cy);
public:
    void EchoGrammarData(const string &grammarFile);
    static void HCIAPI RecordEventChange(RECORDER_EVENT eRecorderEvent, void *pUsrParam);
    static void HCIAPI RecorderRecogFinish(
        RECORDER_EVENT eRecorderEvent,
        ASR_RECOG_RESULT *psAsrRecogResult,
        void *pUsrParam);
    static void HCIAPI RecorderRecogProcess(
        RECORDER_EVENT eRecorderEvent,
        ASR_RECOG_RESULT *psAsrRecogResult,
        void *pUsrParam);
    static void HCIAPI RecorderErr(
        RECORDER_EVENT eRecorderEvent,
        HCI_ERR_CODE eErrorCode,
        void *pUsrParam);
    static void HCIAPI RecorderRecordingCallback(
        unsigned char * pVoiceData,
        unsigned int uiVoiceLen,
        void * pUsrParam
        );
public:
	void AppendMessage(CString &strMsg);
	void RecorderRecording(unsigned char * pVoiceData, unsigned int uiVoiceLen);
	void PostRecorderEventAndMsg(RECORDER_EVENT eRecorderEvent, const CString & strMessage);
private:
    AsrRecogType m_RecogType;
    AsrRecogMode m_RecogMode;
    unsigned int m_GrammarId;
	BOOL m_recordingFlag;
	FILE * m_recordingFile;
	CString m_recordingFileName;
public:
	afx_msg void OnBnClickedBtnBrowser();
	afx_msg void OnBnClickedSaveRecording();
	clock_t m_startClock;
	afx_msg void OnEnChangeEditStatus();
	afx_msg void OnBnClickedOnlyRecording();
	afx_msg void OnBnClickedContinue();
	afx_msg void OnEnChangeEditSaveRecordingFile();
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
public:
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton4();
	afx_msg void OnBnClickedButton5();
	afx_msg void OnBnClickedButton6();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton8();
	afx_msg void OnEnChangeEditWordlist();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	CMFCButton m_btn1;
	CMFCButton m_btn3;
	CMFCButton m_btn4;
	CStatic m_pic1;
	afx_msg void OnStnClickedStatic1();
	afx_msg void OnBnClickedButton9();
protected:
	CRect m_rect;
public:
	afx_msg void OnBnClickedMfcbutton1();
	afx_msg void OnBnClickedMfcbutton3();
	afx_msg void OnBnClickedMfcbutton5();
	afx_msg void OnBnClickedMfcbutton6();
	afx_msg void OnBnClickedMfcbutton7();
	afx_msg void OnBnClickedMfccolorbutton1();
	CMFCButton m_mfcbtn1;
	CMFCButton m_mfcbtn2;
	CMFCButton m_mfcbtn3;
	CMFCButton m_mfcbtn4;
	CMFCButton m_mfcbtn5;
	CMFCButton m_mfcbtn6;
	afx_msg void OnBnClickedMfccolorbutton4();
	afx_msg void OnBnClickedMfcbutton2();
	afx_msg void OnSizing(UINT fwSide, LPRECT pRect);
	CMFCButton m_mfcbtn7;
	CButton m_btn10;
	CEdit m_edit_status;
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedButton12();
};


