


// Recorder_ExampleDlg.cpp : ʵ���ļ�
//
#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "ASR_Recorder_ExampleDlg.h"
#include "hci_asr_recorder.h"
#include "common/CommonTool.h"
#include "common/AccountInfo.h"
#include "hci_tts.h"
#include "hci_tts_player.h"
#include "hci_micarray.h"
//xhyvhf
#include "UPURG.h" //����
#include "Plan_Path_VFH.h"
#include "vfh_algorithm.h"
#include "Voice.h"
//move
#include "Comm_data_motor3.h"

//starxhy
#include "Comm_data_star.h" //�����ǩ��λģ��
#pragma comment(lib,"Comm_data_star.lib") //�Ӿ���λ

#include "cJSON.h"
#include <string>
using std::string;

#include "mmsystem.h"  
#pragma comment(lib,"Winmm.lib")  

//movexhy
#define COMM_MOTOR 3 //�ײ�������ں�
CMotor motor;
CEvent wait_motordata;
CEvent wait_motortts;
UINT ThreadComput_MotorData(LPVOID lpParam);
UINT ThreadComput_MotorTts(LPVOID lpParam);
CWinThread* pThread_Motor_Comput;////������ݽ����߳�
CWinThread* pThread_Motor_tts;//���tts�߳�
CFont *m_pFont;//�����µ�����  
 
CWinThread* pThread_Motor_autowalk;//��������߳�
bool moterautowalk_key= true;//��ʼ����
UINT ThreadComput_Motorautowalk(LPVOID lpParam);

float distance_l = 0.0; //���־���
float distanceold_l = 0.0; //������һ�ξ���
float distance_r = 0.0; //���־���
float distanceold_r = 0.0; //������һ�ξ���
float distance_z = 0.0; //���־���
float distanceold_z = 0.0; //������һ�ξ���
float distancedif_l = 0.0; //���ֲ�ֵ
float distancedif_r = 0.0; //���ֲ�ֵ
float distancedif_z = 0.0; //

bool moter_key = true;//������ݽ����߳̿���
bool motertts_key= true;
bool motertts_key1=true;//tts�̲߳���1
bool motertts_key2= true;//tts�̲߳���2
bool motertts_key3=true;//tts�̲߳���1
bool motertts_key4= true;//tts�̲߳���2

  struct robotinfo Info_robot = 
{
	Info_robot.pi=3.141592654,
	Info_robot.Drobot = 400,
	Info_robot.zuolunfangxiang = 0,
	Info_robot.youlunfangxiang = 0,
	Info_robot.zuolunjuli = 0,
	Info_robot.youlunjuli = 0,
	Info_robot.pianzhuan = 0,//Info_robot.pi/2,
	//#define Start_Pose_x 1388
	//#define Start_Pose_y 1787
	Info_robot.pointrox = 10000,
	Info_robot.pointroy = 10000,
	Info_robot.pian = 0.00001,
	Info_robot.scene_length=20000,
	Info_robot.scene_width=20000,
	Info_robot.pianzhuan_stargazer=0,
	Info_robot.pointrox_stargazer=0,
	Info_robot.pointroy_stargazer=0

};
//starxhy
#define COMM_STAR 4//�Ǳ궨λ����
Cstar StarGazer;  //�Ӿ���λ������

struct StarMark
{
	int markID;
	float mark_angle;
	float mark_x;
	float mark_y;
};

struct StarMark MARK[100]={ //LED��λ��ǩ����
	//ÿ��߳�455
	MARK[0].markID = 624,
	MARK[0].mark_angle = 0.00,
	MARK[0].mark_x = -427,
	MARK[0].mark_y = 10,


	MARK[1].markID = 610,
	MARK[1].mark_angle = 0.00,
	MARK[1].mark_x = -344,
	MARK[1].mark_y = 132,

	MARK[2].markID = 594, 
	MARK[2].mark_angle = 0.00,
	MARK[2].mark_x = -212,
	MARK[2].mark_y = 0,

	MARK[3].markID = 608,
	MARK[3].mark_angle = 0.00,
	MARK[3].mark_x = -20,
	MARK[3].mark_y = 30,

	MARK[4].markID = 626,
	MARK[4].mark_angle = 0.00,
	MARK[4].mark_x = -195,
	MARK[4].mark_y = 179,

	MARK[5].markID = 592,
	MARK[5].mark_angle = 0.00,
	MARK[5].mark_x = -6,
	MARK[5].mark_y = 177,


	MARK[6].markID = 560,
	MARK[6].mark_angle = 0.00,
	MARK[6].mark_x = 160,
	MARK[6].mark_y = 12,

	MARK[7].markID = 546,
	MARK[7].mark_angle = 0.00,
	MARK[7].mark_x = 150,
	MARK[7].mark_y = 183,

	MARK[8].markID = 578,
	MARK[8].mark_angle = 0.00,
	MARK[8].mark_x = 140,
	MARK[8].mark_y = 377,

	MARK[9].markID = 544,
	MARK[9].mark_angle = 0.00,
	MARK[9].mark_x = 309,
	MARK[9].mark_y = 358,

	MARK[10].markID = 576,
	MARK[10].mark_angle = 0.00,
	MARK[10].mark_x = 138,
	MARK[10].mark_y = 540,

	MARK[11].markID = 16,
	MARK[11].mark_angle = 0.00,
	MARK[11].mark_x = 308,
	MARK[11].mark_y = 168,

	MARK[12].markID = 562,
	MARK[12].mark_angle = 0.00,
	MARK[12].mark_x = 311,
	MARK[12].mark_y = 528,

	MARK[13].markID = 530,
	MARK[13].mark_angle = 0.00,
	MARK[13].mark_x = 309,
	MARK[13].mark_y = 1,
};

//xhyvhf
#define Max_Laser_Data_Point 768
struct threadInfo_laser_data
{
	double 	m_Laser_Data_Value[Max_Laser_Data_Point];
	int	m_Laser_Data_Point;
};
//�����������ݶ�ȡ�ṹ����
struct threadInfo_laser_data_postpro
{
	double 	m_Laser_Data_Value_PostPro[Max_Laser_Data_Point];
	int	m_Laser_Data_Point_PostPro;
	double rx;
	double ry;   //robot odometry pos
	double  th;   //robot orientation 

	double  x[10][769];//[cm]
	double  y[10][769];//[cm]
	int bad[10][769];// 0 if OK
	//sources of invalidity - too big range;
	//moving object; occluded;mixed pixel
	int seg[10][769];

};

float m_nDlgWidth,m_nDlgHeight,m_nWidth,m_nHeight,m_Multiple_width,m_Mutiple_heith;
		bool change_flag;

#define COMM_LASER 5 //���⴫�������ں�
CUPURG m_cURG; //���⴮�ڿ�����
int m_laser_data_raw[1000];
int m_laser_data_postpro[1000];
int m_Laser_Data_Point_PostPro;
bool lase_key = true;
bool key_laser = false; //�������ݴ洢λ�ÿ���
CEvent wait_data;
CEvent wait_laserpose;

CEvent wait_vfh;
CEvent wait_motor;

CEvent wait_gridfastslam;

extern int speed_stated=20; //�������ٶ�
extern int waittimer=0;
extern int speed_stated_vfh;

extern double vfh_Scene_scale_x;
extern double vfh_Scene_scale_y;

extern double Obstacle_Distance_init;/////////////////////////��ʼ���Ͼ����趨
extern double delt_Obstacle_Distance_init;
extern double free_Obstacle_Distance_init;
extern int m_laser_data_postpro_vfh[1000]; //vfh�м�������

CWinThread* pThread_DataExchange;  //vfh���ݽ����߳�
CWinThread* pThread_VFHStart; //vfh����

CWinThread* pThread_Read_Laser;  //�������ݶ��߳�
threadInfo_laser_data Info_laser_data;

CWinThread* pReadThread_Laser_Pose_Compute; //����λ�˼����߳�
UINT ThreadReadLaser_Data(LPVOID lpParam);

UINT ThreadDataExchange(LPVOID lpParam); //���ݽ����߳�
UINT ThreaVFH(LPVOID lpParam);

CPlan_Path_VFH plan;
VFH_Algorithm algorithm;

//extern double vfh_Scene_scale_x;
//extern double vfh_Scene_scale_y;

bool dataexchange_key = false;
bool vfh_key = false;
bool motor_key = false;
LARGE_INTEGER freq1;
LARGE_INTEGER start_t1, stop_t1;  
double exe_time1;  

//#pragma comment(lib,"Plan_Path_VFH.lib") //����·���滮��

void Pathplan();

bool gridfastslam_key = true;

int wait_motor_timer = 200;
bool speedkey = 0;
CWinThread* pThread_MototCtrl;
int speed_l_old = 0;
int speed_r_old = 0;
int CtrlMode_old;
int run_direction;
int run_direction_old;
int acceleration = 3;

UINT ThreaMotorCtrl(LPVOID lpParam);
void SpeedBuffer(int speed_l, int speed_r,int *speed_l_old, int *speed_r_old);
extern float pick;
/////////////////////////xhyobj
struct object
{
	int objectnum;
	float objectnum_x;
	float objectnum_y;
	float direct;
	float time;
	int mode;//1:����ת��(չƷ��)2:���ﲥ������(������)3:��������ת��(������)

	char* contect1;
	char* contect2;
	char* contect3;
};

struct object zhanpin[100]={
	zhanpin[0].objectnum=1,
	zhanpin[0].objectnum_x=500,
	zhanpin[0].objectnum_y=500,
	zhanpin[0].direct=0,
	zhanpin[0].time=10000,
	zhanpin[0].mode=1,
	zhanpin[0].contect1="��Һã����Ǹո��ϸڵĽ�˵ԱС�飬���Թ�������ҵ��ѧ������ý�������װ�������£��Ļ������β��ص�ʵ���ҡ����ǿ����ṩ����������ͻ����ȷ�������ܻ����ˣ���ӭ��λ��Ħ�ص�ʵ����չ�������ڣ��������Ҳιۡ�",
	zhanpin[0].contect2="���ǵ�һ��չƷ",
	zhanpin[0].contect3="���ǵ�һ��չƷ",

	zhanpin[1].objectnum=2,
	zhanpin[1].objectnum_x=300,
	zhanpin[1].objectnum_y=500,
	zhanpin[1].direct=315 ,
	zhanpin[1].time=8000,
	zhanpin[1].mode=1,
	zhanpin[1].contect1="",
	zhanpin[1].contect2="",
	zhanpin[1].contect3="",

	zhanpin[2].objectnum=3,
	zhanpin[2].objectnum_x=300,
	zhanpin[2].objectnum_y=600,
	zhanpin[2].direct=180,
	zhanpin[2].time=5000,
	zhanpin[2].mode=1,
	zhanpin[2].contect1="",
	zhanpin[2].contect2="",
	zhanpin[2].contect3="",

	zhanpin[3].objectnum=99,
	zhanpin[3].objectnum_x=100,
	zhanpin[3].objectnum_y=600,
	zhanpin[3].direct=225,//������
	zhanpin[3].time=0,
	zhanpin[3].mode=3,
	zhanpin[3].contect1="",
	zhanpin[3].contect2="",
	zhanpin[3].contect3="",


	zhanpin[4].objectnum=4,
	zhanpin[4].objectnum_x=150,
	zhanpin[4].objectnum_y=600,
	zhanpin[4].direct=180,
	zhanpin[4].time=10000,
	zhanpin[4].mode=1,
	zhanpin[4].contect1="",
	zhanpin[4].contect2="",
	zhanpin[4].contect3="",

	zhanpin[5].objectnum=5,
	zhanpin[5].objectnum_x=0,
	zhanpin[5].objectnum_y=480,
	zhanpin[5].direct=0,
	zhanpin[5].mode=1,
	zhanpin[5].time=2000,
	zhanpin[5].contect1="",
	zhanpin[5].contect2="",
	zhanpin[5].contect3="",

	zhanpin[6].objectnum=6,
	zhanpin[6].objectnum_x=0,
	zhanpin[6].objectnum_y=375,
	zhanpin[6].direct=135,
	zhanpin[6].time=300000,
	zhanpin[6].mode=1,
	zhanpin[6].contect1="�����ҵĽ��С÷�����ǲ��Ǳ�������ѽ��С����͵������������滹�и����ʵ�չʾ���Ͳ�һһ����������ӭ�������������ҵ��ѧ���ͣ��ټ�����",
	zhanpin[6].contect2="���ǵ�����չƷ",
	zhanpin[6].contect3="���ǵ�����չƷ",

	zhanpin[7].objectnum=50,
	zhanpin[7].objectnum_x=400,
	zhanpin[7].objectnum_y=500,
	zhanpin[7].direct=180,//�����ʶ���
	zhanpin[7].time=0,
	zhanpin[7].mode=2,
	zhanpin[7].contect1="����������ǰ�������Ļ������β���18���ص�ʵ���ң������ǹ����Ļ��Ƽ�������ϵ����Ҫ��ɲ��֣������ۺ����������Ļ��Ƽ��˲ţ���֯�Ļ��Ƽ����ºͿ�չѧ����������Ҫƽ̨��",
	zhanpin[7].contect2="",
	zhanpin[7].contect3="",

	zhanpin[8].objectnum=51,
	zhanpin[8].objectnum_x=300,
	zhanpin[8].objectnum_y=550,
	zhanpin[8].direct=90,//�����ʶ���
	zhanpin[8].time=0,
	zhanpin[8].mode=2,
	zhanpin[8].contect1="�õģ��������Ҹ�������ǰ���ǹż������Ƽ��ص�ʵ���ң��������ڹ���ͼ��ݣ��������������з���ͼ�������������Ἴ�����������;������ᴦ����ͼ�顣",
	zhanpin[8].contect2="",
	zhanpin[8].contect3="",

	zhanpin[9].objectnum=52,
	zhanpin[9].objectnum_x=150,
	zhanpin[9].objectnum_y=600,
	zhanpin[9].direct=180,
	zhanpin[9].time=0,
	zhanpin[9].mode=2,
	zhanpin[9].contect1="�����������������ι���һ��չλ��˿���Ļ��������Ʒ������ֻ������ص�ʵ���ң����������㽭����ѧ������չʾ����˿��ͼ�����������ֻ���ƣ��������͡�",
	zhanpin[9].contect2="",
	zhanpin[9].contect3="",

	zhanpin[10].objectnum=53,
	zhanpin[10].objectnum_x=75,
	zhanpin[10].objectnum_y=520,
	zhanpin[10].direct=225,
	zhanpin[10].time=0,
	zhanpin[10].mode=2,
	zhanpin[10].contect1="�õģ����Ǽ��������ڴ�ҿ��Կ�ǰ�����黭�����ص�ʵ���ң������ڹʹ�����Ժ�����黭װ�Ѽ��޸��������ҹ����Ҽ����Ŵ�����Ŀ���й��黭��������������֮�����м���������������������չʾ������װ���޸���������ǧ�����������Ļ������ա�",
	zhanpin[10].contect2="",
	zhanpin[10].contect3="",


	zhanpin[11].objectnum=7,
	zhanpin[11].objectnum_x=500,
	zhanpin[11].objectnum_y=300,
	zhanpin[11].direct=225,
	zhanpin[11].time=5000,
	zhanpin[11].mode=1,
	zhanpin[11].contect1="",
	zhanpin[11].contect2="",
	zhanpin[11].contect3="",
};
//*******
int objectnums1[100]={1,50,2,51,3,52,4,99,53,5,6};//�����¼����չƷ��
//******
int objectnumshand[100]={0,7,10,8,6};//�����¼����չƷ��
//*******
int objectnums[100]={7,1};//�����¼����չƷ��

////////
int objectnowpos=0;//�ڼ���չƷ
int objectnow=0;//չƷ�������չƷ�������

int objectnowhand=0;

#define PIf 3.1415926
extern bool isbegio;
//////////////////////
bool zhanting=false;
bool ishand=false;
////////////////////////////2018/5/31//�����Զ�����
UINT ThreadComput_speakautotts(LPVOID lpParam);
CWinThread* pThread_speak_autotts;////�Զ������߳�
bool autotts_key=true;//�Զ����ſ���
bool isautotts=false;//�Ƿ���Բ���
int autotts_time_sleep=0; 
CRecorder_ExampleDlg *p_CR=NULL;
extern CVoice *p_DR;

///////////////////////////
#define WM_USER_SHOW_STATUS	WM_USER + 100

void Convert(char* strIn,char* strOut, int sourceCodepage, int targetCodepage);
int Json_Explain (char buf[],char Dest[],char result[]) ;

MicArraySession MicArraysession;
MicArrayHandle MicArrayhandle;


float hopedata_x = 0.00;
float hopedata_y = 0.00;
float hopedata_theta = 0.00;

char*tts;
CString *str;
//�ϳɺ���
void TTSSynth(const string &cap_key, const string &txt_file, const string &out_pcm_file);
bool HCIAPI TtsSynthCallbackFunction(_OPT_ _IN_ void * pvUserParam,
                                     _MUST_ _IN_ TTS_SYNTH_RESULT * psTtsSynthResult,
                                     _MUST_ _IN_ HCI_ERR_CODE  hciErrCode);
static HCI_ERR_CODE WakeupFunc(void *pUserContext, MICARRAY_WAKE_RESULT *pWakeResult);

static HCI_ERR_CODE WakeDirectionFunc(void *pUserContext, int nDirection);

static HCI_ERR_CODE VoiceReadyFunc(void *pUserContext, short *pVoiceData, int nVoiceSampleCount);
// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// CRecorder_ExampleDlg �Ի���

CRecorder_ExampleDlg::CRecorder_ExampleDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CRecorder_ExampleDlg::IDD, pParent)
	, m_recordingFlag(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	//  str = _T("");
	//  strMessage = _T("");
	//msg = _T("");
}

void CRecorder_ExampleDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Check(pDX, IDC_BTN_SAVE_RECORDING, m_recordingFlag);
	//  DDX_Control(pDX, IDC_BUTTON6, m_btn_NAV);
	//  DDX_Control(pDX, IDC_BTN_CANCEL_RECORD, m_btn_INDRO);
	//  DDX_Control(pDX, IDC_BTN_START_RECORD, addr_);
	//  DDX_Control(pDX, IDC_BUTTON2, dir_);
	//  DDX_Control(pDX, IDC_BUTTON1, m_IconBtn_up);
	DDX_Control(pDX, IDC_BUTTON1, m_btn1);
	//  DDX_Control(pDX, IDC_BUTTON2, m_btn2);
	//  DDX_Control(pDX, IDC_BUTTON3, m_btn2);
	DDX_Control(pDX, IDC_BUTTON5, m_btn3);
	DDX_Control(pDX, IDC_BUTTON4, m_btn4);
	DDX_Control(pDX, IDC_STATIC1, m_pic1);
	//  DDX_Control(pDX, IDC_MFCBUTTON3, m_btn2);
	DDX_Control(pDX, IDC_MFCBUTTON7, m_mfcbtn7);
	DDX_Control(pDX, IDC_MFCBUTTON1, m_mfcbtn1);
	DDX_Control(pDX, IDC_MFCBUTTON2, m_mfcbtn2);
	DDX_Control(pDX, IDC_MFCBUTTON3, m_mfcbtn3);
	DDX_Control(pDX, IDC_MFCBUTTON4, m_mfcbtn4);
	DDX_Control(pDX, IDC_MFCBUTTON5, m_mfcbtn5);
	DDX_Control(pDX, IDC_MFCBUTTON6, m_mfcbtn6);


}

BEGIN_MESSAGE_MAP(CRecorder_ExampleDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_MESSAGE(WM_USER_SHOW_STATUS, &CRecorder_ExampleDlg::OnShowStatus) 
	ON_BN_CLICKED(IDC_BTN_START_RECORD, &CRecorder_ExampleDlg::OnBnClickedBtnStartRecord)
	ON_BN_CLICKED(IDC_BTN_CANCEL_RECORD, &CRecorder_ExampleDlg::OnBnClickedBtnCancelRecord)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDOK, &CRecorder_ExampleDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_BROWSER, &CRecorder_ExampleDlg::OnBnClickedBtnBrowser)
	ON_BN_CLICKED(IDC_BTN_SAVE_RECORDING, &CRecorder_ExampleDlg::OnBnClickedSaveRecording)
	ON_EN_CHANGE(IDC_EDIT_STATUS, &CRecorder_ExampleDlg::OnEnChangeEditStatus)
	ON_BN_CLICKED(IDC_ONLY_RECORDING, &CRecorder_ExampleDlg::OnBnClickedOnlyRecording)
	ON_BN_CLICKED(IDC_CONTINUE, &CRecorder_ExampleDlg::OnBnClickedContinue)
	ON_EN_CHANGE(IDC_EDIT_SAVE_RECORDING_FILE, &CRecorder_ExampleDlg::OnEnChangeEditSaveRecordingFile)
	ON_BN_CLICKED(IDC_BUTTON1, &CRecorder_ExampleDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CRecorder_ExampleDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CRecorder_ExampleDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON4, &CRecorder_ExampleDlg::OnBnClickedButton4)
	ON_BN_CLICKED(IDC_BUTTON5, &CRecorder_ExampleDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON6, &CRecorder_ExampleDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CRecorder_ExampleDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON8, &CRecorder_ExampleDlg::OnBnClickedButton8)
	ON_EN_CHANGE(IDC_EDIT_WORDLIST, &CRecorder_ExampleDlg::OnEnChangeEditWordlist)
	ON_WM_CTLCOLOR()
	ON_WM_DRAWITEM()
	ON_WM_SIZE()
	ON_STN_CLICKED(IDC_STATIC1, &CRecorder_ExampleDlg::OnStnClickedStatic1)
	//ON_BN_CLICKED(IDC_BUTTON9, &CRecorder_ExampleDlg::OnBnClickedButton9)
	ON_BN_CLICKED(IDC_MFCBUTTON1, &CRecorder_ExampleDlg::OnBnClickedMfcbutton1)
	ON_BN_CLICKED(IDC_MFCBUTTON3, &CRecorder_ExampleDlg::OnBnClickedMfcbutton3)
	ON_BN_CLICKED(IDC_MFCBUTTON5, &CRecorder_ExampleDlg::OnBnClickedMfcbutton5)
	
	ON_BN_CLICKED(IDC_MFCBUTTON6, &CRecorder_ExampleDlg::OnBnClickedMfcbutton6)
	ON_BN_CLICKED(IDC_MFCBUTTON7, &CRecorder_ExampleDlg::OnBnClickedMfcbutton7)
	
	
	
	ON_BN_CLICKED(IDC_MFCBUTTON2, &CRecorder_ExampleDlg::OnBnClickedMfcbutton2)
	ON_BN_CLICKED(IDC_BUTTON11, &CRecorder_ExampleDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CRecorder_ExampleDlg::OnBnClickedButton12)
END_MESSAGE_MAP()

bool CheckAndUpdataAuth()
{
    //��ȡ����ʱ��
    int64 nExpireTime;
    int64 nCurTime = (int64)time( NULL );
    HCI_ERR_CODE errCode = hci_get_auth_expire_time( &nExpireTime );
    if( errCode == HCI_ERR_NONE )
    {
        //��ȡ�ɹ����ж��Ƿ����
        if( nExpireTime > nCurTime )
        {
            //û�й���
            printf( "auth can use continue\n" );
            return true;
        }
    }

    //��ȡ����ʱ��ʧ�ܻ��Ѿ�����
    //�ֶ����ø�����Ȩ
    errCode = hci_check_auth();
    if( errCode == HCI_ERR_NONE )
    {
        //���³ɹ�
        printf( "check auth success \n" );
        return true;
    }
    else
    {
        //����ʧ��
        printf( "check auth return (%d:%s)\n", errCode ,hci_get_error_info(errCode));
        return false;
    }
}

//��ȡcapkey����
void GetCapkeyProperty(const string&cap_key,AsrRecogType & type,AsrRecogMode &mode)
{
    HCI_ERR_CODE errCode = HCI_ERR_NONE;
	CAPABILITY_ITEM *pItem = NULL;

	// ö�����е�asr����
	CAPABILITY_LIST list = {0};
	if ((errCode = hci_get_capability_list("asr", &list))!= HCI_ERR_NONE)
	{
		// û���ҵ���Ӧ��������
		return;
	}

	// ��ȡasr����������Ϣ��
	for (int i = 0; i < list.uiItemCount; i++)
	{
		if (list.pItemList[i].pszCapKey != NULL && stricmp(list.pItemList[i].pszCapKey, cap_key.c_str()) == 0)
		{
			pItem = &list.pItemList[i];
			break;
		}
	}

	// û�л�ȡ��Ӧ�������ã����ء�
	if (pItem == NULL || pItem->pszCapKey == NULL)
	{
		hci_free_capability_list(&list);
		return;
	}

    
	if (strstr(pItem->pszCapKey, "cloud") != NULL)
	{
		type = kRecogTypeCloud;
	}
	else
	{
		type = kRecogTypeLocal;
	}  

	if (strstr(pItem->pszCapKey, "freetalk") != NULL)
	{
		mode = kRecogModeFreetalk;
	}
	else if (strstr(pItem->pszCapKey, "grammar") != NULL)
	{
		mode = kRecogModeGrammar;
	}
	else if (strstr(pItem->pszCapKey, "dialog") != NULL)
	{
		mode = kRecogModeDialog;
	}
	else
	{
		mode = kRecogModeUnkown;
	}

	hci_free_capability_list(&list);
       
    return;
};
// CRecorder_ExampleDlg ��Ϣ�������

BOOL CRecorder_ExampleDlg::OnInitDialog()
{
	CDialog::OnInitDialog();
	//����Բ�ǶԻ���
	/*SetWindowLong(m_hWnd,GWL_HWNDPARENT,NULL);
	CRgn m_rgn; 
	RECT rc; 
	GetWindowRect(&rc); 
	m_rgn.CreateRoundRectRgn(rc.left,rc.top,rc.right,rc.bottom,800,100); 
	SetWindowRgn(m_rgn,TRUE); */



	//SetWindowPos(NULL,0,0,GetSystemMetrics(SM_CXSCREEN),GetSystemMetrics(SM_CYSCREEN),0);
//�ı�


	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	CFont * f;   
    f = new CFont;   
    f->CreateFont(50, // nHeight   
    0, // nWidth   
    0, // nEscapement   
    0, // nOrientation   
    FW_BOLD, // nWeight   
    FALSE, // bItalic   
    FALSE, // bUnderline   
    0, // cStrikeOut   
    ANSI_CHARSET, // nCharSet   
    OUT_DEFAULT_PRECIS, // nOutPrecision   
    CLIP_DEFAULT_PRECIS, // nClipPrecision   
    DEFAULT_QUALITY, // nQuality   
    DEFAULT_PITCH | FF_SWISS, // nPitchAndFamily   
    _T("΢���ź�")); // lpszFac   
  
     // SetTextColor(HDC hDC,RGB(255,255,0)); //����������ɫ  
	/*GetDlgItem(IDC_STATIC4)->SetFont(f);
      GetDlgItem(IDC_MFCBUTTON7)->SetFont(f);  
	  GetDlgItem(IDC_MFCBUTTON2)->SetFont(f);  
	  GetDlgItem(IDC_MFCBUTTON5)->SetFont(f);
	  GetDlgItem(IDC_MFCBUTTON6)->SetFont(f);
	  GetDlgItem(IDC_MFCBUTTON1)->SetFont(f);  
	  GetDlgItem(IDC_MFCBUTTON3)->SetFont(f);  
	  GetDlgItem(IDC_MFCBUTTON4)->SetFont(f);
	  GetDlgItem(IDC_BTN_START_RECORD)->SetFont(f);  
	  GetDlgItem(IDC_BTN_CANCEL_RECORD)->SetFont(f); 

	 */
	 
      //����ť�޸�ΪBS_OWNERDRAW���,����button�Ĳ����Ի�ģʽ
     GetDlgItem(IDC_MFCBUTTON7)->ModifyStyle(0,BS_OWNERDRAW,0);
	 
     //�󶨿ؼ�IDC_BUTTON1����CMyButton����Ӧ���غ���DrawItem()
	
	  m_mfcbtn7.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn7.SetTextColor(RGB(92,55,27));
	  m_mfcbtn6.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn6.SetTextColor(RGB(92,55,27));
	  m_mfcbtn5.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn5.SetTextColor(RGB(92,55,27));
	  m_mfcbtn4.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn4.SetTextColor(RGB(92,55,27));
	   m_mfcbtn3.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn3.SetTextColor(RGB(92,55,27));
	   m_mfcbtn2.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn2.SetTextColor(RGB(92,55,27));
	     m_mfcbtn1.SetFaceColor(RGB(195,209,216));
	  m_mfcbtn1.SetTextColor(RGB(92,55,27));
	
	  
	  //button��ͼ,����
	 /* HBITMAP   hBitmap; 

		HBITMAP hbmp1=::LoadBitmap(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP1));
        m_btn1.SetBitmap(hbmp1);

		HBITMAP hbmp3=::LoadBitmap(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP3));
        m_btn3.SetBitmap(hbmp3);
		HBITMAP hbmp4=::LoadBitmap(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP4));
        m_btn4.SetBitmap(hbmp4);
  
  
       HBITMAP hbmp5=::LoadBitmap(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP5));
        m_pic1.SetBitmap(hbmp5);
		*/
	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�

	//����


	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	((CButton *)GetDlgItem( IDC_CONTINUE ))->SetCheck(TRUE);

	//////////////////xhy
	p_CR=(CRecorder_ExampleDlg*)this;
	//////////////////
    if (Init() == false)
    {
        return FALSE;
    }
	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

void CRecorder_ExampleDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CRecorder_ExampleDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		//CDialog::OnPaint(); 
		//��Ӵ���Ի��򱳾���ͼ
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //��ȡ�Ի��򳤿�      
		CDC   dcBmp;             //���岢����һ���ڴ��豸����
		dcBmp.CreateCompatibleDC(&dc);             //����������DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP7);    //������Դ��ͼƬ
		BITMAP   m_bitmap;                         //ͼƬ����               
		bmpBackground.GetBitmap(&m_bitmap);       //��ͼƬ����λͼ��
		//��λͼѡ����ʱ�ڴ��豸����
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//���ú�����ʾͼƬStretchBlt��ʾ��״�ɱ�
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
			}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CRecorder_ExampleDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

LRESULT CRecorder_ExampleDlg::OnShowStatus( WPARAM wParam, LPARAM lParam )
{
	CString * str = (CString *)lParam;
	AppendMessage(*str);
	delete str;

	RECORDER_EVENT eEvent = (RECORDER_EVENT)wParam;
	switch( eEvent )
	{
		// ���ǿ�ʼ¼���������������߿�ʼʶ����ʹ��ť������
	case RECORDER_EVENT_BEGIN_RECORD:
	case RECORDER_EVENT_BEGIN_RECOGNIZE:		
	case RECORDER_EVENT_HAVING_VOICE:
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );
		break;
		// ״̬���ֲ���
	case RECORDER_EVENT_ENGINE_ERROR:
		break;
		// ¼���������������
	case RECORDER_EVENT_END_RECORD:
	case RECORDER_EVENT_TASK_FINISH:
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
		break;
		// ʶ�����
	case RECORDER_EVENT_RECOGNIZE_COMPLETE:
		if (IsDlgButtonChecked( IDC_CONTINUE ) == FALSE)
		{
			GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
			GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
		}
		break;
		// ����״̬������δ�����������߷�������ȣ���ָ���ť����
	default:
		char buff[32];
		sprintf(buff, "Default Event:%d", eEvent);
		AppendMessage(CString(buff));

		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
	}

	return 0;
}

void CRecorder_ExampleDlg::OnBnClickedBtnStartRecord()
{	

	CString strErrorMessage;
	hci_tts_player_stop();
	hci_tts_player_release();

	RECORDER_ERR_CODE eRetasr = RECORDER_ERR_UNKNOWN;
	RECORDER_CALLBACK_PARAM call_back;
	memset( &call_back, 0, sizeof(RECORDER_CALLBACK_PARAM) );
	call_back.pvStateChangeUsrParam		= this;
	call_back.pvRecogFinishUsrParam		= this;
	call_back.pvErrorUsrParam			= this;
	call_back.pvRecordingUsrParam		= this;
	call_back.pvRecogProcessParam		= this;
    call_back.pfnStateChange	= CRecorder_ExampleDlg::RecordEventChange;
	call_back.pfnRecogFinish	= CRecorder_ExampleDlg::RecorderRecogFinish;
	call_back.pfnError			= CRecorder_ExampleDlg::RecorderErr;
	call_back.pfnRecording		= CRecorder_ExampleDlg::RecorderRecordingCallback;
	call_back.pfnRecogProcess   = CRecorder_ExampleDlg::RecorderRecogProcess;

	string initConfig = "initCapkeys=asr.cloud.dialog";	
	initConfig        += ",dataPath=../../data";
	//string initConfig = "dataPath=" + account_info->data_path();
	//initConfig      += ",encode=speex";
	//initConfig		+= ",initCapkeys=asr.local.grammar";			      //��ʼ����������

	eRetasr = hci_asr_recorder_init( initConfig.c_str(), &call_back);
	if (eRetasr != RECORDER_ERR_NONE)
	{
		hci_release();
		strErrorMessage.Format( "¼������ʼ��ʧ��,������%d", eRetasr);
		MessageBox( strErrorMessage );
		return ;
	}

	RECORDER_ERR_CODE eRet = RECORDER_ERR_NONE;
	
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );	
	
	// ���״̬��¼
	SetDlgItemText( IDC_EDIT1, "" );

    AccountInfo *account_info = AccountInfo::GetInstance();
    string startConfig = "";
	if (!IsDlgButtonChecked( IDC_ONLY_RECORDING ))
    {
     	startConfig += "capkey=" + account_info->cap_key();
	}
	startConfig += ",audioformat=pcm16k16bit";
	//startConfig += ",domain=qwdz,intention=qwmap;music,needcontent=no";
	//startConfig     += ",realTime=rt";
    if (IsDlgButtonChecked( IDC_CONTINUE ))
    {
        startConfig += ",continuous=yes";
    }
	if ( m_RecogMode == kRecogModeGrammar )
	{
		char chTmp[32] = {0};
		sprintf(chTmp,",grammarid=%d",m_GrammarId);
		startConfig += chTmp; 
	}

	if ( m_RecogMode == kRecogModeDialog )
	{
		startConfig +=",intention=weather;joke;story;baike;calendar;translation;news"; 
	}

	eRet = hci_asr_recorder_start(startConfig.c_str(),"");
	if (RECORDER_ERR_NONE != eRet)
	{
		CString strErrMessage;
		strErrMessage.Format( "��ʼ¼��ʧ��,������%d", eRet );
		MessageBox( strErrMessage );
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		return;
	}
}

bool CRecorder_ExampleDlg::Init()
{	
    CString strErrorMessage;


	m_recordingFlag = FALSE;
	m_recordingFileName = "recording.pcm";
	m_recordingFile = NULL;
	SetDlgItemText( IDC_EDIT_SAVE_RECORDING_FILE, m_recordingFileName );
	UpdateData(FALSE);

    // ��ȡAccountInfo����
    AccountInfo *account_info = AccountInfo::GetInstance();
    // �˺���Ϣ��ȡ
    string account_info_file = "../../testdata/AccountInfo.txt";
    bool account_success = account_info->LoadFromFile(account_info_file);
    if (!account_success)
    {
        strErrorMessage.Format("AccountInfo read from %s failed\n", account_info_file.c_str());
        MessageBox(strErrorMessage);
        return false;
    }

    // SYS��ʼ��
    HCI_ERR_CODE errCode = HCI_ERR_NONE;
    // ���ô�����"�ֶ�=ֵ"����ʽ������һ���ַ���������ֶ�֮����','�������ֶ������ִ�Сд��
    string init_config = "";
    init_config += "appKey=" + account_info->app_key();              //����Ӧ�����
    init_config += ",developerKey=" + account_info->developer_key(); //���ƿ�������Կ
    init_config += ",cloudUrl=" + account_info->cloud_url();         //�����Ʒ���Ľӿڵ�ַ
	init_config += ",capKey=sma.local.wake;sma.local.doa;sma.local.vqe" ; 
	init_config += ",authpath=" + account_info->auth_path();         //��Ȩ�ļ�����·������֤��д
    init_config += ",logfilepath=" + account_info->logfile_path();   //��־��·��
	init_config += ",logfilesize=1024000,loglevel=5";
    // ��������ʹ��Ĭ��ֵ��������ӣ���������ÿ��Բο������ֲ�
    errCode = hci_init( init_config.c_str() );
    if( errCode != HCI_ERR_NONE )
    {
        strErrorMessage.Format( "hci_init return (%d:%s)\n", errCode, hci_get_error_info(errCode) );
        MessageBox(strErrorMessage);
        return false;
    }
    printf( "hci_init success\n" );


    // �����Ȩ,��Ҫʱ���ƶ�������Ȩ���˴���Ҫע����ǣ��������ֻ��ͨ�������Ȩ�Ƿ�������ж��Ƿ���Ҫ����
    // ��ȡ��Ȩ����������ڿ������Թ����У���Ȩ�˺�������������sdk���������뵽hci_init�����authPath·����
    // ɾ��HCI_AUTH�ļ��������޷���ȡ�µ���Ȩ�ļ����Ӷ��޷�ʹ������������������
    if (!CheckAndUpdataAuth())
    {
        hci_release();
        strErrorMessage.Format("CheckAndUpdateAuth failed\n");
        MessageBox(strErrorMessage);
        return false;
    }

    // capkey���Ի�ȡ
    m_RecogType = kRecogTypeUnkown;
    m_RecogMode = kRecogModeUnkown;
    GetCapkeyProperty(account_info->cap_key(),m_RecogType,m_RecogMode);

	if( m_RecogType == kRecogTypeCloud && m_RecogMode == kRecogModeGrammar )
	{
        // �ƶ��﷨��ʱ��֧��ʵʱʶ��
		// GetDlgItem( IDC_REALTIME )->EnableWindow(FALSE);
		hci_release();
        strErrorMessage.Format("Recorder not support cloud grammar, init failed\n");
        MessageBox(strErrorMessage);
        return false;
	}

/*��˷��ʼ��*/
	char *configFile ="hci_micarray.ini";
	errCode = hci_micarray_init(configFile, &MicArrayhandle);
/*	if(errCode != HCI_ERR_NONE)
	{
		strErrorMessage.Format("hci_micarray_init failed (%d:%s)\n", errCode, hci_get_error_info(errCode));
        MessageBox(strErrorMessage);

		return -3;
	}
*/	hci_micarray_statechange_func scf = NULL;
	hci_micarray_wakeup_func wuf = WakeupFunc;
	hci_micarray_wakedirection_func wdf = WakeDirectionFunc;
	hci_micarray_voiceready_func vrf = VoiceReadyFunc;

	errCode = hci_micarray_session_start(MicArrayhandle, scf, wuf, wdf, vrf, NULL, &MicArraysession);


/*asr_recorder��ʼ��*/
    RECORDER_ERR_CODE eRet = RECORDER_ERR_UNKNOWN;
/*	RECORDER_CALLBACK_PARAM call_back;
	memset( &call_back, 0, sizeof(RECORDER_CALLBACK_PARAM) );
	call_back.pvStateChangeUsrParam		= this;
	call_back.pvRecogFinishUsrParam		= this;
	call_back.pvErrorUsrParam			= this;
	call_back.pvRecordingUsrParam		= this;
	call_back.pvRecogProcessParam		= this;
    call_back.pfnStateChange	= CRecorder_ExampleDlg::RecordEventChange;
	call_back.pfnRecogFinish	= CRecorder_ExampleDlg::RecorderRecogFinish;
	call_back.pfnError			= CRecorder_ExampleDlg::RecorderErr;
	call_back.pfnRecording		= CRecorder_ExampleDlg::RecorderRecordingCallback;
	call_back.pfnRecogProcess   = CRecorder_ExampleDlg::RecorderRecogProcess;
*/
	//{
	//	// TODO
	//	while(true)
	//	{
	//		RECORDER_ERR_CODE errCode;
	//		string strConfig = string("initCapKeys=asr.local.grammar.v4,dataPath=") + account_info->data_path();
	//		errCode = hci_asr_recorder_init(strConfig.c_str(), &call_back);
	//		assert(RECORDER_ERR_NONE == errCode);

	//		strConfig = "capKey=asr.local.grammar.v4,grammarType=wordlist,audioFormat=pcm16k16bit,isFile=yes"; // + getLocalGrammarConfig();
	//		string grammarDataPath = "../../testdata/wordlist_utf8.txt";
	//		errCode = hci_asr_recorder_start(strConfig.c_str(), grammarDataPath.c_str());
	//		assert(RECORDER_ERR_NONE == errCode);

	//		Sleep(5000);
	//		errCode = hci_asr_recorder_release();
	//		assert(RECORDER_ERR_NONE == errCode);
	//	}
	//}
/*
	string initConfig = "initCapkeys=" + account_info->cap_key();	
	initConfig        += ",dataPath=" + account_info->data_path();
	//string initConfig = "dataPath=" + account_info->data_path();
	//initConfig      += ",encode=speex";
	//initConfig		+= ",initCapkeys=asr.local.grammar";			      //��ʼ����������

	eRet = hci_asr_recorder_init( initConfig.c_str(), &call_back);
	if (eRet != RECORDER_ERR_NONE)
	{
		hci_release();
		strErrorMessage.Format( "¼������ʼ��ʧ��,������%d", eRet);
		MessageBox( strErrorMessage );
		return false;
	}
*/
    m_GrammarId = -1;
    if (m_RecogMode == kRecogModeGrammar)
    {
        string grammarFile = account_info->test_data_path() + "/stock_10001.gram";
        if (m_RecogType == kRecogTypeLocal)
        {
			string strLoadGrammarConfig = "grammarType=jsgf,isFile=yes,capkey=" + account_info->cap_key();
            eRet = hci_asr_recorder_load_grammar(strLoadGrammarConfig.c_str() , grammarFile.c_str(), &m_GrammarId );
            if( eRet != RECORDER_ERR_NONE )
            {
                hci_asr_recorder_release();
                hci_release();
                strErrorMessage.Format( "�����﷨�ļ�ʧ��,������%d", eRet );
                MessageBox( strErrorMessage );
                return false;
            }
            EchoGrammarData(grammarFile);
        }
        else
        {
            // ������ƶ��﷨ʶ����Ҫ������ͨ�����������������ϴ��﷨�ļ�������ÿ���ʹ�õ�ID��
            // m_GrammarId = 2;
        }
    }

	/*tts_player��ʼ��*/
/*
	PLAYER_CALLBACK_PARAM cb;
    cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	cb.pvStateChangeUsrParam = this;
	cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	cb.pvProgressChangeUsrParam = this;
	cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	cb.pvPlayerErrorUsrParam = this;

	PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	string initConfigtts = "initCapkeys=tts.cloud.wangjing";
    initConfigtts += ",dataPath=../../data" ;
	eReti = hci_tts_player_init( initConfigtts.c_str(), &cb );
	if (eRet != PLAYER_ERR_NONE)
	{
		hci_release();
		CString str;
		str.Format( "��������ʼ��ʧ��,������%d.", eRet);
		MessageBox( str );
		
	}
*/

	//movexhy
	#ifdef COMM_MOTOR
	// ������ڳ�ʼ��
//	strcpy(StateResult[0].Name,"���������");
	if(motor.open_com_motor(COMM_MOTOR))
	{
		//������ݼ���
		pThread_Motor_Comput = AfxBeginThread(ThreadComput_MotorData,NULL);
		int a= 1;
	}
	else
	{
		int a=0;
	}
	#endif

#ifdef COMM_STAR
	//�Ǳ궨λ���ڳ�ʼ��
//	strcpy(StateResult[5].Name,"�Ǳ궨λϵͳ");
	if (StarGazer.open_com(COMM_STAR))
	{
//		StateResult[5].Result = 1;
		int a=1;
	} 
	else
	{
//		StateResult[5].Result = 0;
		int a=0;
	}
	
#endif
//xhyvhf
#ifdef COMM_LASER
//	strcpy(StateResult[2].Name,"�����״�");
	//�������ݳ�ʼ��ֵ10000
	for(int loop=0;loop<1000;loop++)
	{
		m_laser_data_postpro[loop] = 50000;
	}

	// ���⴮�ڳ�ʼ��
	if (m_cURG.Create(COMM_LASER))
	{
		//MessageBox("ddddd  ����һ����򵥵���Ϣ��");
		m_cURG.SwitchOn();
		m_cURG.SCIP20();//////////////////////////	
		
		m_cURG.GetDataByGD(0,768,1);

		pThread_Read_Laser=AfxBeginThread(ThreadReadLaser_Data,&Info_laser_data);
	//	Sleep(100);
	////	pThread_Read_Laser_PostPro=AfxBeginThread(ThreadReadLaser_Data_PostPro,&Info_laser_data_postpro);
	//	Sleep(200);
	////	pReadThread_Laser_Pose_Compute = AfxBeginThread(ThreadLaser_Post_Computer,NULL);
	//	StateResult[2].Result = 1;
	//}
	//else
	//{
	//	StateResult[2].Result = 0;
	}
#endif

	plan.Scene_scale_x = 20.0;
	plan.Scene_scale_y = 20.0;
	plan.danger = false;
	vfh_Scene_scale_x = 20.0;
	vfh_Scene_scale_y = 20.0;
	Obstacle_Distance_init = 1000;/////////////////////////��ʼ���Ͼ����趨
	delt_Obstacle_Distance_init = 200;
	free_Obstacle_Distance_init = 1500;
	dataexchange_key = true;
	vfh_key = true;
	motor_key = false;

	//�������ݳ�ʼ��
	for (int loop = 0;loop<1000;loop++)
	{
		plan.m_laser_data_postpro[loop] = 50000;
		m_laser_data_postpro_vfh[loop] = 50000;
	}

	//��VFH�㷨�����ݽ���
	pThread_DataExchange=AfxBeginThread(ThreadDataExchange,NULL);
	plan.Init();
	algorithm.Init();
	pThread_VFHStart = AfxBeginThread(ThreaVFH,NULL); //vfh��ʼ��,����λ���趨
	SetTimer(1,200,NULL);

    return true;
}

void CRecorder_ExampleDlg::EchoGrammarData(const string &grammarFile)
{
    FILE* fp = fopen( grammarFile.c_str(), "rt" );
    if( fp == NULL )
    {
        GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
        CString strErrorMessage;
        strErrorMessage.Format("���﷨�ļ�%sʧ��",grammarFile.c_str());
        MessageBox( strErrorMessage );
        return;
    }

    unsigned char szBom[3];
    fread( szBom, 3, 1, fp );
    // ����bomͷ���������û����ǰλ�ûص�ͷ��
    if( !( szBom[0] == 0xef && szBom[1] == 0xbb && szBom[2] == 0xbf ) )
    {
        fseek( fp, 0, SEEK_SET );
    }

    CString grammarData = "";
    char szData[1024] = {0};
    while( fgets( szData, 1024, fp ) != NULL )
    {
        unsigned char* pszGBK = NULL;
        HciExampleComon::UTF8ToGBK( (unsigned char*)szData, &pszGBK);
        grammarData += (char*)pszGBK;
        HciExampleComon::FreeConvertResult( pszGBK );
        grammarData += "\r\n";
    }

    fclose( fp );
    SetDlgItemText( IDC_EDIT_WORDLIST, grammarData );
    return;
}

bool CRecorder_ExampleDlg::Uninit(void)
{
	HCI_ERR_CODE eRet = HCI_ERR_NONE;	
	// ����Ǳ����﷨ʶ������Ҫ�ͷ��﷨��Դ
	if( m_RecogType == kRecogTypeLocal && m_RecogMode == kRecogModeGrammar )
	{
		hci_asr_recorder_unload_grammar( m_GrammarId );
	}
	RECORDER_ERR_CODE eRecRet;
	eRecRet = hci_asr_recorder_release();

	if(eRecRet != RECORDER_ERR_NONE)
	{
		return false;
	}
	HCI_ERR_CODE errCode;
	errCode = hci_micarray_session_stop(&MicArraysession);

	errCode = hci_micarray_release(&MicArrayhandle);

	eRet = hci_release();

	AccountInfo::ReleaseInstance();

	return eRet == HCI_ERR_NONE;
}

void CRecorder_ExampleDlg::OnBnClickedBtnCancelRecord()
{
	RECORDER_ERR_CODE eRet = hci_asr_recorder_cancel();
	if (RECORDER_ERR_NONE != eRet)
	{
		CString str;
		str.Format( _T("��ֹ¼��ʧ��,������%d"), eRet );
		MessageBox( str );
		return;
	}
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );

	hci_asr_recorder_release();

	/* tts */
	


	HCI_ERR_CODE err_code = HCI_ERR_NONE;

	string tts_init_config = "dataPath=";
    tts_init_config += "../../data";
	tts_init_config += ",initCapkeys=tts.cloud.wangjing";

	err_code = hci_tts_init(tts_init_config.c_str());
	if (err_code != HCI_ERR_NONE)
	{
		printf("hci_tts_init return (%d:%s) \n",err_code,hci_get_error_info(err_code));
		return;
	}

	// ���������ϳɲ����ļ�
	string file_to_synth;
	if ("tts.cloud.wangjing" == "tts.local.synth.sing")
	{
		file_to_synth = "../../testdata/S3ML_sing.txt.enc";
	}
	else
	{
		file_to_synth = "../../testdata/tts.txt";
	}
	
	string out_pcm_file = "../../testdata/ttsceshi.pcm";
	TTSSynth("tts.cloud.wangjing", file_to_synth, out_pcm_file);

	//TTS����ʼ��
	hci_tts_release();
    printf("hci_tts_release\n");

	/* tts_player */


	PLAYER_CALLBACK_PARAM cb;
    cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	cb.pvStateChangeUsrParam = this;
	cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	cb.pvProgressChangeUsrParam = this;
	cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	cb.pvPlayerErrorUsrParam = this;

	PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	string initConfig = "initCapkeys=tts.cloud.wangjing";
    initConfig += ",dataPath=../../data" ;
	eReti = hci_tts_player_init( initConfig.c_str(), &cb );
	if (eRet != PLAYER_ERR_NONE)
	{
		hci_release();
		CString str;
		str.Format( "��������ʼ��ʧ��,������%d.", eRet);
		MessageBox( str );
		
	}

	string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";

//	char *pUTF8Str=tts;
//	unsigned char* pszUTF8 = NULL;
//    HciExampleComon::GBKToUTF8( (unsigned char*)pUTF8Str, &pszUTF8 );


	PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)tts, startConfig.c_str() );
//    HciExampleComon::FreeConvertResult(pszUTF8);
	if(eRetk==PLAYER_ERR_NONE){
		int a=5;

	}
//	hci_tts_player_release();

}

	/*          *///////


void CRecorder_ExampleDlg::OnBnClickedOk()
{
	Uninit(); 

	OnOK();
}

struct 
{
    char* pszName;
    char* pszComment;
}g_sStatus[] =
{
    {"RECORDER_EVENT_BEGIN_RECORD",         "�ʸ������"},
    {"RECORDER_EVENT_HAVING_VOICE",         "�������� ��⵽ʼ�˵�ʱ��ᴥ�����¼�"},
    {"RECORDER_EVENT_NO_VOICE_INPUT",       "û����������"},
    {"RECORDER_EVENT_BUFF_FULL",            "������������"},
    {"RECORDER_EVENT_END_RECORD",           "��֯����ing"},
    {"RECORDER_EVENT_BEGIN_RECOGNIZE",      "������"},
    {"RECORDER_EVENT_RECOGNIZE_COMPLETE",   "�������"},
    {"RECORDER_EVENT_ENGINE_ERROR",         "�������"},
    {"RECORDER_EVENT_DEVICE_ERROR",         "�豸����"},
    {"RECORDER_EVENT_MALLOC_ERROR",         "����ռ�ʧ��"},
    {"RECORDER_EVENT_INTERRUPTED",          "�ڲ�����"},
    {"RECORDER_EVENT_PERMISSION_DENIED",    "�ڲ�����"},
    {"RECORDER_EVENT_TASK_FINISH",          "С��Ҫ��Ϣһ��"},
    {"RECORDER_EVENT_RECOGNIZE_PROCESS",    "ʶ���м�״̬"}
};

void HCIAPI CRecorder_ExampleDlg::RecordEventChange(RECORDER_EVENT eRecorderEvent, void *pUsrParam)
{
    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
	///hhhh
//    p_CR=(CRecorder_ExampleDlg*)pUsrParam;

	if(eRecorderEvent == RECORDER_EVENT_BEGIN_RECOGNIZE)
	{
		dlg->m_startClock = clock();
	}
	if(eRecorderEvent == RECORDER_EVENT_END_RECORD)
	{
		if(dlg->m_recordingFile != NULL)
		{
			fclose(dlg->m_recordingFile);
			dlg->m_recordingFile = NULL;
		}
	}

	CString strMessage(g_sStatus[eRecorderEvent].pszComment);
	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}

void CRecorder_ExampleDlg::AppendMessage(CString & strMsg)
{
    CString strMessage = "";
    p_DR->GetDlgItemText( IDC_EDIT1, strMessage );
    
	int nMessageLenMax = 1024;
	if(strMessage.GetLength() > nMessageLenMax)
	{
		strMessage = strMessage.Right(nMessageLenMax);
	}
	
	CString strNewMessage = "";
	strNewMessage = strMsg;
	if(strMessage.GetLength() > 0)
	{
		strNewMessage += "\r\n";
		strNewMessage += strMessage;
		
		
	}
	
    p_DR->SetDlgItemText( IDC_EDIT1, strNewMessage );

}
void CRecorder_ExampleDlg::PostRecorderEventAndMsg(RECORDER_EVENT eRecorderEvent, const CString & strMessage)
{
	CString * msg = new CString(strMessage);
	::PostMessage(m_hWnd, WM_USER_SHOW_STATUS, eRecorderEvent, (LPARAM)msg);
}

void HCIAPI CRecorder_ExampleDlg::RecorderRecogFinish(
                                       RECORDER_EVENT eRecorderEvent,
                                       ASR_RECOG_RESULT *psAsrRecogResult,
                                       void *pUsrParam)
{
	CString strMessage = "";

    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
	if(eRecorderEvent == RECORDER_EVENT_RECOGNIZE_COMPLETE)
	{
		char buff[32];
		clock_t endClock = clock();

		//strMessage.AppendFormat( "�����������", (int)endClock - (int)dlg->m_startClock );
		
		dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
	}

    strMessage = "";
    if( psAsrRecogResult->uiResultItemCount > 0 )
    {
        unsigned char* pucUTF8 = NULL;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, &pucUTF8 );
       
		unsigned char* pszGBK;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, (unsigned char**)&pszGBK);

		tts=(char* )pszGBK;

	  //strMessage.AppendFormat( "С��Ĵ�: %s", pucUTF8 );
        HciExampleComon::FreeConvertResult( pucUTF8 );
		 char buf[10000] = {NULL};
		 char result[10000]={NULL};
		Convert(psAsrRecogResult->psResultItemList[0].pszResult,buf,CP_UTF8,CP_ACP);
		int iLength ;
		Json_Explain(buf,buf,result);
		


		HciExampleComon::GBKToUTF8( (unsigned char*)buf, (unsigned char**)&pszGBK);


		tts=(char* )pszGBK;
		
		strMessage.AppendFormat( "��������: %s\r\nС��Ĵ�: %s", result , buf);

		
		
//		strMessage.AppendFormat( "ʶ����: %s", pucUTF8 );
//        HciExampleComon::FreeConvertResult( pucUTF8 );
        pucUTF8 = NULL;
		isautotts=true;
		autotts_time_sleep=strlen(tts);
    }
    else
    {
        strMessage.AppendFormat( "С��û���壬����˵һ����" );
    }
	
	
	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}

void HCIAPI CRecorder_ExampleDlg::RecorderRecogProcess(
                                        RECORDER_EVENT eRecorderEvent,
                                        ASR_RECOG_RESULT *psAsrRecogResult,
                                        void *pUsrParam)
{
    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
    CString strMessage = "";
    if( psAsrRecogResult->uiResultItemCount > 0 )
    {
        unsigned char* pucUTF8 = NULL;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, &pucUTF8 );
        strMessage.AppendFormat( "ʶ���м���: %s", pucUTF8 );
        HciExampleComon::FreeConvertResult( pucUTF8 );
        pucUTF8 = NULL;
    }
    else
    {
        strMessage.AppendFormat( "*****��ʶ����*****" );
    }

	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);    
}

void HCIAPI CRecorder_ExampleDlg::RecorderErr(
                               RECORDER_EVENT eRecorderEvent,
                               HCI_ERR_CODE eErrorCode,
                               void *pUsrParam)
{
    CRecorder_ExampleDlg * dlg = (CRecorder_ExampleDlg*)pUsrParam;
    CString strMessage = "";
    strMessage.AppendFormat( "ϵͳ����:%d", eErrorCode );

	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}

void HCIAPI CRecorder_ExampleDlg::RecorderRecordingCallback(
                                     unsigned char * pVoiceData,
                                     unsigned int uiVoiceLen,
                                     void * pUsrParam
                                     )
{
	CRecorder_ExampleDlg * dlg = (CRecorder_ExampleDlg *)pUsrParam;
	dlg->RecorderRecording(pVoiceData, uiVoiceLen);
}

void CRecorder_ExampleDlg::RecorderRecording(unsigned char * pVoiceData, unsigned int uiVoiceLen)
{
	if(m_recordingFlag == FALSE)
	{
		if(m_recordingFile != NULL)
		{
			fclose(m_recordingFile);
			m_recordingFile = NULL;
		}
		return;
	}

	if(m_recordingFile == NULL)
	{
		m_recordingFile = fopen( m_recordingFileName.GetBuffer(), "wb" );
		if( m_recordingFile == NULL )
		{
			return;
		}
	}

	fwrite(pVoiceData, sizeof(unsigned char), uiVoiceLen, m_recordingFile);
	fflush(m_recordingFile);
}

void CRecorder_ExampleDlg::OnBnClickedBtnBrowser()
{
	CFileDialog dlgFile(TRUE, NULL, m_recordingFileName.GetBuffer(), OFN_HIDEREADONLY, _T("PCM Files (*.pcm)|*.pcm|All Files (*.*)|*.*||"), NULL);
    if (dlgFile.DoModal())
    {
        m_recordingFileName = dlgFile.GetPathName();
    }
    
	SetDlgItemText( IDC_EDIT_SAVE_RECORDING_FILE, m_recordingFileName );
}

void CRecorder_ExampleDlg::OnBnClickedSaveRecording()
{
	UpdateData(TRUE);

	if(m_recordingFlag == FALSE)
	{
		if(m_recordingFile != NULL)
		{
			fclose(m_recordingFile);
			m_recordingFile = NULL;
		}
	}
}


void CRecorder_ExampleDlg::OnEnChangeEditStatus()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}


///////
bool HCIAPI TtsSynthCallbackFunction(_OPT_ _IN_ void * pvUserParam,
                                     _MUST_ _IN_ TTS_SYNTH_RESULT * psTtsSynthResult,
                                     _MUST_ _IN_ HCI_ERR_CODE  hciErrCode)
{
    if( hciErrCode != HCI_ERR_NONE )
    {
        return false;
    }

    //printf("voice data size %d\n",psTtsSynthResult->uiVoiceSize);
    // ���ϳɽ��д���ļ�
    if (psTtsSynthResult->pvVoiceData != NULL)
    {
        FILE * fp = (FILE *)pvUserParam;
        fwrite(psTtsSynthResult->pvVoiceData, psTtsSynthResult->uiVoiceSize, 1, fp);
    }

	//mark �ص����
	if (psTtsSynthResult->nMarkCount > 0)
	{
		for (int i=0; i<psTtsSynthResult->nMarkCount; ++i)
		{
			printf("MarkName:%s, with the time in audio:%d \n",psTtsSynthResult->pMark[i].pszName,psTtsSynthResult->pMark[i].time);
		}

	}

    // �˻ص���������false����ֹ�ϳɣ�����true��ʾ�����ϳ�
    return true;
}

void TTSSynth(const string &cap_key, const string &txt_file, const string &out_pcm_file )
{
    // �ϳ��ı���ȡ
    HciExampleComon::FileReader txt_data;
    if( txt_data.Load(txt_file.c_str(),1) == false )
    {
        printf( "Open input text file %s error!\n", txt_file.c_str() );
        return;
    }

    // ������ļ�
    FILE * fp = fopen( out_pcm_file.c_str(), "wb" );
    if( fp == NULL )
    {
        printf( "Create output pcm file %s error!\n", out_pcm_file.c_str());
        return;
    }

    HCI_ERR_CODE err_code = HCI_ERR_NONE;
    // ���� TTS Session
    string session_config = "capkey=";
    session_config += cap_key;
    int session_id = -1;

    printf( "hci_tts_session_start config [%s]\n", session_config.c_str() );
    err_code = hci_tts_session_start( session_config.c_str(), &session_id );
    if( err_code != HCI_ERR_NONE )
    {
        printf("hci_tts_session_start return (%d:%s) \n",err_code,hci_get_error_info(err_code));
        fclose(fp);
        return;
    }
    printf( "hci_tts_session_start success\n" );

	string synth_config;
	if (cap_key.find("tts.cloud.synth") != string::npos)
	{
		//property ���� ˽���� �ƶ����� ���������������ο������ֲ�
		//none: ���б�ǽ��ᱻ��Ϊ�ı�������ȱʡֵ
		synth_config = "property=cn_xiaokun_common,tagmode=none";
	}

	if (cap_key.find("tts.local.synth.sing") != string::npos)
	{
		synth_config = "tagmode=s3ml_sing";
	}
//*	char*tts="����";
	char *pUTF8Str=tts;
	
	unsigned char* pszGBK;
    HciExampleComon::GBKToUTF8( (unsigned char*)pUTF8Str, (unsigned char**)&pszGBK);

    err_code = hci_tts_synth( session_id, (char*)pszGBK, synth_config.c_str(), TtsSynthCallbackFunction, fp );
//*/    
//	err_code = hci_tts_synth( session_id, (char*)txt_data.buff_, synth_config.c_str(), TtsSynthCallbackFunction, fp );
	fclose(fp);

    if( err_code != HCI_ERR_NONE )
    {
        printf("hci_tts_session_start return (%d:%s) \n",err_code,hci_get_error_info(err_code));
    }

    // ��ֹ TTS Session
    err_code = hci_tts_session_stop( session_id );
    if( err_code != HCI_ERR_NONE )
    {
        printf( "hci_tts_session_stop return %d\n", err_code );
        return;
    }
    printf( "hci_tts_session_stop success\n" );

    return;
}

void CRecorder_ExampleDlg::OnBnClickedOnlyRecording()
{
	// TODO: Add your control notification handler code here
}


void CRecorder_ExampleDlg::OnBnClickedContinue()
{
	// TODO: Add your control notification handler code here
}


void CRecorder_ExampleDlg::OnEnChangeEditSaveRecordingFile()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}

void HCIAPI CRecorder_ExampleDlg::CB_EventChange(
                           _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
                           _OPT_ _IN_ void * pUsrParam)
{
    string strEvent;
    switch ( ePlayerEvent ) 
    {
    case PLAYER_EVENT_BEGIN:
        strEvent = "��ʼ����";
        break;
    case PLAYER_EVENT_PAUSE:
        strEvent = "��ͣ����";
        break;
    case PLAYER_EVENT_RESUME:
        strEvent = "�ָ�����";
        break;
    case PLAYER_EVENT_PROGRESS:
        strEvent = "���Ž���";
        break;
    case PLAYER_EVENT_BUFFERING:
        strEvent = "���Ż���";
        break;
    case PLAYER_EVENT_END:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_ENGINE_ERROR:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_DEVICE_ERROR:
        strEvent = "�豸����";
        break;
    }
/*    TRACE ( "%s: %s\n", __FUNCTION__, strEvent.c_str() );
    CTTSPlayer_ExampleDlg* dlg = (CTTSPlayer_ExampleDlg*)pUsrParam;
    dlg->SetDlgItemText( IDC_EDIT_STATUS, (char*)strEvent.c_str() );

	*/
}


void HCIAPI CRecorder_ExampleDlg::CB_ProgressChange (
                               _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
                               _MUST_ _IN_ int nStart,
                               _MUST_ _IN_ int nStop,
                               _OPT_ _IN_ void * pUsrParam)
{
    string strEvent;
    char szData[256] = {0};
    switch ( ePlayerEvent ) 
    {
        //  case PLAYER_EVENT_INIT_COMPLETE:
        //  	strEvent = "��ʼ�����";
        //  	break;
    case PLAYER_EVENT_BEGIN:
        strEvent = "��ʼ����";
        break;
    case PLAYER_EVENT_PAUSE:
        strEvent = "��ͣ����";
        break;
    case PLAYER_EVENT_RESUME:
        strEvent = "�ָ�����";
        break;
    case PLAYER_EVENT_PROGRESS: //����Ľ�����תΪUTF-8֮������յ�
        sprintf( szData, "���Ž��ȣ���ʼ=%d,�յ�=%d", nStart, nStop );
        strEvent = szData;
        break;
    case PLAYER_EVENT_BUFFERING:
        strEvent = "���Ż���";
        break;
    case PLAYER_EVENT_END:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_ENGINE_ERROR:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_DEVICE_ERROR:
        strEvent = "�豸����";
        break;
    }
/*    TRACE ( "%s: %s\n", __FUNCTION__, strEvent.c_str() );
    CTTSPlayer_ExampleDlg* dlg = (CTTSPlayer_ExampleDlg*)pUsrParam;
    dlg->SetDlgItemText( IDC_EDIT_STATUS, (char*)strEvent.c_str() );
*/
}


void HCIAPI CRecorder_ExampleDlg::CB_SdkErr( _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,
                      _MUST_ _IN_ HCI_ERR_CODE eErrorCode,
                      _OPT_ _IN_ void * pUsrParam )
{
    string strEvent;
    switch ( ePlayerEvent ) 
    {
    case PLAYER_EVENT_BEGIN:
        strEvent = "��ʼ����";
        break;
    case PLAYER_EVENT_PAUSE:
        strEvent = "��ͣ����";
        break;
    case PLAYER_EVENT_RESUME:
        strEvent = "�ָ�����";
        break;
    case PLAYER_EVENT_PROGRESS:
        strEvent = "���Ž���";
        break;
    case PLAYER_EVENT_BUFFERING:
        strEvent = "���Ż���";
        break;
    case PLAYER_EVENT_END:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_ENGINE_ERROR:
        strEvent = "�������";
        break;
    case PLAYER_EVENT_DEVICE_ERROR:
        strEvent = "�豸����";
        break;
    }

/*    TRACE ( "%s: %s\n", __FUNCTION__, strEvent.c_str() );
    CTTSPlayer_ExampleDlg* dlg = (CTTSPlayer_ExampleDlg*)pUsrParam;
    dlg->SetDlgItemText( IDC_EDIT_STATUS, (char*)strEvent.c_str() );
*/
}

static HCI_ERR_CODE WakeupFunc(void *pUserContext, MICARRAY_WAKE_RESULT *pWakeResult)
{
	HCI_ERR_CODE errCode = HCI_ERR_NONE;

	return errCode;
}

static HCI_ERR_CODE WakeDirectionFunc(void *pUserContext, int nDirection)
{
	HCI_ERR_CODE errCode = HCI_ERR_NONE;

	printf(":: Wake Direction : %p, %d\n", pUserContext, nDirection);

	return errCode;
}

static HCI_ERR_CODE VoiceReadyFunc(void *pUserContext, short *pVoiceData, int nVoiceSampleCount)
{

	HCI_ERR_CODE errCode = HCI_ERR_NONE;

	static FILE * m_fpOut = NULL;

	int size;
	char *filename = "mictest.pcm";

	if(m_fpOut == NULL)
	{
		//printf("Start record after wakeup: %d.\n", nRecordTime);

		m_fpOut = fopen(filename, "wb");
		if(m_fpOut == NULL)
		{
			printf("VoiceReadyFunc Open File Failed:%s.\n", filename);

			errCode = HCI_ERR_UNSUPPORT;

			return errCode;
		}
	}

	if(m_fpOut != NULL)
	{
		size = fwrite(pVoiceData, sizeof(short), nVoiceSampleCount, m_fpOut);
		if(size != nVoiceSampleCount)
		{
			printf(":: fwrite data failed, %d of %d, %d, %s\n",
			       size, nVoiceSampleCount, errno, strerror(errno));
		}
		fflush(m_fpOut);
	}

	return HCI_ERR_NONE;
}


void Convert(char* strIn,char* strOut, int sourceCodepage, int targetCodepage)  
{  
	int len=lstrlen(strIn);  
	int unicodeLen=MultiByteToWideChar(sourceCodepage,0,strIn,-1,NULL,0);  
	wchar_t* pUnicode;  
	pUnicode=new wchar_t[unicodeLen+1];  
	memset(pUnicode,0,(unicodeLen+1)*sizeof(wchar_t));  
	MultiByteToWideChar(sourceCodepage,0,strIn,-1,(LPWSTR)pUnicode,unicodeLen);  
	BYTE * pTargetData = NULL;  
	int targetLen=WideCharToMultiByte(targetCodepage,0,(LPWSTR)pUnicode,-1,(char *)pTargetData,0,NULL,NULL);  
	pTargetData=new BYTE[targetLen+1];  
	memset(pTargetData,0,targetLen+1);  
	WideCharToMultiByte(targetCodepage,0,(LPWSTR)pUnicode,-1,(char *)pTargetData,targetLen,NULL,NULL);  
	lstrcpy(strOut,(char*)pTargetData);  
	delete pUnicode;  
	delete pTargetData;  
}  

  int Json_Explain (char buf[],char Dest[],char result[])  
{  
	cJSON *json , *json_result, *json_intention, *json_answer,*json_content,*date,*description,*direction,*high,*low,*location,
		*power,*domain,*content,*text,*date_gongli,*lunarDay,*lunarMonth,*week,*holiday,*channelName,*desc,*title;
	char temp[1000] = {NULL};
	// �������ݰ�  
	json = cJSON_Parse(buf);  
	if (!json)  
	{  
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
	}  
else  
	{  
		// ��������ֵ  
		json_result = cJSON_GetObjectItem( json, "result");  //intention=weather;calendar;train;flight;joke;story;baike
		json_answer =  cJSON_GetObjectItem(json , "answer");
		
		json_content = cJSON_GetObjectItem(json_answer , "content");
		json_intention=cJSON_GetObjectItem(json_answer , "intention");

		domain=cJSON_GetObjectItem(json_intention , "domain");

		
		if( json_result != NULL &&domain != NULL )  
		{
			strcpy(result,json_result->valuestring);
			if(!strcmp(domain->valuestring,"weather")){
			// ��valuestring�л�ý��  
				date = cJSON_GetObjectItem(json_content , "date");
				description = cJSON_GetObjectItem(json_content , "description");
				direction = cJSON_GetObjectItem(json_content , "direction");
				location = cJSON_GetObjectItem(json_content , "lcoation");
				power = cJSON_GetObjectItem(json_content , "power");
				high = cJSON_GetObjectItem(json_content , "high");
				low = cJSON_GetObjectItem(json_content , "low");

				strcpy(Dest,json_result->valuestring);
				strcpy(Dest,domain->valuestring);
				
				strcat(Dest,",");
				strcat(Dest,date->valuestring);
				strcat(Dest,",");
			    strcat(Dest,description->valuestring);
				strcat(Dest,",");
				strcat(Dest,direction->valuestring);
				strcat(Dest,",");
				strcat(Dest,"����");
				strcat(Dest,power->valuestring);
				
				strcat(Dest,",");
				//strcat(Dest,location->valuestring);
				strcat(Dest,"�������");
				itoa(high->valueint,temp,10);
				strcat(Dest,temp);
					strcat(Dest,"���϶�");
					strcat(Dest,",");
					strcat(Dest,"�������");
				itoa(low->valueint,temp,10);
				strcat(Dest,temp);
					strcat(Dest,"���϶�");
					strcat(Dest,"��");
			}
			else if(!strcmp(domain->valuestring,"joke")){
				 content = cJSON_GetObjectItem(json_content , "content");	
				strcpy(Dest,content->valuestring);
				
			}
			else if(!strcmp(domain->valuestring,"story")){
				
				content = cJSON_GetObjectItem(json_content , "content");
				strcpy(Dest,content->valuestring);
				
			}
			else if(!strcmp(domain->valuestring,"baike")){
				
				content = cJSON_GetObjectItem(json_content , "content");
				text = content = cJSON_GetObjectItem(json_content , "text");
				strcpy(Dest,text->valuestring);
				
			}
			else if(!strcmp(domain->valuestring,"calendar")){
				content = cJSON_GetObjectItem(json_content , "content");
				date_gongli =  cJSON_GetObjectItem(json_content , "date");
				lunarMonth =  cJSON_GetObjectItem(json_content , "lunarMonthChinese");
				lunarDay =  cJSON_GetObjectItem(json_content , "lunarDayChinese");
				week =  cJSON_GetObjectItem(json_content , "weekday");
				holiday = cJSON_GetObjectItem(json_content , "holiday");
				strcpy(Dest,date_gongli->valuestring);
				strcat(Dest,",");
				strcat(Dest,"ũ��");
				strcat(Dest,lunarMonth->valuestring);
				strcat(Dest,lunarDay->valuestring);
				strcat(Dest,",");
				strcat(Dest,week->valuestring);
				strcat(Dest,",");
				strcat(Dest,holiday->valuestring);
				strcat(Dest,".");
			}
			else if(!strcmp(domain->valuestring,"translation")){
				
				content = cJSON_GetObjectItem(json_content , "content");
				text = content = cJSON_GetObjectItem(json_content , "text");
				strcpy(Dest,text->valuestring);
				
			}
			else if(!strcmp(domain->valuestring,"news")){
				
				content = cJSON_GetObjectItem(json_content , "content");
				channelName = cJSON_GetObjectItem(json_content , "channelName");
				desc = cJSON_GetObjectItem(json_content , "desc");
				title = cJSON_GetObjectItem(json_content , "title");
				strcpy(Dest,channelName->valuestring);
				strcat(Dest,",");
				strcat(Dest,title->valuestring);
				strcat(Dest,",");
				strcat(Dest,desc->valuestring);
				
			}
					
		}
		else
		{  strcpy(Dest,"��ѽ,��������Ҳ��ᰡ,��̽��Ұ�!");}
		// �ͷ��ڴ�ռ�  
		cJSON_Delete(json); 
	}
	
	
	return 0;  
  }  

  //movexhy

  //������ݼ����߳�
  UINT ThreadComput_MotorData(LPVOID lpParam)
{
	while(moter_key)
	{
		//Sleep(200);
		WaitForSingleObject(wait_motordata,200);
	//	distance_l = (float)motor.encoder_l*100/1160/27;
	//	distance_r = (float)motor.encoder_r*100/1160/27;

		distance_l = -(float)motor.encoder_l/1024/26*102*(1-0.23);//(1-0.23)Ϊʵ�ʲ���������������ƫ������
		distance_r = -(float)motor.encoder_r/1024/26*102*(1-0.23);
		distance_z = -(float)motor.encoder_z/1024/26*102*(1-0.23);

		if (distanceold_l != 0)
		{
			distancedif_l = distance_l - distanceold_l;	//�����ֵ����
		}
		if (distanceold_r != 0)
		{
			distancedif_r = distance_r - distanceold_r;
		}
		if (distanceold_z != 0)
		{
			distancedif_z = distance_z - distanceold_z;
		}

		distanceold_l = distance_l;  //���¼�¼
		distanceold_r = distance_r;
		distanceold_z = distance_z;

		motor.RobotPositionCompute(distancedif_l,distancedif_r,distancedif_z,Info_robot);
		//if (motortest.last_angle == 0.00)
		//{
		//	motortest.angle = StarGazer.starAngel;
		//}
		//motortest.angle = StarGazer.starAngel;

		//��λ�˸��µ���ͼ��
		//	map_world.Map_update((int)(Info_robot.pointrox/map_world.Scene_scale_x),(int)(Info_robot.pointroy/map_world.Scene_scale_y),4);
		//	map_world.Map_update((int)(Info_robot.pointrox/map_world.Scene_scale_x),(int)(Info_robot.pointroy/map_world.Scene_scale_y),5);
		//	Laser_Post_Computer();
		//	wait_laserpose.SetEvent();
		//	gps.Location_avg();
		FILE *allout;
		allout = fopen("aa.txt","a+");
		for(int i=0;i<12;++i)
		{
			fprintf(allout," bbbbb===%d    \n" ,plan.distance_min[i]);
		}
		fprintf(allout," ======================   \n" );
		fclose(allout);


	}
	return 0;
}




  void CRecorder_ExampleDlg::OnBnClickedButton2()//��ͣ
  {
	  
 }
  //ȷ��λ��
   UINT ThreadComput_MotorTts(LPVOID lpParam)
{
	while(motertts_key)
	{
		//Sleep(200);
		WaitForSingleObject(wait_motortts,200);
	//	distance_l = (float)motor.encoder_l*100/1160/27;
	//	distance_r = (float)motor.encoder_r*100/1160/27;
		/*if(motertts_key1&&Info_robot.pointrox<7000&&Info_robot.pointrox>6300){
			motertts_key1=false;

			char* tts_motor="�������ǿ��Կ���һЩŮ������ʹ�õı���������˵����޼������޼���ĸ���������̣������磬�ŵص�ʱ������һ�̳��ϡ��̼���ݱ�ֲ����޼����������ս���У�������޼ɢ���ڵأ��������˵о�������Ƽ����㣬�Գ��͵о����ж���";
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
		}
		if(motertts_key2&&Info_robot.pointrox<5000&&Info_robot.pointrox>4200){
			motertts_key2=false;

			char* tts_motor="����������һ���Աߵ����Ǵ������Ǵ���һ�ֽ�������ͷϵ�ڳ���һ�˻������Ƴɵ�����������Աߵ���ʯ�������ʯ��ʹ�õġ������ʯ�ĳ���Ҳ֤���˽�������������ľ����Ѿ���ʹ����ʯ���ˡ�";
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
		}
		if(motertts_key3&&Info_robot.pointrox<2350&&Info_robot.pointrox>1800){
			motertts_key3=false;

			char* tts_motor="���������ͭ�����ƿ�������ƿ������װ�Ƶģ����Ҳ������顣�����ľ������кܶ�͹������ƣ����ǾͿ������ɵذ�����˩��ƿ�����档";
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
		}
		if(motertts_key4&&Info_robot.pointrox<-2300&&Info_robot.pointrox>-3300){
			motertts_key4=false;

			char* tts_motor="�������Ա�չ����������Ƕ䡣���Ƕ���ԭΪһ�ֱ��������δ����ɴ���ʱ������Ϊһ��������ΪȨ�����������׳ơ���ϡ�����������ε�Ӱ��Ҳ���������ֹ�������أ����Ƕ����һ�ֱ�����Ҳ��һ�����̡�";
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
		}
*/
/*		if(Info_robot.pointrox<9700&&Info_robot.pointrox>9500){
			motertts_key1=true;
			motertts_key2=true;
		}
*/		

	}
	return 0;
   }

//xhy����
UINT ThreadComput_Motorautowalk(LPVOID lpParam)
{

	while(moterautowalk_key)
	{
		WaitForSingleObject(wait_motortts,300);
	
		/*if(Info_robot.pointrox_stargazer<100)
		{
			motor.VectorMove(1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
		}
		else if(Info_robot.pianzhuan_stargazer>=10)
		{
			motor.VectorMove(200,1.0,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
		}
		else if(Info_robot.pianzhuan_stargazer<=10&&Info_robot.pointroy_stargazer<200)
		{
			motor.VectorMove(1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
		}*/
		if (plan.Range_to_go(plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy))
			{
				
				char* tts_motor=zhanpin[objectnow].contect1;
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
//			WaitForSingleObject(wait_motortts,100);
			Sleep(zhanpin[objectnow].time);
			
			if(zhanpin[objectnow].mode==2)
			{
				objectnowpos=objectnowpos+1;
			int nums_oba=0;
			for(;nums_oba<100;++nums_oba)
			{
				if(zhanpin[nums_oba].objectnum==objectnums[objectnowpos])
				{
					break;
				}
			}

			plan.target_x=zhanpin[nums_oba].objectnum_x;
			plan.target_z=zhanpin[nums_oba].objectnum_y;//xhyĿ���
			plan.speed_line_pos=0;
			plan.Desired_Angle_ob=zhanpin[nums_oba].direct;
			objectnow=nums_oba;
			isbegio=true;
			plan.SERVEMODE = 2;
			plan.CtrlMode=2;
			}
			else if(abs(plan.pianzhuan*180/PIf-plan.Desired_Angle_ob)<=20.0)
			{
				

				wait_motor_timer = 200*10000;
				plan.speed_line = 0;
				plan.speed_angle = 0;
				plan.speed_l = 0;
				plan.speed_r = 0;
//				edit_target->SetWindowText("����Ŀ��");
				plan.SERVEMODE = 4; //�л����ȴ�ģʽ
				motor.stop(); //ֹͣ
				Sleep(150);
				motor.stop(); //ֹͣ
				plan.speed_l = 0;
				plan.speed_r = 0;
			//	plate_wight = PlateData[2]; //��¼��ǰ��������
				waittimer = 0;
/////////////////////////////////////////////////xhytts
			
			objectnowpos=objectnowpos+1;
			int nums_oba=0;
			for(;nums_oba<100;++nums_oba)
			{
				if(zhanpin[nums_oba].objectnum==objectnums[objectnowpos])
				{
					break;
				}
			}

			plan.target_x=zhanpin[nums_oba].objectnum_x;
			plan.target_z=zhanpin[nums_oba].objectnum_y;//xhyĿ���
			plan.speed_line_pos=0;
			plan.Desired_Angle_ob=zhanpin[nums_oba].direct;
			objectnow=nums_oba;
			isbegio=true;
			plan.SERVEMODE = 2;
			plan.CtrlMode=2;
			FILE *alloutXXX;
		alloutXXX = fopen("alloutdaoda.txt","a+");
		fprintf(alloutXXX ," %d  %d  %d %d  %f %f  %f \n",objectnowpos,nums_oba,plan.target_x,plan.target_z);
		fclose(alloutXXX );

////////////////////////////////////////////////
				//���ƻ�����ת��Ŀ�����
			//	plan.Desired_Angle;
				//if (plan.Desired_Angle<=10)
				//{
				//	SetTimer(3,400,NULL); //����ȡ�ͼ��
				//	SetTimer(4,1000,NULL); //�����ȴ�ȡ�ͼ�ʱ
				//}
				//else
				//{
				//	motor.Velocity_control(0,plan.Desired_Angle*3.14/180);
				//}
					//SetTimer(3,400,NULL); //����ȡ�ͼ��
					//SetTimer(4,1000,NULL); //�����ȴ�ȡ�ͼ�ʱ
				
			}
			
		}
	}
	return 0;
}

   //ǰ��������ת��ת
 void CRecorder_ExampleDlg::OnBnClickedButton6()//����
{
	
	plan.speed_line = 0;
	plan.speed_angle = 0;
	plan.speed_l = 0;
	plan.speed_r = 0;
	// TODO: Add your control notification handler code here
	//	//////////////////////////////////zcs add start
		gridfastslam_key=true;
		wait_gridfastslam.SetEvent();



		///////////////////////////////////////zcs add end
		wait_motor_timer = 200;
		speedkey = 1;
		int nIndex = 0;
		CString strCBText;
		//��ʼָ������ģʽ,�˴�ֻ������������,��Ҫд��������
		//QueryPerformanceCounter(&start_t);
		//fprintf(file,"starttime:		%f    \n",1e3*start_t.QuadPart/freq.QuadPart);
		//nIndex = Combox_Target1.GetCurSel();
		//if (nIndex>=0)
		//{
		//	Combox_Target1.GetLBText( nIndex, strCBText);
		//	target1 = atoi(strCBText);
		//}
		//else
		//{
		//	nIndex = 0;
		//	target1 = 0;
		//}

		//nIndex = Combox_Target2.GetCurSel();
		//if (nIndex>=0)
		//{
		//	Combox_Target2.GetLBText( nIndex, strCBText);
		//	target2 = atoi(strCBText);
		//}
		//else
		//{
		//	nIndex = 0;
		//	target2 = 0;
		//}

		//nIndex = Combox_Target3.GetCurSel();
		//if (nIndex>=0)
		//{
		//	Combox_Target3.GetLBText( nIndex, strCBText);
		//	target3 = atoi(strCBText);
		//}
		//else
		//{
		//	nIndex = 0;
		//	target3 = 0;
		//}

		//if (target1 == 0 && target2 == 0 && target3 == 0)
		//{
		//	MessageBox("δ������ȷ��Ŀ���!");
		//}
		//else
		//{
		////	plate_wight = PlateData[2];

		//	//SystemSet.TableLocation_X[target1];
		//	//SystemSet.TableLocation_X[target1];
		//	//SystemSet.TableLocation_X[target1];
		//	if(target1!=0)
		//	{
		////		map_world.Map_update(SystemSet.TableLocation_X[target1],SystemSet.TableLocation_Y[target1],3);
		//		plan.target1_x = SystemSet.TableLocation_X[target1];
		//		plan.target1_z = SystemSet.TableLocation_Y[target1];
		//	}
		//	if(target2!=0)
		//	{
		////		map_world.Map_update(SystemSet.TableLocation_X[target2],SystemSet.TableLocation_Y[target2],3);
		//		plan.target2_x = SystemSet.TableLocation_X[target2];
		//		plan.target2_z = SystemSet.TableLocation_Y[target2];
		//	}
		//	if(target3!=0)
		//	{
		////		map_world.Map_update(SystemSet.TableLocation_X[target3],SystemSet.TableLocation_Y[target3],3);
		//		plan.target3_x = SystemSet.TableLocation_X[target3];
		//		plan.target3_z = SystemSet.TableLocation_Y[target3];
		//	}

		//	if (target1 != 0 )
		//	{
		//		plan.target_x = plan.target1_x;    
		//		plan.target_z = plan.target1_z;
		//		target_num = 1;
		//	}
		//	else if (target2 != 0 )
		//	{
		//		plan.target_x = plan.target2_x;    
		//		plan.target_z = plan.target2_z;
		//		target_num = 2;
		//	}
		//	else if (target3 != 0 )
		//	{
		//		plan.target_x = plan.target3_x;    
		//		plan.target_z = plan.target3_z;
		//		target_num = 3;
		//	}
		//	edit_target->SetWindowText("Ŀ��1");
		///////////////////////////////////////////////xhy��ʼ��
		int nums_ob=0;
		for(;nums_ob<100;++nums_ob)
		{
			if(zhanpin[nums_ob].objectnum==objectnums[0])
			{
				break;
			}
		}
			plan.target_x=zhanpin[nums_ob].objectnum_x;
			plan.target_z=zhanpin[nums_ob].objectnum_y;//xhyĿ���
			
			plan.speed_line_pos=0;
			plan.Desired_Angle_ob=zhanpin[nums_ob].direct;
			objectnow=nums_ob;
			objectnowpos=0;
			plan.SERVEMODE = 2;
			plan.CtrlMode=2;
			if (!motor_key)
			{
				motor_key = true;
				pThread_MototCtrl = AfxBeginThread(ThreaMotorCtrl,NULL); //��������߳�
			}
			

	hci_asr_recorder_cancel();
	hci_asr_recorder_release();

	PLAYER_CALLBACK_PARAM cb;
    cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	cb.pvStateChangeUsrParam = this;
	cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	cb.pvProgressChangeUsrParam = this;
	cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	cb.pvPlayerErrorUsrParam = this;

	PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	string initConfigtts = "initCapkeys=tts.cloud.wangjing";
    initConfigtts += ",dataPath=../../data" ;
	eReti = hci_tts_player_init( initConfigtts.c_str(), &cb );
	if (eReti != PLAYER_ERR_NONE)
	{
		hci_release();
		CString str;
		str.Format( "��������ʼ��ʧ��,������%d.", eReti);
		MessageBox( str );
		
	}



//	pThread_Motor_tts = AfxBeginThread(ThreadComput_MotorTts,NULL);

	pThread_Motor_autowalk= AfxBeginThread(ThreadComput_Motorautowalk,NULL);
		

}

   void CRecorder_ExampleDlg::OnBnClickedButton3()//����
   {
	   ishand=true;
	    motor.VectorMove(-1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
	   // TODO: �ڴ���ӿؼ�֪ͨ����������
   }
     void CRecorder_ExampleDlg::OnBnClickedButton1()//ǰ��
  {
	  ishand=true;
	  // TODO: Add your control notification handler code here
	  motor.VectorMove(1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
	 }

	 void CRecorder_ExampleDlg::OnBnClickedButton4()//��ת
	 {
		 ishand=true;
		 // TODO: Add your control notification handler code here
		 motor.VectorMove(200,2.0,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
	 }


	 void CRecorder_ExampleDlg::OnBnClickedButton5()//��ת
	 {
		 ishand=true;
		 // TODO: Add your control notification handler code here
		  motor.VectorMove(200,-2.0,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
	 }

//xhyvhf
UINT ThreadReadLaser_Data(LPVOID lpParam)
{
	FILE *allout;
	allout = fopen("allout2laser.txt","a+");
	while (lase_key)
	{
		m_cURG.GetDataByGD(0,768,1);
		WaitForSingleObject(m_cURG.wait_laser,INFINITE);

		//	m_cURG.GetDataByGD(0,768,1);//ǰ������������ɨ��Ƕȷ�Χ��384����ǰ�����ߣ�288��Ϊ90�ȷ�Χ�������һ�����������˽Ƕȷֱ��ʡ���ȡ�������������;
		//////////////////////////////////////////
		Info_laser_data.m_Laser_Data_Point=m_nValPoint_temp;

		m_Laser_Data_Point_PostPro=m_nValPoint_temp;
		//	pReadThread_postpro->m_Laser_Point=m_nValPoint_temp;

		key_laser = !m_cURG.key;
		for (int i=0;i<Info_laser_data.m_Laser_Data_Point;i++)
		{
			Info_laser_data.m_Laser_Data_Value[i]=m_cURG.m_distVal_temp_test[key_laser][i];
			m_laser_data_raw[i]=m_cURG.m_distVal_temp_test[key_laser][i];
			m_laser_data_postpro[i] = m_cURG.m_distVal_temp_test[key_laser][i];
			if(i%10==0)
			{
				fprintf(allout," 0000===%d   \n",m_laser_data_postpro[i]);
			}
		}

		//pReadThread_postpro->pm_median_filter();//�������ݵ���ֵ�˲�����
		//////////////////////////////////////////////////////////
		//	plan.MyFtestDanger();

		wait_data.SetEvent();
		m_cURG.wait_laser.ResetEvent();
		wait_laserpose.SetEvent();
	}
	return 0;
}

UINT ThreadDataExchange(LPVOID lpParam)
{
	while (dataexchange_key)
	{
		//WaitForSingleObject(waittime,INFINITE);
		//WaitForSingleObject(waittime,3000);
		WaitForSingleObject(wait_data,INFINITE);
		plan.speed_stated = speed_stated;
		for (int loop = 0;loop<3;loop++)
		{
			//plan.PlateData[loop] = PlateData[loop];
		}
		for (int loop = 0;loop<1000;loop++)
		{
			plan.m_laser_data_postpro[loop] = m_laser_data_postpro[loop];
			m_laser_data_postpro_vfh[loop] = m_laser_data_postpro[loop];
		}
//		plan.waittimer = waittimer;
		
		/*plan.pointrox = Info_robot.pointrox_corrected;
		plan.pointroy = Info_robot.pointroy_corrected;
		plan.pianzhuan = Info_robot.pianzhuan_corrected;*/

		plan.pointrox = Info_robot.pointrox;
		plan.pointroy = Info_robot.pointroy;
		plan.pianzhuan = Info_robot.pianzhuan;
		plan.MyFtestDanger();
		speed_stated_vfh = speed_stated;
		//wait_vfh.ResetEvent();
		wait_vfh.SetEvent();
		wait_data.ResetEvent();
	}
	return 0;
}

UINT ThreaVFH(LPVOID lpParam)
{
	plan.Init();
	algorithm.Init();
	plan.laser_position = false;
	while (vfh_key)
	{
		QueryPerformanceFrequency(&freq1);
		QueryPerformanceCounter(&start_t1);
		WaitForSingleObject(wait_vfh,INFINITE);
		
		Pathplan();
		////////////////////////////zcs add start
		wait_motor.SetEvent();
		//	wait_gridfastslam.SetEvent();
		/////////////////////////////zcs add end
		wait_vfh.ResetEvent();
		QueryPerformanceCounter(&stop_t1);
		exe_time1 = 1e3*(stop_t1.QuadPart-start_t1.QuadPart)/freq1.QuadPart;
		start_t1.QuadPart = stop_t1.QuadPart;
		//fprintf(file,"vfhtime:		%f    \n",exe_time1);
	}
	return 0;
}

void Pathplan()
{
	//pp.Init();
	//	read_target_position_and_settarget();
	//double mm_x=Info_robot.pointrox_corrected;//��λ�õĺ��׵�λת��������
	//double mm_y=Info_robot.pointroy_corrected;
	//double mm_angle=Info_robot.pianzhuan_corrected;

	double mm_x=Info_robot.pointrox;//��λ�õĺ��׵�λת��������
	double mm_y=Info_robot.pointroy;
	double mm_angle=Info_robot.pianzhuan;

	//////////////////////////////////////////////////////////////////////////hhh
	//��ƫת�ǻ���ʼ�ձ�����һ�������ڣ�0��2pi����
	while(mm_angle>=Info_robot.pi*2)
	{
		mm_angle -= Info_robot.pi*2;
	}
	while(mm_angle<0)
	{	
		mm_angle+= Info_robot.pi*2;
	}
	//////////////////////////////////////////////////////////////////////////

	plan.PlanPath_vfh(mm_x,mm_y,mm_angle*180/PI);
}



UINT ThreaMotorCtrl(LPVOID lpParam)
{

	FILE *allout;
	allout = fopen("allout4.txt","a+");

	FILE *alloutxhy;
	alloutxhy = fopen("alloutxhy.txt","a+");
	

	//fprintf(allout,"�Ƕȣ�%f   l:  %f  r:  %f  z:   %f   \n",(double)dtheta, distancedif_l, distancedif_r,distancedif_z);
	//fclose(allout);
	while(motor_key)
	{
	//	if (plan.danger)

		WaitForSingleObject(wait_motor,wait_motor_timer);
	//	else
	//		WaitForSingleObject(wait_motor,200);



		fprintf(allout,"%f    %f    %f    %f    \n",plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy);
		if (plan.CtrlMode == 1 || plan.CtrlMode == 3) 
		{
			if (CtrlMode_old != 3)
			{
			//	SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				//motor.gomotor(0,0);
				CtrlMode_old = 3;
				//Sleep(300);
			}
			
			//	plan.speed_line = 0;
			//	plan.speed_angle = 0;
			/*if (plan.speed_l>19)
			{
				plan.speed_r-=(plan.speed_l-19);
				plan.speed_l = 19;
			}
			if (plan.speed_r>19)
			{
				plan.speed_l-=(plan.speed_r-19);
				plan.speed_r = 19;
			}*/
			//if (plan.speed_l >= 0 && plan.speed_r >= 0)
			//{
			//	run_direction = 1;
			//}
			//else if (plan.speed_l < 0 && plan.speed_r < 0)
			//{
			//	run_direction = -1;
			//}
			//if (run_direction_old != run_direction)
			//{
			//	SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
			//	run_direction_old = run_direction;
			//	//Sleep(350);
			//}
			//if ((plan.speed_l-speed_l_old >10 || plan.speed_l-speed_l_old< -10) || (plan.speed_r-speed_r_old >10 || plan.speed_r-speed_r_old< -10))
			//{
			//	SpeedBuffer(plan.speed_l,plan.speed_r,&speed_l_old,&speed_r_old);
			//}
			/*else
			{
				if (plan.speed_l<0 && plan.speed_r<0)
				{
					if (plan.speed_l<-5)
					{
						plan.speed_l = -5;
					}
					if (plan.speed_r<-5)
					{
						plan.speed_r = -5;
					}
					
				}*/
			//	motor.gomotor(plan.speed_l,plan.speed_r);
			//}
			
			motor.gomotor(plan.speed_l,-plan.speed_r,2*(plan.speed_r - plan.speed_l));

			fprintf(allout,"%d    %d    \n",plan.speed_l,plan.speed_r);
			/*Sleep(100);
			motor.gomotor(plan.speed_l,plan.speed_r,0);
			Sleep(100);
			motor.gomotor(plan.speed_l,plan.speed_r,0);*/
		}
		else if(plan.CtrlMode == 2)
		{
			//plan.speed_l = 0;
			//plan.speed_r = 0;
			if (CtrlMode_old != 2)
			{
			//	SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				CtrlMode_old = 2;
				//Sleep(300);
			}
			if (plan.speed_line >=0)
			{
				run_direction = 1;
			}
			else if (plan.speed_line<0)
			{
				run_direction = -1;
			}
			
			if (run_direction_old != run_direction)
			{
				SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				run_direction_old = run_direction;
			//	Sleep(350);
			}
		//	motor.Velocity_control(plan.speed_line,plan.speed_angle);
			fprintf(allout," %d  %d %f  %f   line_speed��%f   angle_speed:  %f  Desired_Angle:  %f  pick:   %f   pianzhuan:  %f\n",plan.target1_x,plan.target1_z,Info_robot.pointrox,Info_robot.pointroy,plan.speed_line,plan.speed_angle,plan.Desired_Angle,pick,Info_robot.pianzhuan);
			if(!ishand)
			{
				motor.VectorMove(plan.speed_line*40,plan.speed_angle,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
			}
		//	motor.gomotor(plan.speed_l,-plan.speed_r,2*(plan.speed_r - plan.speed_l));
			plan.speed_l = motor.move_lsp;
			plan.speed_r = motor.move_rsp;
			speed_l_old = plan.speed_l;
			speed_r_old = plan.speed_r;

			//motor.Velocity_control(plan.speed_line,plan.speed_angle);
		}
		//wait_motor.ResetEvent();
	}
fclose(allout);
fclose(alloutxhy);
	return 0;
	
}

void SpeedBuffer(int speed_l, int speed_r,int *speed_l_old, int *speed_r_old)
{
	do 
	{
		if (!speedkey)
		{
			return;
		}
		if (*speed_r_old < speed_r)
		{
			*speed_r_old +=acceleration;
			if (*speed_r_old > speed_r)
			{
				*speed_r_old = speed_r;
			}
		}
		else if (*speed_r_old > speed_r)
		{
			*speed_r_old -=acceleration;
			if (*speed_r_old < speed_r)
			{
				*speed_r_old = speed_r;
			}
		}

		if (*speed_l_old < speed_l)
		{
			*speed_l_old +=acceleration;
			if (*speed_l_old > speed_l)
			{
				*speed_l_old = speed_l;
			}
		}
		else if (*speed_l_old > speed_l)
		{
			*speed_l_old -=acceleration;
			if (*speed_l_old < speed_l)
			{
				*speed_l_old = speed_l;
			}
		}
		if (*speed_l_old<-5 && *speed_r_old<-5)
		{
			if (*speed_l_old <-5)
			{
				*speed_l_old = -5;
			}
			if (*speed_r_old < -5)
			{
				*speed_r_old = -5;
			}
			motor.gomotor((*speed_l_old)*(-1),(*speed_r_old)*(-1),2*((*speed_r_old)*(-1)-(*speed_l_old)*(-1)));
			return;
		}
	//	motor.gomotor(*speed_l_old,*speed_r_old,0);
		motor.gomotor((*speed_l_old)*(-1),(*speed_r_old)*(-1),2*((*speed_r_old)*(-1)-(*speed_l_old)*(-1)));
		Sleep(50);

	} while ((*speed_r_old != speed_r) || (*speed_l_old != speed_l));
}

void CRecorder_ExampleDlg::OnTimer(UINT_PTR nIDEvent)
{

	switch (nIDEvent)
	{
		case 2://�ж��Ƿ񵽴�Ŀ���(200ms)
		{
		//	if (plan.Range_to_go(plan.target_x,plan.target_z,Info_robot.pointrox_corrected,Info_robot.pointroy_corrected))
			if (plan.Range_to_go(plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy))
			{
				KillTimer(2);
			//	KillTimer(1);
				wait_motor_timer = 200*10000;
				plan.speed_line = 0;
				plan.speed_angle = 0;
				plan.speed_l = 0;
				plan.speed_r = 0;
//				edit_target->SetWindowText("����Ŀ��");
				plan.SERVEMODE = 4; //�л����ȴ�ģʽ
				motor.stop(); //ֹͣ
				Sleep(150);
				motor.stop(); //ֹͣ
				plan.speed_l = 0;
				plan.speed_r = 0;
			//	plate_wight = PlateData[2]; //��¼��ǰ��������
				waittimer = 0;
				//���ƻ�����ת��Ŀ�����
			//	plan.Desired_Angle;
				//if (plan.Desired_Angle<=10)
				//{
				//	SetTimer(3,400,NULL); //����ȡ�ͼ��
				//	SetTimer(4,1000,NULL); //�����ȴ�ȡ�ͼ�ʱ
				//}
				//else
				//{
				//	motor.Velocity_control(0,plan.Desired_Angle*3.14/180);
				//}
					//SetTimer(3,400,NULL); //����ȡ�ͼ��
					//SetTimer(4,1000,NULL); //�����ȴ�ȡ�ͼ�ʱ
				
			}
			break;
			default:
				break;
		}
		break;
		
	}
}

void CRecorder_ExampleDlg::OnBnClickedButton7()//��������
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	char* tts_motor=zhanpin[objectnumshand[objectnowhand]].contect1;
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
			++objectnowhand;
}


void CRecorder_ExampleDlg::OnBnClickedButton8()//�ֶ��ٿ�
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	hci_asr_recorder_cancel();
	hci_asr_recorder_release();

	PLAYER_CALLBACK_PARAM cb;
    cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	cb.pvStateChangeUsrParam = this;
	cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	cb.pvProgressChangeUsrParam = this;
	cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	cb.pvPlayerErrorUsrParam = this;

	PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	string initConfigtts = "initCapkeys=tts.cloud.wangjing";
    initConfigtts += ",dataPath=../../data" ;
	eReti = hci_tts_player_init( initConfigtts.c_str(), &cb );
	if (eReti != PLAYER_ERR_NONE)
	{
		hci_release();
		CString str;
		str.Format( "��������ʼ��ʧ��,������%d.", eReti);
		MessageBox( str );
		
	}
}


void CRecorder_ExampleDlg::OnEnChangeEditWordlist()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialog::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
}

//�ɲ���
HBRUSH CRecorder_ExampleDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);
	pDC->SetBkMode(OPAQUE);
	pDC->SetTextColor(RGB(74,37,15));
	pDC->SetBkColor(RGB(178,136,80));
	hbr=CreateSolidBrush(RGB(178,136,80));
	// TODO:  �ڴ˸��� DC ���κ�����

	// TODO:  ���Ĭ�ϵĲ������軭�ʣ��򷵻���һ������
	return hbr;
}
/*
void CRecorder_ExampleDlg::DrawItem(int nIDCtl,LPDRAWITEMSTRUCT lpDrawItemStruct)

{
	
	if( nIDCtl==IDC_BTN_START_RECORD || nIDCtl==IDC_MFCBUTTON2 || nIDCtl==IDC_MFCBUTTON7 || nIDCtl==IDC_MFCBUTTON1 || nIDCtl==IDC_BTN_CANCEL_RECORD || nIDCtl== IDC_MFCBUTTON6)         //checking for the button 
	{
	CDC dc;
    dc.Attach(lpDrawItemStruct->hDC);//�õ����Ƶ��豸����CDC


   //�õ�Button������,����Ĳ�����:1,�ȵõ�����Դ��༭�İ�ť������,

   //Ȼ�󽫴��������»��Ƶ���ť��,

    //ͬʱ�������ֵı���ɫ��Ϊ͸��,����,��ť�Ͻ�����ʾ����

    const int bufSize = 512;

    TCHAR buffer[bufSize];

    GetWindowText(buffer, bufSize);

   int size=strlen(buffer);//�õ�����

   DrawText(lpDrawItemStruct->hDC,buffer,size,&lpDrawItemStruct->rcItem,DT_CENTER|DT_VCENTER|DT_SINGLELINE|DT_TABSTOP);//��������

   SetBkMode(lpDrawItemStruct->hDC,TRANSPARENT);//͸��

   if (lpDrawItemStruct->itemState&ODS_SELECTED)//�����°�ťʱ�Ĵ���

   {////�ػ���������
		
         CBrush brush(m_DownColor);

          dc.FillRect(&(lpDrawItemStruct->rcItem),&brush);//���û�ˢbrush�������ο�

         //��Ϊ����������ػ�,��������ҲҪ�ػ�

         DrawText(lpDrawItemStruct->hDC,buffer,size,&lpDrawItemStruct->rcItem,DT_CENTER|DT_VCENTER|DT_SINGLELINE|DT_TABSTOP);

          SetBkMode(lpDrawItemStruct->hDC,TRANSPARENT);

    }

  else//����ť���������ߵ���ʱ

    {

           CBrush brush(m_UpColor);

            dc.FillRect(&(lpDrawItemStruct->rcItem),&brush);//

            DrawText(lpDrawItemStruct->hDC,buffer,size,&lpDrawItemStruct->rcItem,DT_CENTER|DT_VCENTER|DT_SINGLELINE|DT_TABSTOP);

            SetBkMode(lpDrawItemStruct->hDC,TRANSPARENT);

     }

    if ((lpDrawItemStruct->itemState&ODS_SELECTED)&&(lpDrawItemStruct->itemAction &(ODA_SELECT|ODA_DRAWENTIRE)))

     {//ѡ���˱��ؼ�,�����߿�

               COLORREF fc=RGB(255-GetRValue(m_UpColor),255-GetGValue(m_UpColor),255-GetBValue(m_UpColor));

             CBrush brush(fc);

            dc.FrameRect(&(lpDrawItemStruct->rcItem),&brush);//�û�ˢbrush�������α߿�

       }

     if (!(lpDrawItemStruct->itemState &ODS_SELECTED) &&(lpDrawItemStruct->itemAction & ODA_SELECT))

         {

          CBrush brush(m_UpColor); //���Ƶ�ѡ��״̬����,ȥ���߿�

         dc.FrameRect(&lpDrawItemStruct->rcItem,&brush);//}

        dc.Detach();

	 }}
	CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);}*/

void CRecorder_ExampleDlg::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ

	CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);

	/*if( nIDCtl==IDC_BTN_START_RECORD || nIDCtl==IDC_MFCCOLORBUTTON5 || nIDCtl==IDC_MFCBUTTON7 || nIDCtl==IDC_BUTTON8 || nIDCtl==IDC_BTN_CANCEL_RECORD || nIDCtl== IDC_MFCBUTTON6)         //checking for the button 
    {
    CDC dc;
    RECT rect;
    dc.Attach(lpDrawItemStruct ->hDC);   // Get the Button DC to CDC
    
    rect = lpDrawItemStruct->rcItem;     //Store the Button rect to our local rect.
    

    dc.Draw3dRect(&rect,RGB(255,255,255),RGB(0,0,0)); 
	
	

  
    dc.FillSolidRect(&rect,RGB(178,136,80));//Here you can define the required color to appear on the Button.

    UINT state=lpDrawItemStruct->itemState; //This defines the state of the Push button either pressed or not. 

    if((state & ODS_SELECTED))
    {
        dc.DrawEdge(&rect,EDGE_SUNKEN,BF_RECT);
	}
    else
    {
        dc.DrawEdge(&rect,EDGE_RAISED,BF_RECT);
    }

    dc.SetBkColor(RGB(178,136,80));   //Setting the Text Background color
    dc.SetTextColor(RGB(74,37,15));     //Setting the Text Color
	

    TCHAR buffer[MAX_PATH];           //To store the Caption of the button.
    ZeroMemory(buffer,MAX_PATH );     //Intializing the buffer to zero
        ::GetWindowText(lpDrawItemStruct->hwndItem,buffer,MAX_PATH); //Get the Caption of Button Window 
    
    dc.DrawText(buffer,&rect,DT_CENTER|DT_VCENTER|DT_SINGLELINE);//Redraw the Caption of Button Window 
    
    dc.Detach(); // Detach the Button DC
    }                
    CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);
	
	
	*/

	
}



 void CRecorder_ExampleDlg::OnSize(UINT nType, int cx, int cy)
	 {
		 CDialog::OnSize(nType, cx, cy);
		 CDialog::OnSize(nType, cx, cy);


		 
		if(nType==1) return; //��С����ʲô������ 
		CWnd *pWnd; 
		pWnd = GetDlgItem(IDD_RECORDER_EXAMPLE_DIALOG); //��ȡ�ؼ����
		ChangeSize(pWnd,cx,cy); //����changesize()����
		pWnd = GetDlgItem(IDOK ); //��ȡ�ؼ����
		ChangeSize(pWnd,cx,cy);//����changesize()����
		pWnd = GetDlgItem(IDC_MFCBUTTON7);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON6);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON2);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON1);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON3);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON4);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON5);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON9);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON10);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON13);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON14);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON15);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON16);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_MFCBUTTON17);
		ChangeSize(pWnd,cx,cy);
		pWnd = GetDlgItem(IDC_STATIC4);
		ChangeSize(pWnd,cx,cy);

		

		//ChangeSize(pWnd,cx,cy)��һ���Զ���ĺ�������Ҫ�����protect�����н����������afx_msg void ChangeSize(CWnd * pWnd, int cx, int cy); 
		pWnd = GetDlgItem(IDC_EDIT_STATUS); 
		ChangeSize(pWnd,cx,cy);
		GetClientRect(&m_rect); //���仯��ĶԻ�������Ϊ�ɴ�С
		}
void  CRecorder_ExampleDlg::ChangeSize(CWnd * pWnd, int cx, int cy)
		{
		if (pWnd)
		{
		CRect rect; 
		pWnd->GetWindowRect(&rect); //��ȡ�ؼ��仯ǰ�Ĵ�С
		ScreenToClient(&rect);//���ؼ���Сת��Ϊ�ڶԻ����е��������� 
		rect.left=rect.left*cx/m_rect.Width();//�����ؼ���С ��cx/m_rect.Width()Ϊ�Ի����ں���ı仯����
		rect.right=rect.right*cx/m_rect.Width(); //cx�洢���Ǳ仯��Ŀ�ȣ�cy�洢���Ǳ仯��ĸ߶�
		rect.top=rect.top*cy/m_rect.Height(); //m_rect.height()��ʾ���Ǳ仯ǰ������ĸ߶�
		rect.bottom=rect.bottom*cy/m_rect.Height();
		pWnd->MoveWindow(rect);//���ÿؼ���С
		}
		}
		




void CRecorder_ExampleDlg::OnStnClickedStatic1()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
}


void CRecorder_ExampleDlg::OnBnClickedButton9()//��������
{
	// TODO: Add your control notification handler code here
	CString strErrorMessage;
	hci_tts_player_stop();
	hci_tts_player_release();

	RECORDER_ERR_CODE eRetasr = RECORDER_ERR_UNKNOWN;
	RECORDER_CALLBACK_PARAM call_back;
	memset( &call_back, 0, sizeof(RECORDER_CALLBACK_PARAM) );
	call_back.pvStateChangeUsrParam		= this;
	call_back.pvRecogFinishUsrParam		= this;
	call_back.pvErrorUsrParam			= this;
	call_back.pvRecordingUsrParam		= this;
	call_back.pvRecogProcessParam		= this;
    call_back.pfnStateChange	= CRecorder_ExampleDlg::RecordEventChange;
	call_back.pfnRecogFinish	= CRecorder_ExampleDlg::RecorderRecogFinish;
	call_back.pfnError			= CRecorder_ExampleDlg::RecorderErr;
	call_back.pfnRecording		= CRecorder_ExampleDlg::RecorderRecordingCallback;
	call_back.pfnRecogProcess   = CRecorder_ExampleDlg::RecorderRecogProcess;

	string initConfig = "initCapkeys=asr.cloud.dialog";	
	initConfig        += ",dataPath=../../data";
	//string initConfig = "dataPath=" + account_info->data_path();
	//initConfig      += ",encode=speex";
	//initConfig		+= ",initCapkeys=asr.local.grammar";			      //��ʼ����������

	eRetasr = hci_asr_recorder_init( initConfig.c_str(), &call_back);
	if (eRetasr != RECORDER_ERR_NONE)
	{
		hci_release();
		strErrorMessage.Format( "¼������ʼ��ʧ��,������%d", eRetasr);
		MessageBox( strErrorMessage );
		return ;
	}

	RECORDER_ERR_CODE eRet = RECORDER_ERR_NONE;
	
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );	
	
	// ���״̬��¼
	
	SetDlgItemText(IDC_EDIT1, "" );

    AccountInfo *account_info = AccountInfo::GetInstance();
    string startConfig = "";
	if (!IsDlgButtonChecked( IDC_ONLY_RECORDING ))
    {
     	startConfig += "capkey=" + account_info->cap_key();
	}
	startConfig += ",audioformat=pcm16k16bit";
	//startConfig += ",domain=qwdz,intention=qwmap;music,needcontent=no";
	//startConfig     += ",realTime=rt";
    if (IsDlgButtonChecked( IDC_CONTINUE ))
    {
        startConfig += ",continuous=yes";
    }
	if ( m_RecogMode == kRecogModeGrammar )
	{
		char chTmp[32] = {0};
		sprintf(chTmp,",grammarid=%d",m_GrammarId);
		startConfig += chTmp; 
	}

	if ( m_RecogMode == kRecogModeDialog )
	{
		startConfig +=",intention=weather;joke;story;baike;calendar;translation;news"; 
	}

	eRet = hci_asr_recorder_start(startConfig.c_str(),"");
	if (RECORDER_ERR_NONE != eRet)
	{
		CString strErrMessage;
		strErrMessage.Format( "��ʼ¼��ʧ��,������%d", eRet );
		MessageBox( strErrMessage );
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		return;
	}

	pThread_speak_autotts= AfxBeginThread(ThreadComput_speakautotts,NULL);
}

 UINT ThreadComput_speakautotts(LPVOID lpParam)
{
	while(autotts_key)
	{
		if(isautotts)
		{
			isautotts=false;
//			CRecorder_ExampleDlg dig;
//			dig.OnBnClickedBtnCancelRecord();
			if(p_CR!=NULL)
			{
				p_CR->OnBnClickedBtnCancelRecord();

				if(autotts_time_sleep<10)
				{
					autotts_time_sleep=10;
				}
				Sleep(autotts_time_sleep*200);

				p_CR->OnBnClickedBtnStartRecord();

			}
		}
	}
	return 0;
 }

 void CRecorder_ExampleDlg::OnBnClickedMfcbutton1()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
 }


 void CRecorder_ExampleDlg::OnBnClickedMfcbutton3()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
 }


 void CRecorder_ExampleDlg::OnBnClickedMfcbutton5()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
 }





 void CRecorder_ExampleDlg::OnBnClickedMfcbutton6()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
	 zhanting=!zhanting;
	  ishand=false;
	  if(zhanting==true)
	  {
		  plan.SERVEMODE = 4;
//		  motor_key=false;
	  }
	  else
	  {
//		  motor_key=true;
		  plan.SERVEMODE = 2;
	  }
	  
/*
	 // TODO: �ڴ���ӿؼ�֪ͨ����������  
    MCI_OPEN_PARMS open = {0};//���岢��ʼ���ṹ��  
    char szErr[100];//���屣���������  
  
    open.lpstrElementName = "d:\\a.wav";//ָ�������ļ�·��  
    open.lpstrDeviceType = "mpegvideo";//ָ�������豸  
  
    DWORD err;//���������Ϣ  
    err = mciSendCommand(0,MCI_OPEN,MCI_OPEN_TYPE|MCI_OPEN_ELEMENT|MCI_WAIT,(DWORD)(LPVOID)&open);//��ʼ����Ƶ�豸  
    if (err == 0)  
	{  
        MCI_PLAY_PARMS play;  
        play.dwFrom = 0;  
        play.dwCallback = NULL;  
  
        mciSendCommand(open.wDeviceID,MCI_PLAY,0,(DWORD)&play);  
    }  
    else  
    {  
        mciGetErrorString(err,(LPSTR)szErr,100);  
        MessageBox(szErr);  
    }  
*/
//	  PlaySound("d:\\a.wav",NULL,SND_FILENAME|SND_ASYNC);

	//hci_asr_recorder_cancel();
	//hci_asr_recorder_release();

	//PLAYER_CALLBACK_PARAM cb;
 //   cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	//cb.pvStateChangeUsrParam = this;
	//cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	//cb.pvProgressChangeUsrParam = this;
	//cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	//cb.pvPlayerErrorUsrParam = this;

	//PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	//string initConfigtts = "initCapkeys=tts.cloud.wangjing";
 //   initConfigtts += ",dataPath=../../data" ;
	//eReti = hci_tts_player_init( initConfigtts.c_str(), &cb );
	//if (eReti != PLAYER_ERR_NONE)
	//{
	//	hci_release();
	//	CString str;
	//	str.Format( "��������ʼ��ʧ��,������%d.", eReti);
	//	MessageBox( str );
	//	
	//}



	//pThread_Motor_tts = AfxBeginThread(ThreadComput_MotorTts,NULL);

	//pThread_Motor_autowalk= AfxBeginThread(ThreadComput_Motorautowalk,NULL);

/*

	string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
	char* tts_motor="����˭";
	unsigned char* pszUTF8 = NULL;
    HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

	PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
	if(eRetk==PLAYER_ERR_NONE){
		int a=5;

	}
*/
 }


 void CRecorder_ExampleDlg::OnBnClickedMfcbutton7()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
	 if(p_CR!=NULL)
	{
		p_CR->OnBnClickedButton6();
	}
 }



// INT_PTR CRecorder_ExampleDlg::DoModal()
// {
//	 // TODO: �ڴ����ר�ô����/����û���
//
//	 return CDialog::DoModal();
// }




 


 void CRecorder_ExampleDlg::OnBnClickedMfcbutton2()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
	CVoice dlg;
	
	dlg.DoModal();
 }


 void CRecorder_ExampleDlg::OnBnClickedButton11()
 {
	 hci_asr_recorder_cancel();
	hci_asr_recorder_release();

	PLAYER_CALLBACK_PARAM cb;
    cb.pfnStateChange = CRecorder_ExampleDlg::CB_EventChange;
	cb.pvStateChangeUsrParam = this;
	cb.pfnProgressChange = CRecorder_ExampleDlg::CB_ProgressChange;
	cb.pvProgressChangeUsrParam = this;
	cb.pfnPlayerError = CRecorder_ExampleDlg::CB_SdkErr;
	cb.pvPlayerErrorUsrParam = this;

	PLAYER_ERR_CODE eReti = PLAYER_ERR_NONE;
	string initConfigtts = "initCapkeys=tts.cloud.wangjing";
    initConfigtts += ",dataPath=../../data" ;
			eReti = hci_tts_player_init( initConfigtts.c_str(), &cb );
	/*if (eReti != PLAYER_ERR_NONE)
	{
		hci_release();
		CString str;
		str.Format( "��������ʼ��ʧ��,������%d.", eReti);
		MessageBox( str );
		
	}*/
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
	char* p;
	CString str1;
	CString strTemp;
	p_DR->GetDlgItem(IDC_COMBO1)->GetWindowText(str1);
	if(str1==("��������")){
		strTemp.Format(_T("û��Ŷ"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("�˼��޳�")){
	
	strTemp.Format(_T("��������"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	 char* tts_motor = p;
	unsigned char* pszUTF8 = NULL;
	HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

	string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
	PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
	hci_release();		

 }


 void CRecorder_ExampleDlg::OnBnClickedButton12()
 {
	 // TODO: �ڴ���ӿؼ�֪ͨ����������
	 CString strCaption = "";
	GetDlgItemText( IDC_BUTTON12, strCaption );
	if( strCaption == "��ͣ" )
	{
		PLAYER_ERR_CODE eRet = hci_tts_player_pause();
		if( eRet != PLAYER_ERR_NONE )
		{
			CString str;
			str.Format( "��ͣ����ʧ��,������%d.", eRet);
			MessageBox( str );
			return;
		}
		SetDlgItemText( IDC_BUTTON12, "����" );
	}
	else
	{
		PLAYER_ERR_CODE eRet = hci_tts_player_resume();
		if( eRet != PLAYER_ERR_NONE )
		{
			CString str;
			str.Format( "��������ʧ��,������%d.", eRet );
			MessageBox( str );
			return;
		}
		SetDlgItemText(IDC_BUTTON12, "��ͣ" );
	}
 }
