#include "stdafx.h"
#include "ASR_Recorder_Example.h"
#include "ASR_Recorder_ExampleDlg.h"
#include "hci_asr_recorder.h"
#include "common/CommonTool.h"
#include "common/AccountInfo.h"
#include "hci_tts.h"
#include "hci_tts_player.h"
#include "hci_micarray.h"
#include "UPURG.h" //激光
#include "Plan_Path_VFH.h"
#include "vfh_algorithm.h"
#include "Voice.h"
#include "Comm_data_motor3.h"
#include "Comm_data_star.h" //红外标签定位模块
#pragma comment(lib,"Comm_data_star.lib") //视觉定位
#include "cJSON.h"
#include <string>
#include "mmsystem.h"  
using std::string;
#pragma comment(lib,"Winmm.lib")  
#define COMM_MOTOR 3 //底部电机串口号
#define COMM_STAR 4//星标定位串口
#define COMM_LASER 5 //激光传感器串口号
CVoice* dlg;
CMotor motor;
CEvent wait_motordata;
CEvent wait_motortts;
UINT ThreadComput_MotorData(LPVOID lpParam);
UINT ThreadComput_MotorTts(LPVOID lpParam);
CWinThread* pThread_Motor_Comput;////电机数据接收线程
CWinThread* pThread_Motor_tts;//电机tts线程
CFont *m_pFont;//创建新的字体  
CWinThread* pThread_Motor_autowalk;//电机漫游线程
bool moterautowalk_key= true;//开始漫游
UINT ThreadComput_Motorautowalk(LPVOID lpParam);
float distance_l = 0.0; //左轮距离
float distanceold_l = 0.0; //左轮上一次距离
float distance_r = 0.0; //右轮距离
float distanceold_r = 0.0; //右轮上一次距离
float distance_z = 0.0; //右轮距离
float distanceold_z = 0.0; //右轮上一次距离
float distancedif_l = 0.0; //左轮差值
float distancedif_r = 0.0; //右轮差值
float distancedif_z = 0.0; //
bool moter_key = true;//电机数据接收线程控制
bool motertts_key= true;
bool motertts_key1=true;//tts线程测试1
bool motertts_key2= true;//tts线程测试2
bool motertts_key3=true;//tts线程测试1
bool motertts_key4= true;//tts线程测试2
struct robotinfo Info_robot = {
	Info_robot.pi=3.141592654,
	Info_robot.Drobot = 400,
	Info_robot.zuolunfangxiang = 0,
	Info_robot.youlunfangxiang = 0,
	Info_robot.zuolunjuli = 0,
	Info_robot.youlunjuli = 0,
	Info_robot.pianzhuan = 0,//Info_robot.pi/2,
	Info_robot.pointrox = 10000,
	Info_robot.pointroy = 10000,
	Info_robot.pian = 0.00001,
	Info_robot.scene_length=20000,
	Info_robot.scene_width=20000,
	Info_robot.pianzhuan_stargazer=0,
	Info_robot.pointrox_stargazer=0,
	Info_robot.pointroy_stargazer=0
};
Cstar StarGazer;  //视觉定位控制类
struct StarMark{
	int markID;
	float mark_angle;
	float mark_x;
	float mark_y;
};
struct StarMark MARK[100]={ //LED定位标签数组
	//每块边长455
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
#define Max_Laser_Data_Point 768
struct threadInfo_laser_data{
	double 	m_Laser_Data_Value[Max_Laser_Data_Point];
	int	m_Laser_Data_Point;
};
//激光测距器数据读取结构参数
struct threadInfo_laser_data_postpro{
	double 	m_Laser_Data_Value_PostPro[Max_Laser_Data_Point];
	int	m_Laser_Data_Point_PostPro;
	double rx;
	double ry;   //robot odometry pos
	double  th;   //robot orientation 
	double  x[10][769];//[cm]
	double  y[10][769];//[cm]
	int bad[10][769];// 0 if OK
	int seg[10][769];
};
float m_nDlgWidth,m_nDlgHeight,m_nWidth,m_nHeight,m_Multiple_width,m_Mutiple_heith;
bool change_flag;
CUPURG m_cURG; //激光串口控制类
int m_laser_data_raw[1000];
int m_laser_data_postpro[1000];
int m_Laser_Data_Point_PostPro;
bool lase_key = true;
bool key_laser = false; //激光数据存储位置开关
CEvent wait_data;
CEvent wait_laserpose;
CEvent wait_vfh;
CEvent wait_motor;
CEvent wait_gridfastslam;
extern int speed_stated=20; //机器人速度
extern int waittimer=0;
extern int speed_stated_vfh;
extern double vfh_Scene_scale_x;
extern double vfh_Scene_scale_y;
extern double Obstacle_Distance_init;/////////////////////////初始避障距离设定
extern double delt_Obstacle_Distance_init;
extern double free_Obstacle_Distance_init;
extern int m_laser_data_postpro_vfh[1000]; //vfh中激光数据
CWinThread* pThread_DataExchange;  //vfh数据交换线程
CWinThread* pThread_VFHStart; //vfh启动
CWinThread* pThread_Read_Laser;  //激光数据读线程
threadInfo_laser_data Info_laser_data;
CWinThread* pReadThread_Laser_Pose_Compute; //激光位姿计算线程
UINT ThreadReadLaser_Data(LPVOID lpParam);
UINT ThreadDataExchange(LPVOID lpParam); //数据交换线程
UINT ThreaVFH(LPVOID lpParam);
CPlan_Path_VFH plan;
VFH_Algorithm algorithm;
bool dataexchange_key = false;
bool vfh_key = false;
bool motor_key = false;
LARGE_INTEGER freq1;
LARGE_INTEGER start_t1, stop_t1;  
double exe_time1;  
//#pragma comment(lib,"Plan_Path_VFH.lib") //引入路径规划库
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
struct object{
	int objectnum;
	float objectnum_x;
	float objectnum_y;
	float direct;
	float time;
	int mode;//1:到达转向(展品点)2:到达播放语音(语音点)3:即将到达转向(引导点)
	char* contect1;
	char* contect2;
	char* contect3;
};
struct object zhanpin[100]={
	zhanpin[0].objectnum=0,
	zhanpin[0].objectnum_x=0,
	zhanpin[0].objectnum_y=0,
	zhanpin[0].direct=0,
	zhanpin[0].time=10000,
	zhanpin[0].mode=1,
	zhanpin[0].contect1="这是终点",
	zhanpin[0].contect2="这是第一个展品",
	zhanpin[0].contect3="这是第一个展品",

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
	zhanpin[3].direct=225,//引导点
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
	zhanpin[6].contect1="这是我的姐姐小梅，它是不是比我苗条呀？小灵就送到这里啦，后面还有更精彩的展示，就不一一介绍啦，欢迎大家来哈尔滨工业大学做客，再见啦。",
	zhanpin[6].contect2="这是第三个展品",
	zhanpin[6].contect3="这是第三个展品",

	zhanpin[7].objectnum=50,
	zhanpin[7].objectnum_x=400,
	zhanpin[7].objectnum_y=500,
	zhanpin[7].direct=180,//语音朗读点
	zhanpin[7].time=0,
	zhanpin[7].mode=2,
	zhanpin[7].contect1="现在我们左前方的是文化和旅游部的18家重点实验室，他们是国家文化科技创新体系的重要组成部分，是凝聚和培养优秀文化科技人才，组织文化科技创新和开展学术交流的重要平台。",
	zhanpin[7].contect2="",
	zhanpin[7].contect3="",

	zhanpin[8].objectnum=51,
	zhanpin[8].objectnum_x=300,
	zhanpin[8].objectnum_y=550,
	zhanpin[8].direct=90,//语音朗读点
	zhanpin[8].time=0,
	zhanpin[8].mode=2,
	zhanpin[8].contect1="好的，下面请大家跟我来，前方是古籍保护科技重点实验室，它依托于国家图书馆，带来的是自主研发的图书整本批量脱酸技术，请大家欣赏经过脱酸处理后的图书。",
	zhanpin[8].contect2="",
	zhanpin[8].contect3="",

	zhanpin[9].objectnum=52,
	zhanpin[9].objectnum_x=150,
	zhanpin[9].objectnum_y=600,
	zhanpin[9].direct=180,
	zhanpin[9].time=0,
	zhanpin[9].mode=2,
	zhanpin[9].contect1="好啦，现在我们来参观下一个展位，丝绸文化传承与产品设计数字化技术重点实验室，它依托于浙江理工大学，这里展示的是丝绸图案及面料数字化设计，请大家欣赏。",
	zhanpin[9].contect2="",
	zhanpin[9].contect3="",

	zhanpin[10].objectnum=53,
	zhanpin[10].objectnum_x=75,
	zhanpin[10].objectnum_y=520,
	zhanpin[10].direct=225,
	zhanpin[10].time=0,
	zhanpin[10].mode=2,
	zhanpin[10].contect1="好的，我们继续，现在大家可以看前方的书画保护重点实验室，依托于故宫博物院，古书画装裱及修复技艺是我国国家级非遗传承项目，中国书画艺术在世界艺术之林享有极高声誉。现在您看到的展示，正是装裱修复技艺延续千年流传下来的基本工艺。",
	zhanpin[10].contect2="",
	zhanpin[10].contect3="",

	zhanpin[11].objectnum=7,
	zhanpin[11].objectnum_x=500,
	zhanpin[11].objectnum_y=500,
	zhanpin[11].direct=45,
	zhanpin[11].time=42000,
	zhanpin[11].mode=1,
	zhanpin[11].contect1="公元十二世纪初，继渤海国之后，靺鞨后裔女真人再度兴起，定居在阿什河畔的女真完颜部，在首领阿骨打的带领下统一了女真各部,并于1115年建立金朝，定都上京会宁府（今黑龙江省哈尔滨市阿城区白城），成为白山黑水的主宰。进而灭辽和北宋，统治中华大地半壁江山达百余年。金朝最强盛时所辖疆域北至外兴安岭，东北至鄂霍次克海及日本海，东南抵鸭绿江与高丽为邻，西达陕西西北地域与西夏交界，南与南宋以淮河为界，肃慎族系首次入主中原。",
	zhanpin[11].contect2="",
	zhanpin[11].contect3="",

	zhanpin[12].objectnum=1,
	zhanpin[12].objectnum_x=500,
	zhanpin[12].objectnum_y=500,
	zhanpin[12].direct=0,
	zhanpin[12].time=10000,
	zhanpin[12].mode=1,
	zhanpin[12].contect1="大家好，我是刚刚上岗的解说员小灵，来自哈尔滨工业大学，互动媒体设计与装备服务创新，文化和旅游部重点实验室。我是可以提供引导，讲解和互动等服务的智能机器人，欢迎各位观摩重点实验室展览，现在，请大家随我参观。",
	zhanpin[12].contect2="这是第一个展品",
	zhanpin[12].contect3="这是第一个展品",

	zhanpin[13].objectnum=81,
	zhanpin[13].objectnum_x=500,
	zhanpin[13].objectnum_y=450,
	zhanpin[13].direct=180,
	zhanpin[13].time=30000,
	zhanpin[13].mode=1,
	zhanpin[13].contect1="这件六耳大铜锅，为金代典型炊具，这种锅在辽代已较流行，金代沿袭并有所发展。在游猎或行军打仗中，需要临时安营扎寨，埋锅造饭，众多的人吃饭需用大锅。而女真人喜欢的食物中，羊、牛、马占有很大比重，他们又习惯煮食大块肉，这种六耳锅恰巧适应了当时的需要。这件大铜锅完整无损，是黑龙江省目前出土的铜锅中较大的一口，对于研究金代冶炼技术的发展有重大价值。请跟我来",
	zhanpin[13].contect2="这是第一个展品",
	zhanpin[13].contect3="这是第一个展品",

	zhanpin[14].objectnum=82,
	zhanpin[14].objectnum_x=500,
	zhanpin[14].objectnum_y=300,
	zhanpin[14].direct=180,
	zhanpin[14].time=0,
	zhanpin[14].mode=1,
	zhanpin[14].contect1="",
	zhanpin[14].contect2="这是第一个展品",
	zhanpin[14].contect3="这是第一个展品",
	
	zhanpin[15].objectnum=83,
	zhanpin[15].objectnum_x=400,
	zhanpin[15].objectnum_y=300,
	zhanpin[15].direct=90,
	zhanpin[15].time=0,
	zhanpin[15].mode=1,
	zhanpin[15].contect1="",
	zhanpin[15].contect2="这是第一个展品",
	zhanpin[15].contect3="这是第一个展品",

	zhanpin[16].objectnum=84,
	zhanpin[16].objectnum_x=400,
	zhanpin[16].objectnum_y=350,
	zhanpin[16].direct=0,
	zhanpin[16].time=67000,
	zhanpin[16].mode=1,
	zhanpin[16].contect1="金代陶瓷业在中国窑瓷史上是一个不可缺少的组成部分，迁都前在东北地区所生产的陶瓷水平较低，迁都燕京之后在关内生产的陶瓷则有了较大的发展，瓷器造型多承袭宋式的日用器皿，较典型的有双系、三系、四系瓶、系耳罐等，这与女真族的游牧渔猎生活有一定渊源关系。清酒肥羊四系瓶，此瓶瓷质较粗，胎呈米灰色。小口，削肩。肩上附有用于系带的四个桥状耳.大鼓腹.矮圈足。器表绘有简疏、明快的黑色草叶纹和弦纹。中间空白处用黑油横书“清酒肥羊”四个字。这件瓷器从造形、纹饰和文字内容上均带有浓厚的草地游牧生活的气息，是女真传统文化和中原文化相结合的产物。属金代磁州窑系产品。此外还有黑釉玉壶春瓶、黑釉葫芦型小壶。",
	zhanpin[16].contect2="这是第一个展品",
	zhanpin[16].contect3="这是第一个展品",

	zhanpin[17].objectnum=85,
	zhanpin[17].objectnum_x=400,
	zhanpin[17].objectnum_y=400,
	zhanpin[17].direct=0,
	zhanpin[17].time=72000,
	zhanpin[17].mode=1,
	zhanpin[17].contect1="中原地区广为流传的历史故事被采用在铜镜上，由此出现了人物故事镜，汉文化对女真人潜移默化的影响可见一斑。金代铜禁政策严厉，为防止铜源流失，遂在新、旧铜镜边缘刻写官府验记文字和押记符号，这是金代铜镜的一个主要特征。金代山水人物故事镜是我国目前保存最为完好的金代铜镜，做工十分精致。镜背图案分为上、下两部分，铜镜采用写实与夸张相结合的艺术手法，构图虽然复杂，但布局合理，工艺精湛，堪称中国金代铜镜的佳作。这面山水人物故事纹铜镜是1975年从中国黑龙江省绥棱县境内的一座金代贵族墓葬中出土的，是女真贵族曾经使用过的一件器物，距今已有千年的历史。铜镜出土时没有锈蚀，而且保存完好，这在传世的中国古代铜镜中十分罕见，为古代铜镜中所鲜有，是不可多得的实物珍品。",
	zhanpin[17].contect2="这是第一个展品",
	zhanpin[17].contect3="这是第一个展品",

	zhanpin[18].objectnum=86,
	zhanpin[18].objectnum_x=400,
	zhanpin[18].objectnum_y=450,
	zhanpin[18].direct=0,
	zhanpin[18].time=52000,
	zhanpin[18].mode=1,
	zhanpin[18].contect1="1988年5月，哈尔滨市道外区巨源乡城子村的村民在种地时挖到了一座石椁木棺墓。经考古人员考证，该墓是金代齐国王完颜晏夫妇合葬墓。完颜晏(?～1162年)，女真名斡论，是金太祖完颜阿骨打的堂弟，生前拜太尉、齐国王。此墓为夫妻合葬土坑竖穴石椁木棺墓，男墓主人身着8层17件服装，女墓主人身着9层16件服装，用料精美，做工考究。这两套自冠饰、衣裳、至鞋袜服饰基本完整，均为金代贵族服饰，款式还保留女真服饰左衽、窄袖等特点。金齐国王墓出土的丝织品服饰填补了中国金代服饰史的空白，被誉为“北国马王堆”。",
	zhanpin[18].contect2="这是第一个展品",
	zhanpin[18].contect3="这是第一个展品",

	zhanpin[19].objectnum=87,
	zhanpin[19].objectnum_x=400,
	zhanpin[19].objectnum_y=500,
	zhanpin[19].direct=0,
	zhanpin[19].time=0,
	zhanpin[19].mode=1,
	zhanpin[19].contect1="",
	zhanpin[19].contect2="这是第一个展品",
	zhanpin[19].contect3="这是第一个展品",

	zhanpin[20].objectnum=88,
	zhanpin[20].objectnum_x=450,
	zhanpin[20].objectnum_y=500,
	zhanpin[20].direct=90,
	zhanpin[20].time=80000,
	zhanpin[20].mode=1,
	zhanpin[20].contect1="下面介绍的是本馆十大镇馆之宝,铜坐龙。龙是我们华夏先民创造的一种独特的动物形象，崛起于白山黑水间的女真族，入主中原，建立的大金国，受中原先进文化的影响，金代帝王也喜欢把龙作为王权的象征之物。中国古代的龙的形制有盘龙、团龙、飞龙、行龙，而金代却是坐龙居多。女真族是一个狩猎民族，狗是他们的忠实伙伴，金代的坐龙采用狗的坐姿，符合了这个民族的文化心理。金仿宋制，依照《营造法式》记载，宋代建筑的望柱头有用坐龙装饰的规制，大量出土于金代皇家建筑遗址的金代铜座龙，或许是金代皇家建筑中的装饰物件。该件铜坐龙于黑龙江省哈尔滨市阿城区白城金上京会宁府遗址出土，通高19点6厘米，宽17厘米，进深8点5厘米。这就是历史第四展厅的全部内容啦，今天小灵就给大家讲解到这里啦,，大家可以继续往前走参观其余展厅哦，再见啦！",
	zhanpin[20].contect2="这是第一个展品",
	zhanpin[20].contect3="这是第一个展品",

	zhanpin[21].objectnum=89,
	zhanpin[21].objectnum_x=500,
	zhanpin[21].objectnum_y=500,
	zhanpin[21].direct=45,
	zhanpin[21].time=5000,
	zhanpin[21].mode=1,
	zhanpin[21].contect1="",
	zhanpin[21].contect2="这是第一个展品",
	zhanpin[21].contect3="这是第一个展品",
};
int objectnums1[100]={1,50,2,51,3,52,4,99,53,5,6};//这个记录的是展品号(过时)
int objectnumshand[100]={0,7,10,8,6};//这个记录的是展品号
int objectnums[100]={7,81,82,83,84,85,86,87,88,89};//这个记录的是展品号
int objectnowpos=0;//第几个展品
int objectnow=0;//展品数组的中展品的数组号
int objectnowhand=0;
#define PIf 3.1415926
extern bool isbegio;
bool zhanting=false;
bool ishand=false;
UINT ThreadComput_speakautotts(LPVOID lpParam);
CWinThread* pThread_speak_autotts;////自动播放线程
bool autotts_key=true;//自动播放开关
bool isautotts=false;//是否可以播放
int autotts_time_sleep=0; 
CRecorder_ExampleDlg *p_CR=NULL;
extern CVoice *p_DR;
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
//合成函数
void TTSSynth(const string &cap_key, const string &txt_file, const string &out_pcm_file);
bool HCIAPI TtsSynthCallbackFunction(_OPT_ _IN_ void * pvUserParam,_MUST_ _IN_ TTS_SYNTH_RESULT * psTtsSynthResult,_MUST_ _IN_ HCI_ERR_CODE  hciErrCode);
static HCI_ERR_CODE WakeupFunc(void *pUserContext, MICARRAY_WAKE_RESULT *pWakeResult);
static HCI_ERR_CODE WakeDirectionFunc(void *pUserContext, int nDirection);
static HCI_ERR_CODE VoiceReadyFunc(void *pUserContext, short *pVoiceData, int nVoiceSampleCount);
// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialog{
public:
	CAboutDlg();
// 对话框数据
	enum { IDD = IDD_ABOUTBOX };
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
// 实现
protected:
	DECLARE_MESSAGE_MAP()
};
CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD){
}
void CAboutDlg::DoDataExchange(CDataExchange* pDX){
	CDialog::DoDataExchange(pDX);
}
BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()
// CRecorder_ExampleDlg 对话框

CRecorder_ExampleDlg::CRecorder_ExampleDlg(CWnd* pParent /*=NULL*/): CDialog(CRecorder_ExampleDlg::IDD, pParent), m_recordingFlag(FALSE){
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}
void CRecorder_ExampleDlg::DoDataExchange(CDataExchange* pDX){
	CDialog::DoDataExchange(pDX);
	DDX_Check(pDX, IDC_BTN_SAVE_RECORDING, m_recordingFlag);
	DDX_Control(pDX, IDC_BUTTON1, m_btn1);
	DDX_Control(pDX, IDC_BUTTON5, m_btn3);
	DDX_Control(pDX, IDC_BUTTON4, m_btn4);
	DDX_Control(pDX, IDC_STATIC1, m_pic1);
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
	ON_BN_CLICKED(IDC_MFCBUTTON1, &CRecorder_ExampleDlg::OnBnClickedMfcbutton1)
	ON_BN_CLICKED(IDC_MFCBUTTON3, &CRecorder_ExampleDlg::OnBnClickedMfcbutton3)
	ON_BN_CLICKED(IDC_MFCBUTTON5, &CRecorder_ExampleDlg::OnBnClickedMfcbutton5)
	ON_BN_CLICKED(IDC_MFCBUTTON6, &CRecorder_ExampleDlg::OnBnClickedMfcbutton6)
	ON_BN_CLICKED(IDC_MFCBUTTON7, &CRecorder_ExampleDlg::OnBnClickedMfcbutton7)
	ON_BN_CLICKED(IDC_MFCBUTTON2, &CRecorder_ExampleDlg::OnBnClickedMfcbutton2)
	ON_BN_CLICKED(IDC_BUTTON11, &CRecorder_ExampleDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CRecorder_ExampleDlg::OnBnClickedButton12)
END_MESSAGE_MAP()

bool CheckAndUpdataAuth(){
    //获取过期时间
    int64 nExpireTime;
    int64 nCurTime = (int64)time( NULL );
    HCI_ERR_CODE errCode = hci_get_auth_expire_time( &nExpireTime );
    if( errCode == HCI_ERR_NONE ){
        //获取成功则判断是否过期
        if( nExpireTime > nCurTime ){
            //没有过期
            printf( "auth can use continue\n" );
            return true;
        }
    }
    //获取过期时间失败或已经过期
    //手动调用更新授权
    errCode = hci_check_auth();
    if( errCode == HCI_ERR_NONE ){
        //更新成功
        printf( "check auth success \n" );
        return true;
    }else{
        //更新失败
        printf( "check auth return (%d:%s)\n", errCode ,hci_get_error_info(errCode));
        return false;
    }
}
//获取capkey属性
void GetCapkeyProperty(const string&cap_key,AsrRecogType & type,AsrRecogMode &mode){
    HCI_ERR_CODE errCode = HCI_ERR_NONE;
	CAPABILITY_ITEM *pItem = NULL;
	// 枚举所有的asr能力
	CAPABILITY_LIST list = {0};
	if ((errCode = hci_get_capability_list("asr", &list))!= HCI_ERR_NONE){
		// 没有找到相应的能力。
		return;
	}
	// 获取asr能力配置信息。
	for (int i = 0; i < list.uiItemCount; i++){
		if (list.pItemList[i].pszCapKey != NULL && stricmp(list.pItemList[i].pszCapKey, cap_key.c_str()) == 0){
			pItem = &list.pItemList[i];
			break;
		}
	}
	// 没有获取相应能力配置，返回。
	if (pItem == NULL || pItem->pszCapKey == NULL){
		hci_free_capability_list(&list);
		return;
	}
	if (strstr(pItem->pszCapKey, "cloud") != NULL){
		type = kRecogTypeCloud;
	}else{
		type = kRecogTypeLocal;
	}  
	if (strstr(pItem->pszCapKey, "freetalk") != NULL){
		mode = kRecogModeFreetalk;
	}else if (strstr(pItem->pszCapKey, "grammar") != NULL){
		mode = kRecogModeGrammar;
	}else if (strstr(pItem->pszCapKey, "dialog") != NULL){
		mode = kRecogModeDialog;
	}else{
		mode = kRecogModeUnkown;
	}
	hci_free_capability_list(&list);
    return;
};
BOOL CRecorder_ExampleDlg::OnInitDialog(){
	CDialog::OnInitDialog();
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);
	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL){
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty()){
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
    _T("微软雅黑")); // lpszFac   
     // SetTextColor(HDC hDC,RGB(255,255,0)); //设置字体颜色  
      //将按钮修改为BS_OWNERDRAW风格,允许button的采用自绘模式
     GetDlgItem(IDC_MFCBUTTON7)->ModifyStyle(0,BS_OWNERDRAW,0);
     //绑定控件IDC_BUTTON1与类CMyButton，响应重载函数DrawItem()
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
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标
	((CButton *)GetDlgItem( IDC_CONTINUE ))->SetCheck(TRUE);
	p_CR=(CRecorder_ExampleDlg*)this;
    if (Init() == false){
        return FALSE;
    }
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}
void CRecorder_ExampleDlg::OnSysCommand(UINT nID, LPARAM lParam){
	if ((nID & 0xFFF0) == IDM_ABOUTBOX){
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else{
		CDialog::OnSysCommand(nID, lParam);
	}
}
// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CRecorder_ExampleDlg::OnPaint(){
	if (IsIconic()){
		CPaintDC dc(this); // 用于绘制的设备上下文
		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);
		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}else{
		//CDialog::OnPaint(); 
		//添加代码对话框背景贴图
		CPaintDC   dc(this);  
		CRect   rect;  
		GetClientRect(&rect);    //获取对话框长宽      
		CDC   dcBmp;             //定义并创建一个内存设备环境
		dcBmp.CreateCompatibleDC(&dc);             //创建兼容性DC
		CBitmap   bmpBackground;   
		bmpBackground.LoadBitmap(IDB_BITMAP7);    //载入资源中图片
		BITMAP   m_bitmap;                         //图片变量               
		bmpBackground.GetBitmap(&m_bitmap);       //将图片载入位图中
		//将位图选入临时内存设备环境
		CBitmap  *pbmpOld=dcBmp.SelectObject(&bmpBackground);
		//调用函数显示图片StretchBlt显示形状可变
		dc.StretchBlt(0,0,rect.Width(),rect.Height(),&dcBmp,0,0,m_bitmap.bmWidth,m_bitmap.bmHeight,SRCCOPY); 
	}
}
//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CRecorder_ExampleDlg::OnQueryDragIcon(){
	return static_cast<HCURSOR>(m_hIcon);
}
LRESULT CRecorder_ExampleDlg::OnShowStatus( WPARAM wParam, LPARAM lParam ){
	CString * str = (CString *)lParam;
	AppendMessage(*str);
	delete str;
	RECORDER_EVENT eEvent = (RECORDER_EVENT)wParam;
	switch( eEvent ){
	// 若是开始录音、听到声音或者开始识别，则使按钮不可用
	case RECORDER_EVENT_BEGIN_RECORD:
	case RECORDER_EVENT_BEGIN_RECOGNIZE:		
	case RECORDER_EVENT_HAVING_VOICE:
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );
		break;
		// 状态保持不变
	case RECORDER_EVENT_ENGINE_ERROR:
		break;
		// 录音结束、任务结束
	case RECORDER_EVENT_END_RECORD:
	case RECORDER_EVENT_TASK_FINISH:
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
		break;
		// 识别结束
	case RECORDER_EVENT_RECOGNIZE_COMPLETE:
		if (IsDlgButtonChecked( IDC_CONTINUE ) == FALSE)
		{
			GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
			GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
		}
		break;
		// 其他状态，包括未听到声音或者发生错误等，则恢复按钮可用
	default:
		char buff[32];
		sprintf(buff, "Default Event:%d", eEvent);
		AppendMessage(CString(buff));

		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
	}
	return 0;
}
void CRecorder_ExampleDlg::OnBnClickedBtnStartRecord(){	
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
	eRetasr = hci_asr_recorder_init( initConfig.c_str(), &call_back);
	if (eRetasr != RECORDER_ERR_NONE){
		hci_release();
		strErrorMessage.Format( "录音机初始化失败,错误码%d", eRetasr);
		MessageBox( strErrorMessage );
		return ;
	}

	RECORDER_ERR_CODE eRet = RECORDER_ERR_NONE;
	
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );	
	
	// 清空状态记录
	SetDlgItemText( IDC_EDIT1, "" );

    AccountInfo *account_info = AccountInfo::GetInstance();
    string startConfig = "";
	if (!IsDlgButtonChecked( IDC_ONLY_RECORDING )){
     	startConfig += "capkey=" + account_info->cap_key();
	}
	startConfig += ",audioformat=pcm16k16bit";
	//startConfig += ",domain=qwdz,intention=qwmap;music,needcontent=no";
	//startConfig     += ",realTime=rt";
    if (IsDlgButtonChecked( IDC_CONTINUE )){
        startConfig += ",continuous=yes";
    }
	if ( m_RecogMode == kRecogModeGrammar ){
		char chTmp[32] = {0};
		sprintf(chTmp,",grammarid=%d",m_GrammarId);
		startConfig += chTmp; 
	}
	if ( m_RecogMode == kRecogModeDialog ){
		startConfig +=",intention=weather;joke;story;baike;calendar;translation;news"; 
	}
	eRet = hci_asr_recorder_start(startConfig.c_str(),"");
	if (RECORDER_ERR_NONE != eRet){
		CString strErrMessage;
		strErrMessage.Format( "开始录音失败,错误码%d", eRet );
		MessageBox( strErrMessage );
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		return;
	}
}
bool CRecorder_ExampleDlg::Init(){	
    CString strErrorMessage;
	m_recordingFlag = FALSE;
	m_recordingFileName = "recording.pcm";
	m_recordingFile = NULL;
	SetDlgItemText( IDC_EDIT_SAVE_RECORDING_FILE, m_recordingFileName );
	UpdateData(FALSE);
    // 获取AccountInfo单例
    AccountInfo *account_info = AccountInfo::GetInstance();
    // 账号信息读取
    string account_info_file = "../../testdata/AccountInfo.txt";
    bool account_success = account_info->LoadFromFile(account_info_file);
    if (!account_success){
        strErrorMessage.Format("AccountInfo read from %s failed\n", account_info_file.c_str());
        MessageBox(strErrorMessage);
        return false;
    }

    // SYS初始化
    HCI_ERR_CODE errCode = HCI_ERR_NONE;
    // 配置串是由"字段=值"的形式给出的一个字符串，多个字段之间以','隔开。字段名不分大小写。
    string init_config = "";
    init_config += "appKey=" + account_info->app_key();              //灵云应用序号
    init_config += ",developerKey=" + account_info->developer_key(); //灵云开发者密钥
    init_config += ",cloudUrl=" + account_info->cloud_url();         //灵云云服务的接口地址
	init_config += ",capKey=sma.local.wake;sma.local.doa;sma.local.vqe" ; 
	init_config += ",authpath=" + account_info->auth_path();         //授权文件所在路径，保证可写
    init_config += ",logfilepath=" + account_info->logfile_path();   //日志的路径
	init_config += ",logfilesize=1024000,loglevel=5";
    // 其他配置使用默认值，不再添加，如果想设置可以参考开发手册
    errCode = hci_init( init_config.c_str() );
    if( errCode != HCI_ERR_NONE ){
        strErrorMessage.Format( "hci_init return (%d:%s)\n", errCode, hci_get_error_info(errCode) );
        MessageBox(strErrorMessage);
        return false;
    }
    printf( "hci_init success\n" );


    // 检测授权,必要时到云端下载授权。此处需要注意的是，这个函数只是通过检测授权是否过期来判断是否需要进行
    // 获取授权操作，如果在开发调试过程中，授权账号中新增了灵云sdk的能力，请到hci_init传入的authPath路径中
    // 删除HCI_AUTH文件。否则无法获取新的授权文件，从而无法使用新增的灵云能力。
    if (!CheckAndUpdataAuth()){
        hci_release();
        strErrorMessage.Format("CheckAndUpdateAuth failed\n");
        MessageBox(strErrorMessage);
        return false;
    }

    // capkey属性获取
    m_RecogType = kRecogTypeUnkown;
    m_RecogMode = kRecogModeUnkown;
    GetCapkeyProperty(account_info->cap_key(),m_RecogType,m_RecogMode);

	if( m_RecogType == kRecogTypeCloud && m_RecogMode == kRecogModeGrammar ){
        // 云端语法暂时不支持实时识别
		// GetDlgItem( IDC_REALTIME )->EnableWindow(FALSE);
		hci_release();
        strErrorMessage.Format("Recorder not support cloud grammar, init failed\n");
        MessageBox(strErrorMessage);
        return false;
	}

/*asr_recorder初始化*/
    RECORDER_ERR_CODE eRet = RECORDER_ERR_UNKNOWN;
    m_GrammarId = -1;
    if (m_RecogMode == kRecogModeGrammar){
        string grammarFile = account_info->test_data_path() + "/stock_10001.gram";
        if (m_RecogType == kRecogTypeLocal){
			string strLoadGrammarConfig = "grammarType=jsgf,isFile=yes,capkey=" + account_info->cap_key();
            eRet = hci_asr_recorder_load_grammar(strLoadGrammarConfig.c_str() , grammarFile.c_str(), &m_GrammarId );
            if( eRet != RECORDER_ERR_NONE ){
                hci_asr_recorder_release();
                hci_release();
                strErrorMessage.Format( "载入语法文件失败,错误码%d", eRet );
                MessageBox( strErrorMessage );
                return false;
            }
            EchoGrammarData(grammarFile);
        }
        else{
            // 如果是云端语法识别，需要开发者通过开发者社区自行上传语法文件，并获得可以使用的ID。
            // m_GrammarId = 2;
        }
    }

#ifdef COMM_MOTOR
// 电机串口初始化
if(motor.open_com_motor(COMM_MOTOR)){
	//电机数据计算
	pThread_Motor_Comput = AfxBeginThread(ThreadComput_MotorData,NULL);
	int a= 1;
}else{
	int a=0;
}
#endif

#ifdef COMM_STAR
	//星标定位串口初始化
	if (StarGazer.open_com(COMM_STAR)){
		int a=1;
	}else{
		int a=0;
	}
#endif

#ifdef COMM_LASER
	//激光数据初始赋值10000
	for(int loop=0;loop<1000;loop++){
		m_laser_data_postpro[loop] = 50000;
	}
	// 激光串口初始化
	if (m_cURG.Create(COMM_LASER)){
		m_cURG.SwitchOn();
		m_cURG.SCIP20();	
		m_cURG.GetDataByGD(0,768,1);
		pThread_Read_Laser=AfxBeginThread(ThreadReadLaser_Data,&Info_laser_data);
	}
#endif
	plan.Scene_scale_x = 20.0;
	plan.Scene_scale_y = 20.0;
	plan.danger = false;
	vfh_Scene_scale_x = 20.0;
	vfh_Scene_scale_y = 20.0;
	Obstacle_Distance_init = 1200;/////////////////////////初始避障距离设定
	delt_Obstacle_Distance_init = 200;
	free_Obstacle_Distance_init = 1600;
	dataexchange_key = true;
	vfh_key = true;
	motor_key = false;
	//激光数据初始化
	for (int loop = 0;loop<1000;loop++){
		plan.m_laser_data_postpro[loop] = 50000;
		m_laser_data_postpro_vfh[loop] = 50000;
	}
	//与VFH算法中数据交换
	pThread_DataExchange=AfxBeginThread(ThreadDataExchange,NULL);
	plan.Init();
	algorithm.Init();
	pThread_VFHStart = AfxBeginThread(ThreaVFH,NULL); //vfh初始化,激光位置设定
	SetTimer(1,200,NULL);
    return true;
}
void CRecorder_ExampleDlg::EchoGrammarData(const string &grammarFile){
    FILE* fp = fopen( grammarFile.c_str(), "rt" );
    if( fp == NULL ){
        GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
        CString strErrorMessage;
        strErrorMessage.Format("打开语法文件%s失败",grammarFile.c_str());
        MessageBox( strErrorMessage );
        return;
    }
    unsigned char szBom[3];
    fread( szBom, 3, 1, fp );
    // 若有bom头，则清除，没有则当前位置回到头部
    if( !( szBom[0] == 0xef && szBom[1] == 0xbb && szBom[2] == 0xbf ) ){
        fseek( fp, 0, SEEK_SET );
    }
    CString grammarData = "";
    char szData[1024] = {0};
    while( fgets( szData, 1024, fp ) != NULL ){
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
bool CRecorder_ExampleDlg::Uninit(void){
	HCI_ERR_CODE eRet = HCI_ERR_NONE;	
	// 如果是本地语法识别，则需要释放语法资源
	if( m_RecogType == kRecogTypeLocal && m_RecogMode == kRecogModeGrammar ){
		hci_asr_recorder_unload_grammar( m_GrammarId );
	}
	RECORDER_ERR_CODE eRecRet;
	eRecRet = hci_asr_recorder_release();
	if(eRecRet != RECORDER_ERR_NONE){
		return false;
	}
	HCI_ERR_CODE errCode;
	errCode = hci_micarray_session_stop(&MicArraysession);
	errCode = hci_micarray_release(&MicArrayhandle);
	eRet = hci_release();
	AccountInfo::ReleaseInstance();
	return eRet == HCI_ERR_NONE;
}
void CRecorder_ExampleDlg::OnBnClickedBtnCancelRecord(){
	RECORDER_ERR_CODE eRet = hci_asr_recorder_cancel();
	if (RECORDER_ERR_NONE != eRet){
		CString str;
		str.Format( _T("终止录音失败,错误码%d"), eRet );
		MessageBox( str );
		return;
	}
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( FALSE );
	hci_asr_recorder_release();
	HCI_ERR_CODE err_code = HCI_ERR_NONE;
	string tts_init_config = "dataPath=";
    tts_init_config += "../../data";
	tts_init_config += ",initCapkeys=tts.cloud.wangjing";
	err_code = hci_tts_init(tts_init_config.c_str());
	if (err_code != HCI_ERR_NONE){
		printf("hci_tts_init return (%d:%s) \n",err_code,hci_get_error_info(err_code));
		return;
	}
	// 设置语音合成测试文件
	string file_to_synth;
	if ("tts.cloud.wangjing" == "tts.local.synth.sing"){
		file_to_synth = "../../testdata/S3ML_sing.txt.enc";
	}else{
		file_to_synth = "../../testdata/tts.txt";
	}
	string out_pcm_file = "../../testdata/ttsceshi.pcm";
	TTSSynth("tts.cloud.wangjing", file_to_synth, out_pcm_file);
	//TTS反初始化
	hci_tts_release();
    printf("hci_tts_release\n");
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
	if (eRet != PLAYER_ERR_NONE){
		hci_release();
		CString str;
		str.Format( "播放器初始化失败,错误码%d.", eRet);
		MessageBox( str );
	}
	string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
	PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)tts, startConfig.c_str() );
	if(eRetk==PLAYER_ERR_NONE){
		int a=5;
	}
}
void CRecorder_ExampleDlg::OnBnClickedOk(){
	delete dlg;
}
struct{
    char* pszName;
    char* pszComment;
}
g_sStatus[] ={
    {"RECORDER_EVENT_BEGIN_RECORD",         "问个问题吧"},
    {"RECORDER_EVENT_HAVING_VOICE",         "听到声音 检测到始端的时候会触发该事件"},
    {"RECORDER_EVENT_NO_VOICE_INPUT",       "没有听到声音"},
    {"RECORDER_EVENT_BUFF_FULL",            "缓冲区已填满"},
    {"RECORDER_EVENT_END_RECORD",           "组织语言ing"},
    {"RECORDER_EVENT_BEGIN_RECOGNIZE",      "倾听中"},
    {"RECORDER_EVENT_RECOGNIZE_COMPLETE",   "我想好了"},
    {"RECORDER_EVENT_ENGINE_ERROR",         "引擎出错"},
    {"RECORDER_EVENT_DEVICE_ERROR",         "设备出错"},
    {"RECORDER_EVENT_MALLOC_ERROR",         "分配空间失败"},
    {"RECORDER_EVENT_INTERRUPTED",          "内部错误"},
    {"RECORDER_EVENT_PERMISSION_DENIED",    "内部错误"},
    {"RECORDER_EVENT_TASK_FINISH",          "小灵要休息一下"},
    {"RECORDER_EVENT_RECOGNIZE_PROCESS",    "识别中间状态"}
};

void HCIAPI CRecorder_ExampleDlg::RecordEventChange(RECORDER_EVENT eRecorderEvent, void *pUsrParam){
    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
	if(eRecorderEvent == RECORDER_EVENT_BEGIN_RECOGNIZE){
		dlg->m_startClock = clock();
	}
	if(eRecorderEvent == RECORDER_EVENT_END_RECORD){
		if(dlg->m_recordingFile != NULL){
			fclose(dlg->m_recordingFile);
			dlg->m_recordingFile = NULL;
		}
	}

	CString strMessage(g_sStatus[eRecorderEvent].pszComment);
	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}
void CRecorder_ExampleDlg::AppendMessage(CString & strMsg){
    CString strMessage = "";
    p_DR->GetDlgItemText( IDC_EDIT1, strMessage );
	int nMessageLenMax = 1024;
	if(strMessage.GetLength() > nMessageLenMax){
		strMessage = strMessage.Right(nMessageLenMax);
	}
	CString strNewMessage = "";
	strNewMessage = strMsg;
	if(strMessage.GetLength() > 0){
		strNewMessage += "\r\n";
		strNewMessage += strMessage;
	}
    p_DR->SetDlgItemText( IDC_EDIT1, strNewMessage );

}
void CRecorder_ExampleDlg::PostRecorderEventAndMsg(RECORDER_EVENT eRecorderEvent, const CString & strMessage){
	CString * msg = new CString(strMessage);
	::PostMessage(m_hWnd, WM_USER_SHOW_STATUS, eRecorderEvent, (LPARAM)msg);
}
void HCIAPI CRecorder_ExampleDlg::RecorderRecogFinish(RECORDER_EVENT eRecorderEvent,ASR_RECOG_RESULT *psAsrRecogResult,void *pUsrParam){
	CString strMessage = "";
    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
	if(eRecorderEvent == RECORDER_EVENT_RECOGNIZE_COMPLETE){
		char buff[32];
		clock_t endClock = clock();
		dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
	}

    strMessage = "";
    if( psAsrRecogResult->uiResultItemCount > 0 ){
        unsigned char* pucUTF8 = NULL;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, &pucUTF8 );
       
		unsigned char* pszGBK;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, (unsigned char**)&pszGBK);

		tts=(char* )pszGBK;

	  //strMessage.AppendFormat( "小灵的答案: %s", pucUTF8 );
        HciExampleComon::FreeConvertResult( pucUTF8 );
		char buf[10000] = {NULL};
		char result[10000]={NULL};
		Convert(psAsrRecogResult->psResultItemList[0].pszResult,buf,CP_UTF8,CP_ACP);
		int iLength ;
		Json_Explain(buf,buf,result);
		HciExampleComon::GBKToUTF8( (unsigned char*)buf, (unsigned char**)&pszGBK);
		tts=(char* )pszGBK;
		strMessage.AppendFormat( "您的问题: %s\r\n小灵的答案: %s", result , buf);
        pucUTF8 = NULL;
		isautotts=true;
		autotts_time_sleep=strlen(tts);
    }
    else{
        strMessage.AppendFormat( "小灵没听清，能再说一遍吗" );
    }
	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}
void HCIAPI CRecorder_ExampleDlg::RecorderRecogProcess(RECORDER_EVENT eRecorderEvent,ASR_RECOG_RESULT *psAsrRecogResult,void *pUsrParam){
    CRecorder_ExampleDlg *dlg = (CRecorder_ExampleDlg*)pUsrParam;
    CString strMessage = "";
    if( psAsrRecogResult->uiResultItemCount > 0 ){
        unsigned char* pucUTF8 = NULL;
        HciExampleComon::UTF8ToGBK( (unsigned char*)psAsrRecogResult->psResultItemList[0].pszResult, &pucUTF8 );
        strMessage.AppendFormat( "识别中间结果: %s", pucUTF8 );
        HciExampleComon::FreeConvertResult( pucUTF8 );
        pucUTF8 = NULL;
    }
    else{
        strMessage.AppendFormat( "*****无识别结果*****" );
    }
	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);    
}
void HCIAPI CRecorder_ExampleDlg::RecorderErr(RECORDER_EVENT eRecorderEvent,HCI_ERR_CODE eErrorCode,void *pUsrParam){
    CRecorder_ExampleDlg * dlg = (CRecorder_ExampleDlg*)pUsrParam;
    CString strMessage = "";
    strMessage.AppendFormat( "系统错误:%d", eErrorCode );

	dlg->PostRecorderEventAndMsg(eRecorderEvent, strMessage);
}
void HCIAPI CRecorder_ExampleDlg::RecorderRecordingCallback(unsigned char * pVoiceData,unsigned int uiVoiceLen,void * pUsrParam){
	CRecorder_ExampleDlg * dlg = (CRecorder_ExampleDlg *)pUsrParam;
	dlg->RecorderRecording(pVoiceData, uiVoiceLen);
}
void CRecorder_ExampleDlg::RecorderRecording(unsigned char * pVoiceData, unsigned int uiVoiceLen){
	if(m_recordingFlag == FALSE){
		if(m_recordingFile != NULL){
			fclose(m_recordingFile);
			m_recordingFile = NULL;
		}
		return;
	}

	if(m_recordingFile == NULL){
		m_recordingFile = fopen( m_recordingFileName.GetBuffer(), "wb" );
		if( m_recordingFile == NULL ){
			return;
		}
	}

	fwrite(pVoiceData, sizeof(unsigned char), uiVoiceLen, m_recordingFile);
	fflush(m_recordingFile);
}
void CRecorder_ExampleDlg::OnBnClickedBtnBrowser(){
	CFileDialog dlgFile(TRUE, NULL, m_recordingFileName.GetBuffer(), OFN_HIDEREADONLY, _T("PCM Files (*.pcm)|*.pcm|All Files (*.*)|*.*||"), NULL);
    if (dlgFile.DoModal()){
        m_recordingFileName = dlgFile.GetPathName();
    }
    SetDlgItemText( IDC_EDIT_SAVE_RECORDING_FILE, m_recordingFileName );
}
void CRecorder_ExampleDlg::OnBnClickedSaveRecording(){
	UpdateData(TRUE);

	if(m_recordingFlag == FALSE){
		if(m_recordingFile != NULL)	{
			fclose(m_recordingFile);
			m_recordingFile = NULL;
		}
	}
}
void CRecorder_ExampleDlg::OnEnChangeEditStatus(){
}
bool HCIAPI TtsSynthCallbackFunction(_OPT_ _IN_ void * pvUserParam,_MUST_ _IN_ TTS_SYNTH_RESULT * psTtsSynthResult,_MUST_ _IN_ HCI_ERR_CODE  hciErrCode){
    if( hciErrCode != HCI_ERR_NONE ){
        return false;
    }
    //printf("voice data size %d\n",psTtsSynthResult->uiVoiceSize);
    // 将合成结果写入文件
    if (psTtsSynthResult->pvVoiceData != NULL){
        FILE * fp = (FILE *)pvUserParam;
        fwrite(psTtsSynthResult->pvVoiceData, psTtsSynthResult->uiVoiceSize, 1, fp);
    }
	//mark 回调结果
	if (psTtsSynthResult->nMarkCount > 0){
		for (int i=0; i<psTtsSynthResult->nMarkCount; ++i){
			printf("MarkName:%s, with the time in audio:%d \n",psTtsSynthResult->pMark[i].pszName,psTtsSynthResult->pMark[i].time);
		}

	}
    // 此回调函数返回false会中止合成，返回true表示继续合成
    return true;
}
void TTSSynth(const string &cap_key, const string &txt_file, const string &out_pcm_file ){
    // 合成文本读取
    HciExampleComon::FileReader txt_data;
    if( txt_data.Load(txt_file.c_str(),1) == false )
    {
        printf( "Open input text file %s error!\n", txt_file.c_str() );
        return;
    }
    // 打开输出文件
    FILE * fp = fopen( out_pcm_file.c_str(), "wb" );
    if( fp == NULL ){
        printf( "Create output pcm file %s error!\n", out_pcm_file.c_str());
        return;
    }

    HCI_ERR_CODE err_code = HCI_ERR_NONE;
    // 启动 TTS Session
    string session_config = "capkey=";
    session_config += cap_key;
    int session_id = -1;

    printf( "hci_tts_session_start config [%s]\n", session_config.c_str() );
    err_code = hci_tts_session_start( session_config.c_str(), &session_id );
    if( err_code != HCI_ERR_NONE ){
        printf("hci_tts_session_start return (%d:%s) \n",err_code,hci_get_error_info(err_code));
        fclose(fp);
        return;
    }
    printf( "hci_tts_session_start success\n" );

	string synth_config;
	if (cap_key.find("tts.cloud.synth") != string::npos){
		//property 属于 私有云 云端能力 必填参数，具体请参考开发手册
		//none: 所有标记将会被视为文本读出，缺省值
		synth_config = "property=cn_xiaokun_common,tagmode=none";
	}

	if (cap_key.find("tts.local.synth.sing") != string::npos){
		synth_config = "tagmode=s3ml_sing";
	}
//*	char*tts="我是";
	char *pUTF8Str=tts;
	
	unsigned char* pszGBK;
    HciExampleComon::GBKToUTF8( (unsigned char*)pUTF8Str, (unsigned char**)&pszGBK);

    err_code = hci_tts_synth( session_id, (char*)pszGBK, synth_config.c_str(), TtsSynthCallbackFunction, fp );
//*/    
//	err_code = hci_tts_synth( session_id, (char*)txt_data.buff_, synth_config.c_str(), TtsSynthCallbackFunction, fp );
	fclose(fp);

    if( err_code != HCI_ERR_NONE ){
        printf("hci_tts_session_start return (%d:%s) \n",err_code,hci_get_error_info(err_code));
    }

    // 终止 TTS Session
    err_code = hci_tts_session_stop( session_id );
    if( err_code != HCI_ERR_NONE ){
        printf( "hci_tts_session_stop return %d\n", err_code );
        return;
    }
    printf( "hci_tts_session_stop success\n" );

    return;
}
void CRecorder_ExampleDlg::OnBnClickedOnlyRecording(){
	
}
void CRecorder_ExampleDlg::OnBnClickedContinue(){
	
}
void CRecorder_ExampleDlg::OnEnChangeEditSaveRecordingFile(){
	
}
void HCIAPI CRecorder_ExampleDlg::CB_EventChange(_MUST_ _IN_ PLAYER_EVENT ePlayerEvent,_OPT_ _IN_ void * pUsrParam){
    string strEvent;
    switch ( ePlayerEvent ){
    case PLAYER_EVENT_BEGIN:strEvent = "开始播放";break;
    case PLAYER_EVENT_PAUSE:strEvent = "暂停播放"; break;
    case PLAYER_EVENT_RESUME:strEvent = "恢复播放";break;
    case PLAYER_EVENT_PROGRESS:strEvent = "播放进度";break;
    case PLAYER_EVENT_BUFFERING:strEvent = "播放缓冲";break;
    case PLAYER_EVENT_END:strEvent = "播放完毕";break;
    case PLAYER_EVENT_ENGINE_ERROR:strEvent = "引擎出错";break;
    case PLAYER_EVENT_DEVICE_ERROR:strEvent = "设备出错";break;
    }
}
void HCIAPI CRecorder_ExampleDlg::CB_ProgressChange (_MUST_ _IN_ PLAYER_EVENT ePlayerEvent,_MUST_ _IN_ int nStart,_MUST_ _IN_ int nStop,_OPT_ _IN_ void * pUsrParam){
    string strEvent;
    char szData[256] = {0};
    switch ( ePlayerEvent ){
    case PLAYER_EVENT_BEGIN:strEvent = "开始播放";break;
    case PLAYER_EVENT_PAUSE:strEvent = "暂停播放";break;
    case PLAYER_EVENT_RESUME:strEvent = "恢复播放";break;
    case PLAYER_EVENT_PROGRESS:sprintf( szData, "播放进度：起始=%d,终点=%d", nStart, nStop );strEvent = szData;break;
    case PLAYER_EVENT_BUFFERING:strEvent = "播放缓冲";break;
    case PLAYER_EVENT_END:strEvent = "播放完毕";break;
    case PLAYER_EVENT_ENGINE_ERROR:strEvent = "引擎出错";break;
	case PLAYER_EVENT_DEVICE_ERROR:strEvent = "设备出错";break;
    }
}
void HCIAPI CRecorder_ExampleDlg::CB_SdkErr( _MUST_ _IN_ PLAYER_EVENT ePlayerEvent,_MUST_ _IN_ HCI_ERR_CODE eErrorCode,_OPT_ _IN_ void * pUsrParam ){
    string strEvent;
    switch ( ePlayerEvent ){
	case PLAYER_EVENT_BEGIN:strEvent = "开始播放";break;
	case PLAYER_EVENT_PAUSE:strEvent = "暂停播放";break;
	case PLAYER_EVENT_RESUME:strEvent = "恢复播放";break;
	case PLAYER_EVENT_PROGRESS:strEvent = "播放进度";break;
	case PLAYER_EVENT_BUFFERING:strEvent = "播放缓冲";break;
	case PLAYER_EVENT_END:strEvent = "播放完毕";break;
	case PLAYER_EVENT_ENGINE_ERROR:strEvent = "引擎出错";break;
	case PLAYER_EVENT_DEVICE_ERROR:strEvent = "设备出错";break;
    }
}
static HCI_ERR_CODE WakeupFunc(void *pUserContext, MICARRAY_WAKE_RESULT *pWakeResult){
	HCI_ERR_CODE errCode = HCI_ERR_NONE;
	return errCode;
}
static HCI_ERR_CODE WakeDirectionFunc(void *pUserContext, int nDirection){
	HCI_ERR_CODE errCode = HCI_ERR_NONE;
	printf(":: Wake Direction : %p, %d\n", pUserContext, nDirection);
	return errCode;
}
static HCI_ERR_CODE VoiceReadyFunc(void *pUserContext, short *pVoiceData, int nVoiceSampleCount){
	HCI_ERR_CODE errCode = HCI_ERR_NONE;
	static FILE * m_fpOut = NULL;
	int size;
	char *filename = "mictest.pcm";
	if(m_fpOut == NULL){
		m_fpOut = fopen(filename, "wb");
		if(m_fpOut == NULL){
			printf("VoiceReadyFunc Open File Failed:%s.\n", filename);
			errCode = HCI_ERR_UNSUPPORT;
			return errCode;
		}
	}
	if(m_fpOut != NULL){
		size = fwrite(pVoiceData, sizeof(short), nVoiceSampleCount, m_fpOut);
		if(size != nVoiceSampleCount){
			printf(":: fwrite data failed, %d of %d, %d, %s\n",
			       size, nVoiceSampleCount, errno, strerror(errno));
		}
		fflush(m_fpOut);
	}
	return HCI_ERR_NONE;
}
void Convert(char* strIn,char* strOut, int sourceCodepage, int targetCodepage){  
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
int Json_Explain (char buf[],char Dest[],char result[]){  
	cJSON *json , *json_result, *json_intention, *json_answer,*json_content,*date,*description,*direction,*high,*low,*location,
		*power,*domain,*content,*text,*date_gongli,*lunarDay,*lunarMonth,*week,*holiday,*channelName,*desc,*title;
	char temp[1000] = {NULL};
	// 解析数据包  
	json = cJSON_Parse(buf);  
	if (!json){  
		printf("Error before: [%s]\n",cJSON_GetErrorPtr());  
	}  
	else{// 解析开关值  
		json_result = cJSON_GetObjectItem( json, "result");  //intention=weather;calendar;train;flight;joke;story;baike
		json_answer =  cJSON_GetObjectItem(json , "answer");
		json_content = cJSON_GetObjectItem(json_answer , "content");
		json_intention=cJSON_GetObjectItem(json_answer , "intention");
		domain=cJSON_GetObjectItem(json_intention , "domain");
		if( json_result != NULL &&domain != NULL ){
			strcpy(result,json_result->valuestring);
			if(!strcmp(domain->valuestring,"weather")){
				// 从valuestring中获得结果  
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
				strcat(Dest,"风力");
				strcat(Dest,power->valuestring);
				strcat(Dest,",");
				//strcat(Dest,location->valuestring);
				strcat(Dest,"最高气温");
				itoa(high->valueint,temp,10);
				strcat(Dest,temp);
				strcat(Dest,"摄氏度");
				strcat(Dest,",");
				strcat(Dest,"最低气温");
				itoa(low->valueint,temp,10);
				strcat(Dest,temp);
				strcat(Dest,"摄氏度");
				strcat(Dest,"。");
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
				strcat(Dest,"农历");
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
		{  strcpy(Dest,"哎呀,这个问题我不会啊,你教教我吧!");}
		// 释放内存空间  
		cJSON_Delete(json); 
	}
	return 0;  
  }  
//电机数据计算线程
UINT ThreadComput_MotorData(LPVOID lpParam){
	while(moter_key){
		//Sleep(200);
		WaitForSingleObject(wait_motordata,200);
	//	distance_l = (float)motor.encoder_l*100/1160/27;
	//	distance_r = (float)motor.encoder_r*100/1160/27;

		distance_l = -(float)motor.encoder_l/1024/26*102*(1-0.23);//(1-0.23)为实际测量距离与计算距离偏差修正
		distance_r = -(float)motor.encoder_r/1024/26*102*(1-0.23);
		distance_z = -(float)motor.encoder_z/1024/26*102*(1-0.23);

		if (distanceold_l != 0){
			distancedif_l = distance_l - distanceold_l;	//距离差值计算
		}
		if (distanceold_r != 0){
			distancedif_r = distance_r - distanceold_r;
		}
		if (distanceold_z != 0){
			distancedif_z = distance_z - distanceold_z;
		}

		distanceold_l = distance_l;  //更新记录
		distanceold_r = distance_r;
		distanceold_z = distance_z;

		motor.RobotPositionCompute(distancedif_l,distancedif_r,distancedif_z,Info_robot);
		FILE *allout;
		allout = fopen("aa.txt","a+");
		for(int i=0;i<12;++i){
			fprintf(allout," bbbbb===%d    \n" ,plan.distance_min[i]);
		}
		fprintf(allout," ======================   \n" );
		fclose(allout);
	}
	return 0;
}
void CRecorder_ExampleDlg::OnBnClickedButton2(){//暂停

}
//确定位置
UINT ThreadComput_MotorTts(LPVOID lpParam){
	while(motertts_key){
		//Sleep(200);
		WaitForSingleObject(wait_motortts,200);		
	}
	return 0;
}
UINT ThreadComput_Motorautowalk(LPVOID lpParam){
	while(moterautowalk_key){
		WaitForSingleObject(wait_motortts,300);
		if (plan.Range_to_go(plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy)){
			char* tts_motor=zhanpin[objectnow].contect1;
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );
			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
//			WaitForSingleObject(wait_motortts,100);
			Sleep(zhanpin[objectnow].time);
			if(zhanpin[objectnow].mode==2){
				objectnowpos=objectnowpos+1;
				int nums_oba=0;
				for(;nums_oba<100;++nums_oba){
					if(zhanpin[nums_oba].objectnum==objectnums[objectnowpos]){
						break;
					}
				}
				plan.target_x=zhanpin[nums_oba].objectnum_x;
				plan.target_z=zhanpin[nums_oba].objectnum_y;//xhy目标点
				plan.speed_line_pos=0;
				plan.Desired_Angle_ob=zhanpin[nums_oba].direct;
				objectnow=nums_oba;
				isbegio=true;
				plan.SERVEMODE = 2;
				plan.CtrlMode=2;
			}
			else if(abs(plan.pianzhuan*180/PIf-plan.Desired_Angle_ob)<=20.0){
				wait_motor_timer = 200*10000;
				plan.speed_line = 0;
				plan.speed_angle = 0;
				plan.speed_l = 0;
				plan.speed_r = 0;
				plan.SERVEMODE = 4; //切换到等待模式
				motor.stop(); //停止
				Sleep(150);
				motor.stop(); //停止
				plan.speed_l = 0;
				plan.speed_r = 0;
				waittimer = 0;
			objectnowpos=objectnowpos+1;
			int nums_oba=0;
			for(;nums_oba<100;++nums_oba){
				if(zhanpin[nums_oba].objectnum==objectnums[objectnowpos])
				{
					break;
				}
			}

			plan.target_x=zhanpin[nums_oba].objectnum_x;
			plan.target_z=zhanpin[nums_oba].objectnum_y;//xhy目标点
			plan.speed_line_pos=0;
			plan.Desired_Angle_ob=zhanpin[nums_oba].direct;
			objectnow=nums_oba;
			isbegio=true;
			if(nums_oba!=0)
			{
				plan.SERVEMODE = 2;
			}
			plan.CtrlMode=2;
			FILE *alloutXXX;
		alloutXXX = fopen("alloutdaoda.txt","a+");
		fprintf(alloutXXX ," %d  %d  %d %d  %f %f  %f \n",objectnowpos,nums_oba,plan.target_x,plan.target_z);
		fclose(alloutXXX );
			}
		}
	}
	return 0;
}
//前进后退左转右转
 void CRecorder_ExampleDlg::OnBnClickedButton6(){//导航
	plan.speed_line = 0;
	plan.speed_angle = 0;
	plan.speed_l = 0;
	plan.speed_r = 0;
	gridfastslam_key=true;
	wait_gridfastslam.SetEvent();
	wait_motor_timer = 200;
	speedkey = 1;
	int nIndex = 0;
	CString strCBText;
	int nums_ob=0;
	for(;nums_ob<100;++nums_ob){
		if(zhanpin[nums_ob].objectnum==objectnums[0]){
			break;
		}
	}
	plan.target_x=zhanpin[nums_ob].objectnum_x;
	plan.target_z=zhanpin[nums_ob].objectnum_y;//xhy目标点
			
	plan.speed_line_pos=0;
	plan.Desired_Angle_ob=zhanpin[nums_ob].direct;
	objectnow=nums_ob;
	objectnowpos=0;
	plan.SERVEMODE = 2;
	plan.CtrlMode=2;
	if (!motor_key){
		motor_key = true;
		pThread_MototCtrl = AfxBeginThread(ThreaMotorCtrl,NULL); //电机控制线程
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
	if (eReti != PLAYER_ERR_NONE){
		hci_release();
		CString str;
		str.Format( "播放器初始化失败,错误码%d.", eReti);
		MessageBox( str );
	}
	pThread_Motor_autowalk= AfxBeginThread(ThreadComput_Motorautowalk,NULL);
}
void CRecorder_ExampleDlg::OnBnClickedButton3(){//后退
	ishand=true;
	motor.VectorMove(-1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
}
void CRecorder_ExampleDlg::OnBnClickedButton1(){//前进
	ishand=true;
	motor.VectorMove(1200,0.000000,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
}
void CRecorder_ExampleDlg::OnBnClickedButton4(){//左转
	ishand=true;
	motor.VectorMove(200,2.0,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
}
void CRecorder_ExampleDlg::OnBnClickedButton5(){//右转
	ishand=true;
	motor.VectorMove(200,-2.0,Info_robot.pianzhuan,Info_robot.pointrox,Info_robot.pointroy);
}
UINT ThreadReadLaser_Data(LPVOID lpParam){
	FILE *allout;
	allout = fopen("allout2laser.txt","a+");
	while (lase_key){
		m_cURG.GetDataByGD(0,768,1);
		WaitForSingleObject(m_cURG.wait_laser,INFINITE);

		//	m_cURG.GetDataByGD(0,768,1);//前两个参数决定扫描角度范围（384是正前方的线，288线为90度范围），最后一个参数决定了角度分辨率。获取激光测距起的数据;
		//////////////////////////////////////////
		Info_laser_data.m_Laser_Data_Point=m_nValPoint_temp;

		m_Laser_Data_Point_PostPro=m_nValPoint_temp;
		//	pReadThread_postpro->m_Laser_Point=m_nValPoint_temp;

		key_laser = !m_cURG.key;
		for (int i=0;i<Info_laser_data.m_Laser_Data_Point;i++){
			Info_laser_data.m_Laser_Data_Value[i]=m_cURG.m_distVal_temp_test[key_laser][i];
			m_laser_data_raw[i]=m_cURG.m_distVal_temp_test[key_laser][i];
			m_laser_data_postpro[i] = m_cURG.m_distVal_temp_test[key_laser][i];
			if(i%10==0){
				fprintf(allout," 0000===%d   \n",m_laser_data_postpro[i]);
			}
		}
		wait_data.SetEvent();
		m_cURG.wait_laser.ResetEvent();
		wait_laserpose.SetEvent();
	}
	return 0;
}
UINT ThreadDataExchange(LPVOID lpParam){
	while (dataexchange_key){
		//WaitForSingleObject(waittime,INFINITE);
		//WaitForSingleObject(waittime,3000);
		WaitForSingleObject(wait_data,INFINITE);
		plan.speed_stated = speed_stated;
		for (int loop = 0;loop<1000;loop++){
			plan.m_laser_data_postpro[loop] = m_laser_data_postpro[loop];
			m_laser_data_postpro_vfh[loop] = m_laser_data_postpro[loop];
		}
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
UINT ThreaVFH(LPVOID lpParam){
	plan.Init();
	algorithm.Init();
	plan.laser_position = false;
	while (vfh_key){
		QueryPerformanceFrequency(&freq1);
		QueryPerformanceCounter(&start_t1);
		WaitForSingleObject(wait_vfh,INFINITE);
		Pathplan();
		wait_motor.SetEvent();
		wait_vfh.ResetEvent();
		QueryPerformanceCounter(&stop_t1);
		exe_time1 = 1e3*(stop_t1.QuadPart-start_t1.QuadPart)/freq1.QuadPart;
		start_t1.QuadPart = stop_t1.QuadPart;
		//fprintf(file,"vfhtime:		%f    \n",exe_time1);
	}
	return 0;
}
void Pathplan(){
	double mm_x=Info_robot.pointrox;//将位置的毫米单位转换成厘米
	double mm_y=Info_robot.pointroy;
	double mm_angle=Info_robot.pianzhuan;
	//令偏转角弧度始终保持在一个周期内（0～2pi）；
	while(mm_angle>=Info_robot.pi*2)	mm_angle -= Info_robot.pi*2;
	while(mm_angle<0)	mm_angle+= Info_robot.pi*2;
	plan.PlanPath_vfh(mm_x,mm_y,mm_angle*180/PI);
}
UINT ThreaMotorCtrl(LPVOID lpParam){
	FILE *allout;
	allout = fopen("allout4.txt","a+");
	FILE *alloutxhy;
	alloutxhy = fopen("alloutxhy.txt","a+");
	//fprintf(allout,"角度：%f   l:  %f  r:  %f  z:   %f   \n",(double)dtheta, distancedif_l, distancedif_r,distancedif_z);
	//fclose(allout);
	while(motor_key){
	//	if (plan.danger)
		WaitForSingleObject(wait_motor,wait_motor_timer);
	//	else
	//		WaitForSingleObject(wait_motor,200);
		fprintf(allout,"%f    %f    %f    %f    \n",plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy);
		if (plan.CtrlMode == 1 || plan.CtrlMode == 3){
			if (CtrlMode_old != 3){
			//	SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				//motor.gomotor(0,0);
				CtrlMode_old = 3;
				//Sleep(300);
			}							
			motor.gomotor(plan.speed_l,-plan.speed_r,2*(plan.speed_r - plan.speed_l));

			fprintf(allout,"%d    %d    \n",plan.speed_l,plan.speed_r);
			/*Sleep(100);
			motor.gomotor(plan.speed_l,plan.speed_r,0);
			Sleep(100);
			motor.gomotor(plan.speed_l,plan.speed_r,0);*/
		}
		else if(plan.CtrlMode == 2){
			//plan.speed_l = 0;
			//plan.speed_r = 0;
			if (CtrlMode_old != 2){
			//	SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				CtrlMode_old = 2;
				//Sleep(300);
			}
			if (plan.speed_line >=0){
				run_direction = 1;
			}
			else if (plan.speed_line<0){
				run_direction = -1;
			}
			
			if (run_direction_old != run_direction){
				SpeedBuffer(0,0,&speed_l_old,&speed_r_old);
				run_direction_old = run_direction;
			//	Sleep(350);
			}
		//	motor.Velocity_control(plan.speed_line,plan.speed_angle);
			fprintf(allout," %d  %d %f  %f   line_speed：%f   angle_speed:  %f  Desired_Angle:  %f  pick:   %f   pianzhuan:  %f\n",plan.target1_x,plan.target1_z,Info_robot.pointrox,Info_robot.pointroy,plan.speed_line,plan.speed_angle,plan.Desired_Angle,pick,Info_robot.pianzhuan);
			if(!ishand){
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
void SpeedBuffer(int speed_l, int speed_r,int *speed_l_old, int *speed_r_old){
	do{
		if (!speedkey){
			return;
		}
		if (*speed_r_old < speed_r)	{
			*speed_r_old +=acceleration;
			if (*speed_r_old > speed_r){
				*speed_r_old = speed_r;
			}
		}
		else if (*speed_r_old > speed_r){
			*speed_r_old -=acceleration;
			if (*speed_r_old < speed_r){
				*speed_r_old = speed_r;
			}
		}
		if (*speed_l_old < speed_l){
			*speed_l_old +=acceleration;
			if (*speed_l_old > speed_l){
				*speed_l_old = speed_l;
			}
		}
		else if (*speed_l_old > speed_l){
			*speed_l_old -=acceleration;
			if (*speed_l_old < speed_l){
				*speed_l_old = speed_l;
			}
		}
		if (*speed_l_old<-5 && *speed_r_old<-5){
			if (*speed_l_old <-5){
				*speed_l_old = -5;
			}
			if (*speed_r_old < -5){
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
void CRecorder_ExampleDlg::OnTimer(UINT_PTR nIDEvent){
	switch (nIDEvent){
		case 2:{
			//判断是否到达目标点(200ms)
			if (plan.Range_to_go(plan.target_x,plan.target_z,Info_robot.pointrox,Info_robot.pointroy)){
				KillTimer(2);
				wait_motor_timer = 200*10000;
				plan.speed_line = 0;
				plan.speed_angle = 0;
				plan.speed_l = 0;
				plan.speed_r = 0;
				plan.SERVEMODE = 4; //切换到等待模式
				motor.stop(); //停止
				Sleep(150);
				motor.stop(); //停止
				plan.speed_l = 0;
				plan.speed_r = 0;
				waittimer = 0;	
			}
			break;
		}
		break;
	}
}
void CRecorder_ExampleDlg::OnBnClickedButton7(){//语音播放
	// TODO: 在此添加控件通知处理程序代码
	char* tts_motor=zhanpin[objectnumshand[objectnowhand]].contect1;
			unsigned char* pszUTF8 = NULL;
			HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

			string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
			PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
			++objectnowhand;
}
void CRecorder_ExampleDlg::OnBnClickedButton8(){//手动操控
	// TODO: 在此添加控件通知处理程序代码
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
	if (eReti != PLAYER_ERR_NONE){
		hci_release();
		CString str;
		str.Format( "播放器初始化失败,错误码%d.", eReti);
		MessageBox( str );
		
	}
}
void CRecorder_ExampleDlg::OnEnChangeEditWordlist(){

}
HBRUSH CRecorder_ExampleDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor){
	HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);
	pDC->SetBkMode(OPAQUE);
	pDC->SetTextColor(RGB(74,37,15));
	pDC->SetBkColor(RGB(178,136,80));
	hbr=CreateSolidBrush(RGB(178,136,80));
	// TODO:  在此更改 DC 的任何特性

	// TODO:  如果默认的不是所需画笔，则返回另一个画笔
	return hbr;
}
void CRecorder_ExampleDlg::OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct){
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	CDialog::OnDrawItem(nIDCtl, lpDrawItemStruct);	
}
 void CRecorder_ExampleDlg::OnSize(UINT nType, int cx, int cy){
	CDialog::OnSize(nType, cx, cy);
	CDialog::OnSize(nType, cx, cy);
	if(nType==1) return; //最小化则什么都不做 
	CWnd *pWnd; 
	pWnd = GetDlgItem(IDD_RECORDER_EXAMPLE_DIALOG); //获取控件句柄
	ChangeSize(pWnd,cx,cy); //调用changesize()函数
	pWnd = GetDlgItem(IDOK ); //获取控件句柄
	ChangeSize(pWnd,cx,cy);//调用changesize()函数
	pWnd = GetDlgItem(IDC_MFCBUTTON7);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON6);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON2);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON1);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON3);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON4);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON5);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON9);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON10);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON13);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON14);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON15);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON16);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_MFCBUTTON17);	ChangeSize(pWnd,cx,cy);
	pWnd = GetDlgItem(IDC_STATIC4);	ChangeSize(pWnd,cx,cy);
	//ChangeSize(pWnd,cx,cy)是一个自定义的函数，需要在类的protect属性中进行添加声明afx_msg void ChangeSize(CWnd * pWnd, int cx, int cy); 
	pWnd = GetDlgItem(IDC_EDIT_STATUS); ChangeSize(pWnd,cx,cy);
	GetClientRect(&m_rect); //将变化后的对话框设置为旧大小
}
void  CRecorder_ExampleDlg::ChangeSize(CWnd * pWnd, int cx, int cy){
	if (pWnd){
		CRect rect; 
		pWnd->GetWindowRect(&rect); //获取控件变化前的大小
		ScreenToClient(&rect);//将控件大小转换为在对话框中的区域坐标 
		rect.left=rect.left*cx/m_rect.Width();//调整控件大小 ，cx/m_rect.Width()为对话框在横向的变化比例
		rect.right=rect.right*cx/m_rect.Width(); //cx存储的是变化后的宽度，cy存储的是变化后的高度
		rect.top=rect.top*cy/m_rect.Height(); //m_rect.height()表示的是变化前主窗体的高度
		rect.bottom=rect.bottom*cy/m_rect.Height();
		pWnd->MoveWindow(rect);//设置控件大小
	}
}
void CRecorder_ExampleDlg::OnStnClickedStatic1(){
	// TODO: 在此添加控件通知处理程序代码
}
void CRecorder_ExampleDlg::OnBnClickedButton9(){
	//语音交互
	
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

	eRetasr = hci_asr_recorder_init( initConfig.c_str(), &call_back);
	if (eRetasr != RECORDER_ERR_NONE)
	{
		hci_release();
		strErrorMessage.Format( "录音机初始化失败,错误码%d", eRetasr);
		MessageBox( strErrorMessage );
		return ;
	}

	RECORDER_ERR_CODE eRet = RECORDER_ERR_NONE;
	
	GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( FALSE );
	GetDlgItem( IDC_BTN_CANCEL_RECORD )->EnableWindow( TRUE );	

	SetDlgItemText(IDC_EDIT1, "" );

    AccountInfo *account_info = AccountInfo::GetInstance();
    string startConfig = "";
	if (!IsDlgButtonChecked( IDC_ONLY_RECORDING ))
    {
     	startConfig += "capkey=" + account_info->cap_key();
	}
	startConfig += ",audioformat=pcm16k16bit";

    if (IsDlgButtonChecked( IDC_CONTINUE )){
        startConfig += ",continuous=yes";
    }
	if ( m_RecogMode == kRecogModeGrammar ){
		char chTmp[32] = {0};
		sprintf(chTmp,",grammarid=%d",m_GrammarId);
		startConfig += chTmp; 
	}

	if ( m_RecogMode == kRecogModeDialog ){
		startConfig +=",intention=weather;joke;story;baike;calendar;translation;news"; 
	}

	eRet = hci_asr_recorder_start(startConfig.c_str(),"");
	if (RECORDER_ERR_NONE != eRet){
		CString strErrMessage;
		strErrMessage.Format( "开始录音失败,错误码%d", eRet );
		MessageBox( strErrMessage );
		GetDlgItem( IDC_BTN_START_RECORD )->EnableWindow( TRUE );
		return;
	}

	pThread_speak_autotts= AfxBeginThread(ThreadComput_speakautotts,NULL);
}
 UINT ThreadComput_speakautotts(LPVOID lpParam){
	while(autotts_key){
		if(isautotts){
			isautotts=false;
			if(p_CR!=NULL){
				p_CR->OnBnClickedBtnCancelRecord();
				if(autotts_time_sleep<10){
					autotts_time_sleep=10;
				}
				Sleep(autotts_time_sleep*200);
				p_CR->OnBnClickedBtnStartRecord();

			}
		}
	}
	return 0;
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton1(){
	 // TODO: 在此添加控件通知处理程序代码
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton3(){
	 // TODO: 在此添加控件通知处理程序代码
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton5(){
	 // TODO: 在此添加控件通知处理程序代码
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton6(){
	 // TODO: 在此添加控件通知处理程序代码
	 zhanting=!zhanting;
	  ishand=false;
	  if(zhanting==true){
		  plan.SERVEMODE = 4;
//		  motor_key=false;
	  }
	  else{
//		  motor_key=true;
		  plan.SERVEMODE = 2;
	  }
	  
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton7(){
	 // TODO: 在此添加控件通知处理程序代码
	 if(p_CR!=NULL)
	{
		p_CR->OnBnClickedButton6();
	}
 }
 void CRecorder_ExampleDlg::OnBnClickedMfcbutton2(){
	 // TODO: 在此添加控件通知处理程序代码
	
	dlg= new CVoice;
	dlg->Create(IDD_DIALOG1);
	dlg->ShowWindow(SW_SHOW);

	
 }
 void CRecorder_ExampleDlg::OnBnClickedButton11(){
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

	char* p;
	CString str1;
	CString strTemp;
	p_DR->GetDlgItem(IDC_COMBO1)->GetWindowText(str1);
	if(str1==("黑龙江省博物馆开闭馆时间")){
		strTemp.Format(_T("每周二至周日开馆，周一全天闭馆，节假日除外。冬令时：10月8日-3月31日，9:00至16:00，15:00停止发票。夏令时：4月1日-10月7日，9:00至16:30，15:30停止发票。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("黑龙江省博物馆有几层，都有哪些展厅")){
		strTemp.Format(_T("黑龙江省博物馆共有3层。二层展厅主要有“自然陈列”、“黑龙江历史文物陈列”；一层为“黑龙江俄侨文化文物展”、“邓散木艺术专题陈列”、“每月一星”；负一层为“寒暑假特展”、“民俗展览”、“每月一县”三个临时展厅。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("黑龙江省博物馆镇馆之宝有哪些？")){
		strTemp.Format(_T("黑龙江省博物馆馆藏丰富，2014年举行了“十大镇馆之宝评选”活动，“十大镇馆之宝”分别是：金代铜坐龙、金代齐国王墓丝织品服饰、南宋《蚕织图》、唐代渤海天门军之印、披毛犀化石骨架、南宋《兰亭序》图卷、黑龙江满洲龙、金代山水人物故事镜、松花江猛犸象化石骨架、新石器时代桂叶形石器。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("免费讲解")){
		strTemp.Format(_T("上午9：30开讲，下午14:30开讲。开讲展厅地点为二楼自然陈列展厅。"));
		p = (char*)(LPCTSTR)strTemp;

	}
		if(str1==("黑龙江省博物馆镇馆之宝有哪些？")){
		strTemp.Format(_T("黑龙江省博物馆馆藏丰富，2014年举行了“十大镇馆之宝评选”活动，“十大镇馆之宝”分别是：金代铜坐龙、金代齐国王墓丝织品服饰、南宋《蚕织图》、唐代渤海天门军之印、披毛犀化石骨架、南宋《兰亭序》图卷、黑龙江满洲龙、金代山水人物故事镜、松花江猛犸象化石骨架、新石器时代桂叶形石器。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("洗手间在哪")){
		strTemp.Format(_T("女士洗手间位于二楼楼梯口处，男士洗手间位于一楼楼梯口处。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("便民服务")){
		strTemp.Format(_T("针对残障人士，在博物馆内凭身份证免费租借轮椅，方便参观；为观众提供免费寄存服务。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("文创天地")){
		strTemp.Format(_T("黑龙江省博物馆还设有“龙博书苑”，“文化创意经营中心”、“水吧”等。"));
		p = (char*)(LPCTSTR)strTemp;

	}
		if(str1==("社会服务项目")){
		strTemp.Format(_T("文物及古动物化石的鉴定、修复、复制、咨询。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("黑龙江省博物馆有哪些活动？")){
		strTemp.Format(_T("省博举办诸多丰富多彩的活动内容，有“相约龙博”科普教育活动、“环球自然日——青少年自然科学知识挑战赛”、“青少年科普绘画大赛”、“流动博物馆”等。您可以扫描屏幕上方的二维码实时关注我们，工作人员会在“黑龙江省博物馆互动平台”上发布展陈信息及活动内容。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("黑龙江省博物馆“相约龙博”课堂 ")){
		strTemp.Format(_T("“相约龙博”课堂，旨让青少年在课余时间能在愉快地氛围中收获知识，增长能力，培养青少年的综合素质。工作人员根据省博馆藏资源精心策划活动内容，其中包括：“历史的记忆”、“动物大联盟”、“走进传承”、“玩艺坊”、“小花匠的植物王国”、“物质世界的真相”六大系列。“相约龙博”课堂设在二楼大厅，每逢周末及节假日都会组织开展精彩的活动内容。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("环球自然日——青少年自然科学知识挑战赛")){
		strTemp.Format(_T("“环球自然日——青少年自然科学知识挑战赛”，是由美国著名慈善家肯尼斯•尤金•贝林创办，环球健康与教育基金会发起，用以激发中小学生对于自然科学的兴趣，并提高其研究、分析和交往能力的课外科普教育活动。此项活动于2012年进入中国，2014年起黑龙江赛区启动，由省博承办。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("环球自然日——青少年科普绘画大赛")){
		strTemp.Format(_T("“环球自然日——青少年科普绘画大赛”旨在带动更多的青少年走进博物馆，通过结合博物馆的资源优势，让他们近距离观察并研究相关自然科学知识，同时，将自然和艺术融合，使他们在活动过程中感受自然之美，激发自然科学的学习热情。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("黑龙江省博物馆流动博物馆")){
		strTemp.Format(_T("黑龙江省博物馆自2014年起成立流动博物馆，将展览带到大众身边，足不出户就能够看到展览。截止目前，有“远离毒品、远离邪教、远离赌博、倡导绿色上网”、“昆虫世界中的铠甲勇士——锹甲”、“黑龙江省中药材特展”三个主题展览，曾走进多所院校、社区等，深受广大群众好评。"));
		p = (char*)(LPCTSTR)strTemp;

	}
	if(str1==("黑龙江省文博志愿者基地")){
		strTemp.Format(_T("为更好的发挥博物馆的社会教育功能，更好地为社会大众提供服务，也为各大热心于博物馆和社会服务事业的志愿者提供一个实现社会价值和个人价值的平台，黑龙江省博物馆于2010年成立了“黑龙江省文博志愿者基地”。志愿者服务分为导览志愿者及讲解志愿者，除此之外，他们的身影经常出现在各项活动当中，一直以来深受广大观众的好评。"));
	 p = (char*)(LPCTSTR)strTemp;
	
	}
	if(str1==("博物馆简介")){
		strTemp.Format(_T("黑龙江省博物馆是省级综合性博物馆，2012年被评为国家一级博物馆，是黑龙江省收藏历史文物、艺术品和动、植物标本的中心，是地方史和自然生态的研究中心之一，也是宣传地方历史文化和自然资源的重要场所。"));
		p = (char*)(LPCTSTR)strTemp;

	}

	char* tts_motor = p;
	unsigned char* pszUTF8 = NULL;
	HciExampleComon::GBKToUTF8( (unsigned char*)tts_motor, &pszUTF8 );

	string startConfig = "property=cn_xiaokun_common,tagmode=none,capkey=tts.cloud.wangjing";
			
	PLAYER_ERR_CODE eRetk = hci_tts_player_start( (const char*)pszUTF8, startConfig.c_str() );
	hci_release();		

 }
 void CRecorder_ExampleDlg::OnBnClickedButton12(){
	 // TODO: 在此添加控件通知处理程序代码
	 CString strCaption = "";
	GetDlgItemText( IDC_BUTTON12, strCaption );
	if( strCaption == "暂停" ){
		PLAYER_ERR_CODE eRet = hci_tts_player_pause();
		if( eRet != PLAYER_ERR_NONE )
		{
			CString str;
			str.Format( "暂停播放失败,错误码%d.", eRet);
			MessageBox( str );
			return;
		}
		SetDlgItemText( IDC_BUTTON12, "继续" );
	}
	else{
		PLAYER_ERR_CODE eRet = hci_tts_player_resume();
		if( eRet != PLAYER_ERR_NONE ){
			CString str;
			str.Format( "继续播放失败,错误码%d.", eRet );
			MessageBox( str );
			return;
		}
		SetDlgItemText(IDC_BUTTON12, "暂停" );
	}
 }