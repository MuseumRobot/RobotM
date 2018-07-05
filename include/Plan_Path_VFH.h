
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PATHPLAN_H__98EC49C2_F292_4EA0_8688_D8413860380F__INCLUDED_)
#define AFX_PATHPLAN_H__98EC49C2_F292_4EA0_8688_D8413860380F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "Global_Variable.h"

#include "vfh_algorithm.h"

#define PI 3.1415926
#define delta_X 0
#define delta_Y 20
#define LENGTH_x 1000
#define LENGTH_y 1000
#define Scenario_Width 20000
#define Scenario_Height 20000
#define  eposon 0.01
#define SEED 1
#define delta_distance_target 500
#define near_delta_distance_target 800
#define Start_Pose_x 138
#define Start_Pose_y 0
#define Start_Pose_angle 0.01



struct position{
	int    x ;
	int    y ;
	int    position_number;
	float theta;
};


/** @brief A pose in the plane */
typedef struct player_pose
{
	/** X [m] */
	float px;
	/** Y [m] */
	float py;
	/** yaw [rad] */
	float pa;
} player_pose_t;

typedef struct player_position2d_cmd_pos //������״̬����
{
	/** position [m,m,rad] (x, y, yaw)*/
	player_pose_t pos;
	/** velocity at which to move to the position [m/s] or [rad/s] */
	player_pose_t vel;
	/** Motor state (FALSE is either off or locked, depending on the driver). */
	int state;
} player_position2d_cmd_pos_t;



typedef struct player_position2d_data//�����˵��״̬����
{
	/** position [m,m,rad] (x, y, yaw)*/
	player_pose_t pos;
	/** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
	player_pose_t vel;
	/** Are the motors stalled? */
	int stall;
} player_position2d_data_t;


typedef struct player_position2d_cmd_vel//�������ٶ�
{
	/** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
	player_pose_t vel;
	/** Motor state (FALSE is either off or locked, depending on the driver). */
	int state;
} player_position2d_cmd_vel_t;





class CPlan_Path_VFH
{
public:
	CPlan_Path_VFH(void);
	~CPlan_Path_VFH(void);

	void  Init();

	enum    Direction{LEFT,RIGHT,TARGET};  //�жϻ����˱���ʱ��ת���Լ��������Ƿ��������Ŀ����Ϊ
	Direction direct; 

	//CMotorTest m_pmotor;//���������
	//void  SetTarget(float x,float z,float t);   //����Ŀ��
	//void  SetCurrentZuobiao(float x,float z,float angle);  //���û����˵�ǰ����
	//
	//void  TransformToGrid(float x,float z,int *grid_x,int *grid_z);
	//void  PixelTransformToGrid(float x,float z,int *grid_x,int *grid_z,double RealPositionToPixel_x,double RealPositionToPixel_y);

	//HANDLE hdlWrite;	

	int m_vfh[72];
	int target_angle;

	void  SearchObstacle();  
	void  Decision();  
	void VFH_Star_Decision();
//	void  CreateFWA();   //create free walking area
//	void  CreateOWA();   //create obstacle area

	void  Front();
	void  Reversion();
	void  Clockwise();
	void  Stop();

	void  TowardTarget(int x,int z);   //����Ŀ��ǰ��
	void  MoveTo(int x,int z,bool left);
	void  PlanPath(float m_x,float m_y,float m_angle);
	//void  ComputeObstacle(float x,float z);
	void  PrintMap();
	void  TurnAngle(float angle);
	void  TurnAngle_small(float angle);
	bool  GetFangXiangCha(int x,int z,bool left);
	void  GrabVH();

	void ComputerObstacle(double x,double z);

	//RobotInfo
	double pi;
	int Drobot;
	int zuolunfangxiang;
	int youlunfangxiang;
	int zuolunjuli;
	int youlunjuli;
	double pianzhuan;
	float pointrox;
	float pointroy;
	double pian;
	int distance; //Ŀ������
	int speed_stated;
	int speed_mode;

	int speed_l; //�ϴ������ٶ�
	int speed_r; //�ϴ������ٶ�

	int speed_high;
	int speed_meidum;
	int speed_low;

	bool danger; //true--����ʱ��Ҫ����ת��
	//դ���µ����� ,���˽Ƕ��ⶼ��int��
	double Scene_scale_x; //դ���ͼ��������
	double Scene_scale_y;
	int SERVEMODE; //����ģʽ,0-ֹͣ����;1-�����Ͳ�;2ָ�������Ͳ�
	int target_home_x;  //��ʼ������
	int target_home_y;
	int target_home_t;

	int target1; //Ŀ���״̬
	int target2;
	int target3;

	int	target_x;    
	int	target_z;

	int	target1_x;    
	int	target1_z;

	int	target2_x;    
	int	target2_z;

	int	target3_x;    
	int	target3_z;

	float target_t;

	int temp_target_x;    //������ʾһ����ʱ��Ŀ���
	int temp_target_z;
	bool temp_target_arrive;

	int	current_x;
	int	current_z;
	float current_angle;

	int waittimer;
	int plate_wight;
	int socktime;

	float speeddamping_l;
	float speeddamping_r;

	int zuo1,you1;

	double Grid_Scale_x;
	double Grid_Scale_y;
	double RealPositionToPixel_x;
	double RealPositionToPixel_y;

	double Angleofrobot_to_robottotarget;

	int SonarData[9];
	int PlateData[3];
	
	int tablenum;
	//����ת�����Σ�յȼ�
	int dangerdistance0; //�����ת��
	int dangerdistance1; //�з���ת��
	int dangerdistance2; //��΢ת��

	int distance_min[12];

	//���پ��뼶��
	int speeddistance0; //С�ڵ��پ���
	int speeddistance1; //С�����پ��� ����Ϊ���� 

	bool laser_position; //����λ�� ture--�ϱ� false--�±�

	int DangerGrade[12]; //�жϽ��
	int target_num; //�����Ŀ������

	float Desired_Angle;
	float speed_line;
	float speed_angle;
	bool Dange_for_robot;
	int CtrlMode; //1-�������ٶȿ���ģʽ;2-���ٶȽ��ٶ�ģʽ
	int CtrlMode_old;

	///////////////////////xhy
	int speed_line_pos;//�жϻ������Ƿ�Ӿ�ֹ��ʼ�˶�
	float Desired_Angle_ob;//չƷ����
	////////////////////
	/***********************************************************************
	int tablenum = 0;
	int dangerdistance0 = 120; //ת�����
	int dangerdistance1 = 220; //���پ���
	int dangerdistance2 = 320; //��ȫ����
	int DangerGrade[12] = {0}; //�жϽ��         
	int target_num = 0; //�����Ŀ������

	float Desired_Angle = 0.0;
	float speed_line = 0.0;
	float speed_angle = 0.0;
	bool Dange_for_robot=false;
	/************************************************************************/
	

//	FILE *file;

	int mapp[LENGTH_x][LENGTH_y];  //����դ���ͼ   0���ϰ�   1�յ� 2 ����������߹���·��  
	//3�����ϰ������·�     4�ϰ������ұ�Ե    5�ϰ����ϱ�Ե
	//100*100��դ��,5m*5m
	int  mcount[LENGTH_x][LENGTH_y];
	//static const int BiZhang_JuLi;
	int BiZhang_JuLi;
	static const int SPEED_VALUE;
	//static const double PI;

	double  m_x;
	double  m_y;
	double  m_angle;
	int m_Scenario_Height;
	int m_Scenario_Width;
	//////////////////////////////////
	int inDist[1000];
	int inNum;
	int m_DirFlag; 
	
	//int m_nTurnRightFlag; //ת���־λ
	//int m_nTurnLeftFlag;  //ת���־λ

	int m_nObstacleOnLeft;//����ϰ���������־
	int m_nObstacleOnRight;//����ϰ������Ҳ��־
	int m_nDangerDir;     //�ж��ϰ���������֮һ
	int TestDanger(void); //�ж���һ����Χ�������ϰ��������ϰ����ͣ����ϰ��򷵻�-1
	
	void MyFtestDanger(void); //�ϰ��ж�

	void Compute_to_target_angle(int target_x,int target_z);
	void Beam_Action(); //���ݽ��յ����ݣ������ж�

	void FrontBeam_laser();  
	void  Decision_Beam(); 
	double robotsize;

	////////////////////////////
	double dmax_robot_laPoint;
	double b_constant;
	double C_certainty_value;
	double a_VFH_Constant;      
	double robot_to_target;
	double alpha;
	double prevAngle;
	double angle;

	int m_laser_data_postpro[1000];

	float Desired_Angle_abs; 



	void PlanPath_vfh(float m_x,float m_y,float m_angle);


	int SetupVFHPLUS();
	void basedVFHPLUS(void);
	void ProcessCommand( player_position2d_cmd_pos_t &);
	void ProcessOdom( player_position2d_data_t &);

	void PutCommand( float cmd_speed, float cmd_turnrate );



	void SetCurrentZuobiao(float x,float z,float angle);  //���û����˵�ǰ����

	void TransformToGrid(float x,float z,int *grid_x,int *grid_z);
	void PixelTransformToGrid(float x,float z,int *grid_x,int *grid_z,double RealPositionToPixel_x,double RealPositionToPixel_y);
	
	bool Range_to_go(int target_x,int target_y,int robot_x,int robot_y); //�ж��Ƿ񵽴�Ŀ���
//	void Direct_to_target(float Desired_Angle);

	void SpeedCompute(int raidus, int wheel_gauge,int *speed_l,int *speed_r,int direction); //����ת��뾶������������

	VFH_Algorithm vfh_Algorithm;
private:
	bool active_goal;
	bool turninginplace;
	double odom_pose[3];
	double odom_vel[3];


	double con_vel[3];

	double dist_eps;
	// how fast and how long to back up to escape from a stall
	double escape_speed;
	double escape_max_turnspeed;
	double escape_time;

	// Stall flag and counter
	int odom_stall;

	float speed, turnrate;

	int cmd_state;

	int cmd_type;

	double angle_diff(double a, double b);

};

#endif // !defined(AFX_PATHPLAN_H__98EC49C2_F292_4EA0_8688_D8413860380F__INCLUDED_)
