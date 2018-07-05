
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

typedef struct player_position2d_cmd_pos //机器人状态数据
{
	/** position [m,m,rad] (x, y, yaw)*/
	player_pose_t pos;
	/** velocity at which to move to the position [m/s] or [rad/s] */
	player_pose_t vel;
	/** Motor state (FALSE is either off or locked, depending on the driver). */
	int state;
} player_position2d_cmd_pos_t;



typedef struct player_position2d_data//机器人电机状态数据
{
	/** position [m,m,rad] (x, y, yaw)*/
	player_pose_t pos;
	/** translational velocities [m/s,m/s,rad/s] (x, y, yaw)*/
	player_pose_t vel;
	/** Are the motors stalled? */
	int stall;
} player_position2d_data_t;


typedef struct player_position2d_cmd_vel//机器人速度
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

	enum    Direction{LEFT,RIGHT,TARGET};  //判断机器人避障时的转向以及机器人是否进行搜索目标行为
	Direction direct; 

	//CMotorTest m_pmotor;//电机测试类
	//void  SetTarget(float x,float z,float t);   //设置目标
	//void  SetCurrentZuobiao(float x,float z,float angle);  //设置机器人当前坐标
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

	void  TowardTarget(int x,int z);   //朝向目标前进
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
	int distance; //目标点距离
	int speed_stated;
	int speed_mode;

	int speed_l; //上传左轮速度
	int speed_r; //上传右轮速度

	int speed_high;
	int speed_meidum;
	int speed_low;

	bool danger; //true--避障时需要后退转动
	//栅格下的坐标 ,除了角度外都是int型
	double Scene_scale_x; //栅格地图场景比例
	double Scene_scale_y;
	int SERVEMODE; //服务模式,0-停止服务;1-漫游送餐;2指定餐桌送餐
	int target_home_x;  //起始点坐标
	int target_home_y;
	int target_home_t;

	int target1; //目标点状态
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

	int temp_target_x;    //用来表示一个临时的目标点
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
	//避障转动躲避危险等级
	int dangerdistance0; //大幅度转动
	int dangerdistance1; //中幅度转动
	int dangerdistance2; //轻微转动

	int distance_min[12];

	//减速距离级别
	int speeddistance0; //小于低速距离
	int speeddistance1; //小于中速距离 大于为高速 

	bool laser_position; //激光位置 ture--上边 false--下边

	int DangerGrade[12]; //判断结果
	int target_num; //到达的目标点计数

	float Desired_Angle;
	float speed_line;
	float speed_angle;
	bool Dange_for_robot;
	int CtrlMode; //1-左右轮速度控制模式;2-线速度角速度模式
	int CtrlMode_old;

	///////////////////////xhy
	int speed_line_pos;//判断机器人是否从静止开始运动
	float Desired_Angle_ob;//展品方向
	////////////////////
	/***********************************************************************
	int tablenum = 0;
	int dangerdistance0 = 120; //转向距离
	int dangerdistance1 = 220; //减速距离
	int dangerdistance2 = 320; //安全距离
	int DangerGrade[12] = {0}; //判断结果         
	int target_num = 0; //到达的目标点计数

	float Desired_Angle = 0.0;
	float speed_line = 0.0;
	float speed_angle = 0.0;
	bool Dange_for_robot=false;
	/************************************************************************/
	

//	FILE *file;

	int mapp[LENGTH_x][LENGTH_y];  //代表栅格地图   0无障碍   1终点 2 代表机器人走过的路径  
	//3代表障碍物正下方     4障碍物左右边缘    5障碍物上边缘
	//100*100个栅格,5m*5m
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
	
	//int m_nTurnRightFlag; //转向标志位
	//int m_nTurnLeftFlag;  //转向标志位

	int m_nObstacleOnLeft;//最近障碍物在左侧标志
	int m_nObstacleOnRight;//最近障碍物在右侧标志
	int m_nDangerDir;     //判断障碍类型依据之一
	int TestDanger(void); //判断在一定范围内有无障碍，返回障碍类型，无障碍则返回-1
	
	void MyFtestDanger(void); //障碍判定

	void Compute_to_target_angle(int target_x,int target_z);
	void Beam_Action(); //根据接收的数据，做出行动

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



	void SetCurrentZuobiao(float x,float z,float angle);  //设置机器人当前坐标

	void TransformToGrid(float x,float z,int *grid_x,int *grid_z);
	void PixelTransformToGrid(float x,float z,int *grid_x,int *grid_z,double RealPositionToPixel_x,double RealPositionToPixel_y);
	
	bool Range_to_go(int target_x,int target_y,int robot_x,int robot_y); //判断是否到达目标点
//	void Direct_to_target(float Desired_Angle);

	void SpeedCompute(int raidus, int wheel_gauge,int *speed_l,int *speed_r,int direction); //根据转弯半径计算左右轮速

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
