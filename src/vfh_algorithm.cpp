/*
*  Orca-Components: Components for robotics.
*  
*  Copyright (C) 2004
*  
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*  
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#include "stdafx.h"
#include "vfh_algorithm.h"

#include <stdio.h>
#include <assert.h>
#include <math.h>
////////////////////////////////
#include <float.h>
#include <complex>
/////////////////////////////////////
#include <windows.h>
#include <mmsystem.h>

//#include "Global_Variable.h"

//#include "Plan_Path_VFH.h"


int m_laser_data_postpro_vfh[1000];
double vfh_Scene_scale_x;
double vfh_Scene_scale_y;
//extern MAP map_world;
bool In_front_of_the_barrier = true;
bool no_angle = false;
bool all_angle = false;
int MinSpeed;
int speed_stated_vfh;

double Obstacle_Distance_init;/////////////////////////初始避障距离设定
double delt_Obstacle_Distance_init;
double free_Obstacle_Distance_init;

extern struct robotinfo Info_robot;
extern CPlan_Path_VFH plan;
///////////////////////////////////////////////////
/******************************************************************************\
对一个复数 x 开 n 次方
\******************************************************************************/
std::complex<double> sqrtn(const std::complex<double>&x,double n)
{
	double r = _hypot(x.real(),x.imag()); //模
	if(r > 0.0)
	{
	double a = atan2(x.imag(),x.real()); //辐角
	n = 1.0 / n;
	r = pow(r,n);
	a *= n;
	return std::complex<double>(r * cos(a),r * sin(a));
	}
	return std::complex<double>();
}
/******************************************************************************\
使用费拉里法求解一元四次方程 a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
\******************************************************************************/
void Ferrari(std::complex<double> x[4]
,std::complex<double> a
,std::complex<double> b
,std::complex<double> c
,std::complex<double> d
,std::complex<double> e)
{
	a = 1.0 / a;
	b *= a;
	c *= a;
	d *= a;
	e *= a;
	std::complex<double> P = (c * c + 12.0 * e - 3.0 * b * d) / 9.0;
	std::complex<double> Q = (27.0 * d * d + 2.0 * c * c * c + 27.0 * b * b * e - 72.0 * c * e - 9.0 * b * c * d) / 54.0;
	std::complex<double> D = sqrtn(Q * Q - P * P * P,2.0);
	std::complex<double> u = Q + D;
	std::complex<double> v = Q - D;
	if(v.real() * v.real() + v.imag() * v.imag() > u.real() * u.real() + u.imag() * u.imag())
	{
	u = sqrtn(v,3.0);
	}
	else
	{
	u = sqrtn(u,3.0);
	}
	std::complex<double> y;
	if(u.real() * u.real() + u.imag() * u.imag() > 0.0)
	{
	v = P / u;
	std::complex<double> o1(-0.5,+0.86602540378443864676372317075294);
	std::complex<double> o2(-0.5,-0.86602540378443864676372317075294);
	std::complex<double>&yMax = x[0];
	double m2 = 0.0;
	double m2Max = 0.0;
	int iMax = -1;
		for(int i = 0;i < 3;++i)
		{
		y = u + v + c / 3.0;
		u *= o1;
		v *= o2;
		a = b * b + 4.0 * (y - c);
		m2 = a.real() * a.real() + a.imag() * a.imag();
			if(0 == i || m2Max < m2)
			{
			m2Max = m2;
			yMax = y;
			iMax = i;
			}
		}
	y = yMax;
	}
	else
	{//一元三次方程，三重根
	y = c / 3.0;
	}
	std::complex<double> m = sqrtn(b * b + 4.0 * (y - c),2.0);
	if(m.real() * m.real() + m.imag() * m.imag() >= DBL_MIN)
	{
	std::complex<double> n = (b * y - 2.0 * d) / m;
	a = sqrtn((b + m) * (b + m) - 8.0 * (y + n),2.0);
	x[0] = (-(b + m) + a) / 4.0;
	x[1] = (-(b + m) - a) / 4.0;
	a = sqrtn((b - m) * (b - m) - 8.0 * (y - n),2.0);
	x[2] = (-(b - m) + a) / 4.0;
	x[3] = (-(b - m) - a) / 4.0;
	}
	else
	{
	a = sqrtn(b * b - 8.0 * y,2.0);
	x[0] =
	x[1] = (-b + a) / 4.0;
	x[2] =
	x[3] = (-b - a) / 4.0;
	}
}


/////////////////////////////////////////////////////

VFH_Algorithm::VFH_Algorithm( double cell_size,
	int window_diameter,
	int sector_angle,
	double safety_dist_0ms,
	double safety_dist_1ms, 
	int max_speed,
	int max_speed_narrow_opening,
	int max_speed_wide_opening,
	int max_acceleration,
	int min_turnrate,
	int max_turnrate_0ms,
	int max_turnrate_1ms,

	double min_turn_radius_safety_factor,
	double free_space_cutoff_0ms,
	double obs_cutoff_0ms,
	double free_space_cutoff_1ms,
	double obs_cutoff_1ms,
	double weight_desired_dir,
	double weight_current_dir ):
CELL_WIDTH(cell_size),
	WINDOW_DIAMETER(window_diameter),
	SECTOR_ANGLE(sector_angle),
	SAFETY_DIST_0MS(safety_dist_0ms),
	SAFETY_DIST_1MS(safety_dist_1ms),
	Current_Max_Speed(max_speed),
	MAX_SPEED(max_speed),
	MAX_SPEED_NARROW_OPENING(max_speed_narrow_opening),
	MAX_SPEED_WIDE_OPENING(max_speed_wide_opening),
	MAX_ACCELERATION(max_acceleration),
	MIN_TURNRATE(min_turnrate),
	MAX_TURNRATE_0MS(max_turnrate_0ms),
	MAX_TURNRATE_1MS(max_turnrate_1ms),
	MIN_TURN_RADIUS_SAFETY_FACTOR(min_turn_radius_safety_factor),
	Binary_Hist_Low_0ms(free_space_cutoff_0ms),
	Binary_Hist_High_0ms(obs_cutoff_0ms),
	Binary_Hist_Low_1ms(free_space_cutoff_1ms),
	Binary_Hist_High_1ms(obs_cutoff_1ms),
	U1(weight_desired_dir),
	U2(weight_current_dir),
	Desired_Angle(0),
	Picked_Angle(0),
	Last_Picked_Angle(Picked_Angle),
	last_chosen_speed(0)
{
	this->Last_Binary_Hist = NULL;
	this->Hist = NULL;
	this->Hist_Average_Distance=NULL;
	if ( SAFETY_DIST_0MS == SAFETY_DIST_1MS )
	{
		// For the simple case of a fixed safety_dist, keep things simple.
		NUM_CELL_SECTOR_TABLES = 1;  
	}
	else
	{
		// AB: Made this number up...
		NUM_CELL_SECTOR_TABLES = 4;
	}

}




VFH_Algorithm::~VFH_Algorithm()
{
	if(this->Hist)
		delete[] Hist;
	if(this->Last_Binary_Hist)
		delete[] Last_Binary_Hist;

	if(this->Hist_Average_Distance)
		delete[] Hist_Average_Distance;

//	fclose(outfile);

}

int VFH_Algorithm::GetMaxTurnrate( int speed )//获得最大旋转速度
{ 
	int val = ( MAX_TURNRATE_0MS + (int)(speed*( MAX_TURNRATE_1MS-MAX_TURNRATE_0MS )/10/*MAX_SPEED*/) );//10太小，应该改为最大容许速度，max_speed

	if ( val < 0 )
		val = 0;

	return val;
}

void VFH_Algorithm::SetCurrentMaxSpeed( int max_speed )//设置当前最大旋转速度
{
	this->Current_Max_Speed = MIN( max_speed, this->MAX_SPEED );
	this->Min_Turning_Radius.resize( Current_Max_Speed+1 );//最小旋转半径列表，不同速度对应不同的旋转半径

	// small chunks of forward movements and turns-in-place used to
	// estimate turning radius, coz I'm too lazy to screw around with limits -> 0.
	double dx, dtheta;

	//
	// Calculate the turning radius, indexed by speed.
	// Probably don't need it to be precise (changing in 1mm increments).
	//
	// WARNING: This assumes that the max_turnrate that has been set for VFH is
	//          accurate.
	//
	for(int x=0;x<=Current_Max_Speed;x++) 
	{
		dx = (double) x / 1e5; // dx in m/millisec转化为单位米/毫秒
		dtheta = ((M_PI/180)*(double)(GetMaxTurnrate(x))) / 1000.0; // dTheta in radians/millisec 旋转速度
		Min_Turning_Radius[x] = (int) ( ((dx / tan( dtheta ))*1000.0) * MIN_TURN_RADIUS_SAFETY_FACTOR ); // in cm旋转半径*最小旋转安全因子
	}
}


// Doesn't need optimization: only gets called once per update.无需优化
int VFH_Algorithm::Get_Speed_Index( float speed )
{
	int val = (int) floor(((float)speed/(float)Current_Max_Speed)*NUM_CELL_SECTOR_TABLES);//取下限值

	if ( val >= NUM_CELL_SECTOR_TABLES )
		val = NUM_CELL_SECTOR_TABLES-1;

	// printf("Speed_Index at %dmm/s: %d\n",speed,val);

	return val;
}

// Doesn't need optimization: only gets called on init plus once per update.
int  
	VFH_Algorithm::Get_Safety_Dist( float speed )//获取安全距离
{
	int val = (int) ( SAFETY_DIST_0MS + (int)(speed*( SAFETY_DIST_1MS-SAFETY_DIST_0MS )/50.0) );

	if ( val < 0 )
		val = 0;

	// printf("Safety_Dist at %dmm/s: %d\n",speed,val);

	return val;
}

// AB: Could optimize this with a look-up table, but it shouldn't make much 
//     difference: only gets called once per sector per update.每个扇区每次更新只调用一次，使用查询表优化很难
float
	VFH_Algorithm::Get_Binary_Hist_Low( float speed )//获取二元直方图最低值
{
	return ( Binary_Hist_Low_0ms - (speed*( Binary_Hist_Low_0ms-Binary_Hist_Low_1ms )/(2*Current_Max_Speed)) );
}

// AB: Could optimize this with a look-up table, but it shouldn't make much 
//     difference: only gets called once per sector per update.
float
	VFH_Algorithm::Get_Binary_Hist_High( float speed )//获取二元直方图最高值
{
	return ( Binary_Hist_High_0ms - (speed*( Binary_Hist_High_0ms-Binary_Hist_High_1ms )/(2*Current_Max_Speed)) );
}


int VFH_Algorithm::Init()//VFH算法初始化
{
	
	FILE *outfile31;

//	outfile=fopen("oooooo.txt","w+");
	outfile31=fopen("zzqq_pos31.txt","w+");

//	outfilehist = fopen("hist.txt","w+");
//	outfile3 = fopen("hist1.txt","w+");
	////////////////////////////////////////////初始避障距离设定，可以通过主程序的界面设定，并传递到该程序中
	/////////////////////////////////////////////
//	Obstacle_Distance_init=800;/////////////////////////zcs 
//	delt_Obstacle_Distance_init=200;
//	free_Obstacle_Distance_init=1500;
	////////////////////////////////////////////
	vector <CvPoint2D64f> vec;
	CvPoint2D64f Point_point_temp;


	int ii,jj,m,n=8,poly_n=2;
	index_poly[0]=0;
	index_poly[1]=0;
	index_poly[2]=0;
	index_poly[3]=0;
	index_poly[4]=0;

	/////////////////////////以下参数需要预先手动测量,采用MATLAB中的函数polyfit(x,y,4);直接获得四阶多项式系数（降幂分布）
	double index_poly0=0, index_poly1=0,index_poly2=0,index_poly3=0,index_poly4=0;
	double xxx[]={3150,2940,2750,2220,1910,1660,1380,1120,940,750,545,375,234};/////////////////障碍物和激光器之间的距离，mm
	double yyy[]={17.846,21972,155598,7661305,23121884,50200508,108400552,193907584,271911456,373604384,481242016,607557680,661777728};

	
	index_poly4=-0.00001937;
	index_poly3=0.0984;
	index_poly2=11.4544;
	index_poly1=-672920;
	index_poly0=832200000;

	index_poly[0]=index_poly0;
	index_poly[1]=index_poly1;
	index_poly[2]=index_poly2;
	index_poly[3]=index_poly3;
	index_poly[4]=index_poly4;
	///////////////////////////////////
	
///	 fittingCurve(n,xx,yy,poly_n,index_poly); 四阶多项式采用matlab手动计算系数，

	 Binary_Hist_High_0ms=index_poly0+index_poly1*Obstacle_Distance_init+index_poly2*pow(Obstacle_Distance_init,2)+index_poly3*pow(Obstacle_Distance_init,3)+index_poly4*pow(Obstacle_Distance_init,4);
	 Binary_Hist_High_1ms=index_poly0+index_poly1*(Obstacle_Distance_init+delt_Obstacle_Distance_init)+index_poly2*pow((Obstacle_Distance_init+delt_Obstacle_Distance_init),2)+index_poly3*pow((Obstacle_Distance_init+delt_Obstacle_Distance_init),3)+index_poly4*pow((Obstacle_Distance_init+delt_Obstacle_Distance_init),4);
	
//	 Binary_Hist_Low_0ms=index_poly[0]+index_poly[1]*free_Obstacle_Distance_init+index_poly[2]*pow(free_Obstacle_Distance_init,2);
//	 Binary_Hist_Low_1ms=index_poly[0]+index_poly[1]*(free_Obstacle_Distance_init+delt_Obstacle_Distance_init)+index_poly[2]*pow((free_Obstacle_Distance_init+delt_Obstacle_Distance_init),2);

	 Binary_Hist_Low_0ms=index_poly0+index_poly1*free_Obstacle_Distance_init+index_poly2*pow(free_Obstacle_Distance_init,2)+index_poly3*pow(free_Obstacle_Distance_init,3)+index_poly4*pow(free_Obstacle_Distance_init,4);
	 Binary_Hist_Low_1ms=index_poly0+index_poly1*(free_Obstacle_Distance_init+delt_Obstacle_Distance_init)+index_poly2*pow((free_Obstacle_Distance_init+delt_Obstacle_Distance_init),2)+index_poly3*pow((free_Obstacle_Distance_init+delt_Obstacle_Distance_init),3)+index_poly4*pow((free_Obstacle_Distance_init+delt_Obstacle_Distance_init),4);
	

	fprintf(outfile31,"%f  %f  %f  %f \n ",Binary_Hist_Low_0ms,Binary_Hist_Low_1ms,Binary_Hist_High_0ms,Binary_Hist_High_1ms);
//	 fprintf(outfile31,"%f  %f  %f   \n ",index_poly[0],index_poly[1],index_poly[2]);

	
	///////////////////////////////////////////
	CELL_WIDTH=vfh_Scene_scale_x;//20mm，2cm
	WINDOW_DIAMETER=(int)(6500.0/vfh_Scene_scale_x);//主动窗口直径的珊格数
//	WINDOW_DIAMETER=(int)(6000.0/100);
	//	CELL_WIDTH=2;//20mm，2cm
	//WINDOW_DIAMETER=301;//主动窗口半径的珊格数
	SECTOR_ANGLE=5;
	SAFETY_DIST_0MS=100;
	SAFETY_DIST_1MS=200;//有点小
	Current_Max_Speed=30;//CM/秒
	MAX_SPEED=40;//cm/秒
	MAX_SPEED_NARROW_OPENING=10;
	MAX_SPEED_WIDE_OPENING=40;
	MAX_ACCELERATION=5;
	MIN_TURNRATE=20;
	//MAX_TURNRATE_0MS=20; 
	//MAX_TURNRATE_1MS=20;
	MAX_TURNRATE_0MS=30;//18;
	MAX_TURNRATE_1MS=35;
	MIN_TURN_RADIUS_SAFETY_FACTOR=1.0;

//	if (Obstacle_Distance_init)
//	Binary_Hist_Low_0ms=70000000.0;		
//	Binary_Hist_Low_1ms=50000000.0;

//	Binary_Hist_High_0ms=110000000.0;
//	Binary_Hist_High_1ms=130000000.0;
	//Binary_Hist_Low_0ms=14000000.0;		
	//Binary_Hist_Low_1ms=9000000.0;

	//Binary_Hist_High_0ms=30000000.0;
	//Binary_Hist_High_1ms=80000000.0;
	U1=50.0;
	U2=2.0;//3.0;
	Desired_Angle=0;
	Picked_Angle=0;
	Last_Picked_Angle=Picked_Angle;
	last_chosen_speed=0;
	ROBOT_RADIUS=450;//机器人膨胀半径mm//xhy

	///////////////////////////////////////////
	int x, y, i;
	float plus_dir, neg_dir, plus_sector, neg_sector;//正方向、负方向，正扇区、副扇区
	//bool plus_dir_bw, neg_dir_bw, dir_around_sector;
	//float neg_sector_to_neg_dir, neg_sector_to_plus_dir;
	//float plus_sector_to_neg_dir, plus_sector_to_plus_dir;

	bool cell_small_to_sector,cell_transe_to_sector_small,cell_in_sector,cell_transe_to_all_sector,cell_transe_to_sector_big,cell_big_to_sector;
	int cell_sector_tablenum, max_speed_this_table;
	float r;

	CENTER_X = (int)floor(WINDOW_DIAMETER /2.0);//主动窗口中心坐标珊格数(窗口中心点）
	CENTER_Y = CENTER_X;
	HIST_SIZE = (int)rint(360.0 / SECTOR_ANGLE);//直方图的尺寸
	//  //fprintf(outfile2,"  %d   ",HIST_SIZE);
	// it works now; let's leave the verbose debug statement out
	/*
	printf("CELL_WIDTH: %1.1f\tWINDOW_DIAMETER: %d\tSECTOR_ANGLE: %d\tROBOT_RADIUS: %1.1f\tSAFETY_DIST: %1.1f\tMAX_SPEED: %d\tMAX_TURNRATE: %d\tFree Space Cutoff: %1.1f\tObs Cutoff: %1.1f\tWeight Desired Dir: %1.1f\tWeight Current_Dir:%1.1f\n", CELL_WIDTH, WINDOW_DIAMETER, SECTOR_ANGLE, ROBOT_RADIUS, SAFETY_DIST, MAX_SPEED, MAX_TURNRATE, Binary_Hist_Low, Binary_Hist_High, U1, U2);
	*/

	VFH_Allocate();

	for(x=0;x<HIST_SIZE;x++) 
	{
		Hist[x] = 0;
		Last_Binary_Hist[x] = 1;
		Hist_Average_Distance[x]=0;

	}

	// For the following calcs: 
	//   - (x,y) = (0,0)   is to the front-left of the robot
	//   - (x,y) = (max,0) is to the front-right of the robot
	//
	for(x=0;x<WINDOW_DIAMETER;x++) 
	{
	
	for(y=0;y<(WINDOW_DIAMETER);y++) 
	
		{
			Cell_Mag[x][y] = 0;//每个栅格的幅值为0
			Cell_Dist[x][y] = sqrt(pow(((double)(CENTER_X - x)), 2) + pow((double)(CENTER_Y - y), 2)) * CELL_WIDTH;//栅格到主动窗口中心的距离（去离散化了））

		//	if ((Cell_Dist[x][y]<WINDOW_DIAMETER*CELL_WIDTH/2.0)&& (Cell_Dist[x][y]>(ROBOT_RADIUS/2.0)))
			{

				Cell_Base_Mag[x][y] = pow((WINDOW_DIAMETER*CELL_WIDTH/2.0 - Cell_Dist[x][y]), 4) / 20000000.0;//初始值
				
				// Set up Cell_Direction with the angle in degrees to each cell设置栅格方向，横轴为-Y轴，向右增加，纵轴位X轴，向上增加
				/////	             ^|^(0)
				////                  |
				///          (90)<————>(-90)
				//                    |
				////                  |
				////                  V（180）

				// Set up Cell_Direction with the angle in degrees to each cell设置栅格方向，背向机器人的运动方向为0度
				/////	             ^|^(180)
				////                  |
				///          (90)<————>(270)
				//                    |
				////                  |
				////                  V（-1.0）

		  if (x < CENTER_X)
			{
				if (y < CENTER_Y)///////////////////////////第VI象限
				{
					Cell_Direction[x][y] = atan((float)(CENTER_X - x)/(float)(CENTER_Y - y) );//逆时针为正
						Cell_Direction[x][y] *= (360.0 / 6.28);
						Cell_Direction[x][y] =360-Cell_Direction[x][y];//			
				} 
				else if (y == CENTER_Y) 
				{
				
					Cell_Direction[x][y] =270.0;//机器人后方为180度
				} 
				else if (y > CENTER_Y) /////////////////第II象限
				{
					Cell_Direction[x][y]= atan((float)(CENTER_X - x) /(float)(y - CENTER_Y));
					Cell_Direction[x][y] *= (360.0 / 6.28);
					Cell_Direction[x][y] = 180+Cell_Direction[x][y];
				}
			} 
			else if (x == CENTER_X)
			{
				if (y < CENTER_Y)
				{
					Cell_Direction[x][y] = 360;
				} 
				else if (y == CENTER_Y) 
				{
					Cell_Direction[x][y] = 0;
				}
				else if (y > CENTER_Y)
				{
					Cell_Direction[x][y] = 180;
				}
			} 
			else if (x > CENTER_X) 
			{
				if (y < CENTER_Y)////////////////////////第I象限
				{
					Cell_Direction[x][y] = atan( (float)(x - CENTER_X)/ (float)(CENTER_Y - y));
					Cell_Direction[x][y] *= (360.0 / 6.28);
					Cell_Direction[x][y] = Cell_Direction[x][y];
				} 
				else if (y == CENTER_Y)
				{
					Cell_Direction[x][y]= 90;
				} 
				else if (y > CENTER_Y)////////////////////第II象限
									{
					Cell_Direction[x][y] = atan( (float)(x - CENTER_X)/(float)(y - CENTER_Y));
					Cell_Direction[x][y] *=(360.0 / 6.28);
					Cell_Direction[x][y] =180-Cell_Direction[x][y];

				}
			}

		
		
				//	Cell_Direction[x][y]=Cell_Direction[x][y];//珊格相对于机器人中心的方向
				// For the case where we have a speed-dependent safety_dist, calculate all tables 如果安全距离与速度相关，计算所有的表
				for ( cell_sector_tablenum = 0; cell_sector_tablenum < NUM_CELL_SECTOR_TABLES; 	cell_sector_tablenum++ )//栅格扇区表，速度分级
				{
					max_speed_this_table = (int) (((float)(cell_sector_tablenum+1)/(float)NUM_CELL_SECTOR_TABLES) * (float) MAX_SPEED);

					// printf("cell_sector_tablenum: %d, max_speed: %d, safety_dist: %d\n",
					// cell_sector_tablenum,max_speed_this_table,Get_Safety_Dist(max_speed_this_table));

					// Set Cell_Enlarge to the _angle_ by which a an obstacle must be 
					// enlarged for this cell, at this speed 障碍物和栅格膨胀的速度一样

				//	angle_ext_robot=2.0*atan((Obstacle_Distance_init*0.5)/(ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table)))*180/PI;
					angle_ext_robot=2.0*atan((0.5*(ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table)))/(Obstacle_Distance_init))*180/PI;
					if (Cell_Dist[x][y] > 0)//栅格距离大于0
					{
					//	r = ROBOT_RADIUS + Get_Safety_Dist(max_speed_this_table);//栅格膨胀的大小与机器人的半径及安全距离有关，而安全距离与机器人的移动速度相关
					r=CELL_WIDTH;
						// Cell_Enlarge[x][y] = (float)atan( r / Cell_Dist[x][y] ) * (180/M_PI);
						Cell_Enlarge[x][y] = (float)asin( r / Cell_Dist[x][y] ) * (180/M_PI);//for each cell, define the enlargement angle 对于每个珊格，定义其膨胀角
						//fprintf(outfile2," r=%f %d   kkk %f  ",r,Get_Safety_Dist(max_speed_this_table),Cell_Enlarge[x][y]);
					}
					else
					{
						Cell_Enlarge[x][y] = 0;//机器人自身处的栅格不膨胀
					}

					Cell_Sector[cell_sector_tablenum][x][y].clear();//栅格扇区三维数组
					plus_dir = Cell_Direction[x][y] + Cell_Enlarge[x][y];//正方向，顺时针
					neg_dir  = Cell_Direction[x][y] - Cell_Enlarge[x][y];//负方向，逆时针

				
					//	//fprintf(outfile2," ddd %f  %f  ",plus_dir,neg_dir);
					for(i=0;i<(360 / SECTOR_ANGLE);i++) 
					{
						// Set plus_sector and neg_sector to the angles to the two adjacent sectors在该角度处设定正扇区和负扇区
						plus_sector = (i + 1) * (float)SECTOR_ANGLE;//扇区左边角
						neg_sector = i * (float)SECTOR_ANGLE;//扇区右边角

						/////////////////////////////////////////////////////
						 cell_small_to_sector=0,cell_transe_to_sector_small=0,cell_in_sector=0,cell_transe_to_all_sector=0,cell_transe_to_sector_big=0,cell_big_to_sector=0;

						///////////////////////////////////////////////////
						if ((plus_dir)<neg_sector && (neg_dir<neg_sector))//////////////////////////栅格在该扇区的顺时针方向外面
						{
							cell_small_to_sector=1;
						}
						else if ((plus_dir>=neg_sector) && (neg_dir<neg_sector))//////////////////////////栅格穿越该扇区的小角度边
						{
							cell_transe_to_sector_small=1;
						}
						else if ((plus_dir>=neg_sector) && (neg_dir>=neg_sector)&&(plus_dir<plus_sector) && (neg_dir<plus_sector))//////////////////////////栅格在该扇区内
						{
							cell_in_sector=1;
						}
						else if ((plus_dir>=plus_sector) && (neg_dir<neg_sector))//////////////////////////栅格穿越该扇区的两条边
						{
							cell_transe_to_all_sector=1;
						}
						else if ((plus_dir>=plus_sector) && (neg_dir<plus_sector))//////////////////////////栅格穿越该扇区的大角度边
						{
							cell_transe_to_sector_big=1;
						}
						else 
						{
							cell_big_to_sector=1;
						}


						if ((cell_transe_to_sector_small) || (cell_in_sector) || (cell_transe_to_all_sector)||(cell_transe_to_sector_big)) 
						{
							Cell_Sector[cell_sector_tablenum][x][y].push_back(i);
						
						}

		//			fprintf(outfile31," ddd== %d  %d %d %d  \n",Cell_Sector[cell_sector_tablenum][x][y],x,y,i);
						////////////////////////////////////////////////////////
					//	fprintf(outfile3,"  %d %d %d  %d  %d  %d   %d  %d \n  ",x,y,neg_dir_bw,plus_dir_bw,dir_around_sector,plus_dir_bw,i,ijkl);
						////////////////////////////////////////////////////////
					}
				}

	//			fprintf(outfile31,"%f  %f  %f  %d  %d   \n ",Cell_Dist[x][y],Cell_Direction[x][y],Cell_Base_Mag[x][y],x,y);
			}

			//else 
			//{
			//	Cell_Base_Mag[x][y] = 0.00;//初始值
			//}
		
		}
//		
	}


	last_update_time.tv_sec=timeGetTime();
	//  assert( timeGetTime( &last_update_time ) == 0 );

	// Print_Cells_Sector();
		fclose(outfile31);
	return(1);
}
 void VFH_Algorithm::fittingCurve(int n,double x[],double y[],int poly_n, double index_poly[])
 {
	 int i,j;
	 double *tempx,*tempy,*sumxx,*sumxy,*ata;

	
	 tempx=(double*)malloc(n*sizeof(double));
	 sumxx=(double*)malloc((poly_n*2+1)*sizeof(double));
	 tempy=(double*)malloc(n*sizeof(double));
	 sumxy=(double*)malloc((poly_n+1)*sizeof(double));
	 ata=(double*)malloc((poly_n+1)*(poly_n+1)*sizeof(double));
	 for (i=0;i<n;i++)
	 {
		 tempx[i]=1;
		 tempy[i]=y[i];
	 }
	 for (i=0;i<2*poly_n+1;i++)
		 for (sumxx[i]=0,j=0;j<n;j++)
		 {
			 sumxx[i]+=tempx[j];
			 tempx[j]*=x[j];
		 }
		 for (i=0;i<poly_n+1;i++)
			 for (sumxy[i]=0,j=0;j<n;j++)
			 {
				 sumxy[i]+=tempy[j];
				 tempy[j]*=x[j];
			 }
			 for (i=0;i<poly_n+1;i++)
				 for (j=0;j<poly_n+1;j++)
					 ata[i*(poly_n+1)+j]=sumxx[i+j];
		//	 gauss_solve(poly_n+1,ata,index_poly,sumxy);

			 ////////////////////////////////////////////////

			 int k,r;
			 double max;
			 int n=poly_n+1;
			
			 for (k=0;k<n-1;k++)
			 {
				 max=fabs(ata[k*n+k]); /*find maxmum*/
				 r=k;
				 for (i=k+1;i<n-1;i++)
					 if (max<fabs(ata[i*n+i]))
					 {
						 max=fabs(ata[i*n+i]);
						 r=i;
					 }
					 if (r!=k)
						 for (i=0;i<n;i++)         /*change array:A[k]&A[r]  */
						 {
							 max=ata[k*n+i];
							 ata[k*n+i]=ata[r*n+i];
							 ata[r*n+i]=max;
						 }
						 max=sumxy[k];                    /*change array:b[k]&b[r]     */
						 sumxy[k]=sumxy[r];
						 sumxy[r]=max;
						 for (i=k+1;i<n;i++)
						 {
							 for (j=k+1;j<n;j++)
								 ata[i*n+j]-=ata[i*n+k]*ata[k*n+j]/ata[k*n+k];
							 sumxy[i]-=ata[i*n+k]*sumxy[k]/ata[k*n+k];
						 }
			 }  

			 for (i=n-1;i>=0;index_poly[i]/=ata[i*n+i],i--)
				 for (j=i+1,index_poly[i]=sumxy[i];j<n;j++)
					 index_poly[i]-=ata[i*n+j]*index_poly[j];
			 ////////////////////////////////////////////////

			 free(tempx);
			 free(sumxx);
			 free(tempy);
			 free(sumxy);
			 free(ata);
 }


int VFH_Algorithm::VFH_Allocate() 
{
	std::vector<float> temp_vec;//一维浮点向量
	std::vector<int> temp_vec3;//一维整形向量
	std::vector<std::vector<int> > temp_vec2;//二维整形向量
	std::vector<std::vector<std::vector<int> > > temp_vec4;//三维整形向量
	int x;

	Cell_Direction.clear();
	Cell_Base_Mag.clear();
	Cell_Mag.clear();
	Cell_Dist.clear();
	Cell_Enlarge.clear();
	Cell_Sector.clear();

	temp_vec.clear();
	for(x=0;x<WINDOW_DIAMETER;x++) {
		temp_vec.push_back(0);//堆栈保存
	}

	temp_vec2.clear();
	temp_vec3.clear();
	for(x=0;x<WINDOW_DIAMETER;x++) {
		temp_vec2.push_back(temp_vec3);//将一维整形向量栈入，形成二维整形向量
	}

	for(x=0;x<WINDOW_DIAMETER;x++) {
		Cell_Direction.push_back(temp_vec);//栅格方向为2维浮点型向量
		Cell_Base_Mag.push_back(temp_vec);//栅格基本幅值为2维浮点型向量
		Cell_Mag.push_back(temp_vec);//栅格幅值为2维浮点型向量
		Cell_Dist.push_back(temp_vec);//栅格距离
		Cell_Enlarge.push_back(temp_vec);//
		temp_vec4.push_back(temp_vec2);//3d
	}

	for(x=0;x<NUM_CELL_SECTOR_TABLES;x++)
	{
		Cell_Sector.push_back(temp_vec4);//4d
	}

	Hist = new float[HIST_SIZE];
	Hist_Average_Distance=new float[HIST_SIZE];
		Last_Binary_Hist = new float[HIST_SIZE];
	this->SetCurrentMaxSpeed( MAX_SPEED );

	return(1);
}

int VFH_Algorithm::Update_VFH( float current_speed, 
	float goal_direction,
	float goal_distance,
	float goal_distance_tolerance,
	float &chosen_speed, 
	float &chosen_turnrate ) //扇区更新
{
	FILE *alloutfile9;
	
	alloutfile9 = fopen("allout.txt","a+");
	
	int print = 1;

	this->Desired_Angle = goal_direction;
	this->Dist_To_Goal  = goal_distance;
	this->Goal_Distance_Tolerance = goal_distance_tolerance;

	// 
	// Set current_pos_speed to the maximum of 
	// the set point (last_chosen_speed) and the current actual speed.
	// This ensures conservative behaviour if the set point somehow ramps up beyond
	// the actual speed.
	// Ensure that this speed is positive.
	//
	float current_pos_speed;
	float current_pos_turnrate;

	current_pos_turnrate=chosen_turnrate;
	if ( current_speed < 0 )
	{
		current_pos_speed = 0;
	}
	else
	{
		current_pos_speed = current_speed;
	}


	//if ( current_pos_speed < last_chosen_speed )//不只是否合理呀？
	//{
	//	current_pos_speed = last_chosen_speed;
	//}
	// printf("Update_VFH: current_pos_speed = %d\n",current_pos_speed);


	// Work out how much time has elapsed since the last update,
	// so we know how much to increase speed by, given MAX_ACCELERATION.
	timeval now;
	timeval diff;
	double  diffSeconds;
	now.tv_sec=timeGetTime();
	now.tv_usec=0.0;
	//  assert( GlobalTime->GetTime( &now ) == 0 );

	TIMESUB( &now, &last_update_time, &diff );
	diffSeconds = (diff.tv_sec + ( (double)diff.tv_usec / 1000 ))/1000.00;
	/*FILE *outfile;
	outfile=fopen("zzz.txt","a+");*/	
	/*//fprintf(outfile,"%lf\n", diffSeconds);
	fclose(outfile);*/

	last_update_time.tv_sec = now.tv_sec;
	last_update_time.tv_usec = now.tv_usec;

	if ( Build_Primary_Polar_Histogram(current_pos_speed/*,outfile*/) == 0)//创建初始极坐标直方图，获得每个扇区的Hist(x)值初始polar直方图为0,表明全堵塞
	{
		// Something's inside our safety distance: brake hard and
		// turn on the spot
		Picked_Angle = Last_Picked_Angle;
		Max_Speed_For_Picked_Angle = 0;//对应所拾取的方向角度的最大速度为0，机器人停止
		Last_Picked_Angle = Picked_Angle;

		fprintf(alloutfile9,"00000P  rimary Histogram %f \n",current_pos_speed);

	//	Print_Hist(outfile,1);*********************************************************
		/*if (print) 
		{
		fprintf(outfile,"00000P  rimary Histogram\n");
		Print_Hist(outfile);
		}*/
	}
	else//部分可通行
	{
		fprintf(alloutfile9,"00001Q  rimary Histogram  %f \n",current_pos_speed);
		Build_Binary_Polar_Histogram(current_pos_speed);//根据当前速度创建二元极坐标直方图，继续修改Hist(x)值
		//		if (print) 
		{
			//printf("Binary Histogram\n");
		//	Print_Hist(outfile,2);******************************************************
			fprintf(alloutfile9,"00002Q  rimary Histogram %f \n",current_pos_speed );
		}
		//fprintf(alloutfile9,"\n\n二元极坐标：\n");

	//		Build_Masked_Polar_Histogram(current_pos_speed);//创建遮挡极坐标直方图图
		////		  if (print)
		//		{
		//		fprintf(outfile,"Masked Histogram \n");
		//		Print_Hist(outfile,3);
		//		}
		//fprintf(alloutfile9,"\n\n遮挡坐标前：\n");
		//Print_Hist(alloutfile9);
		// Sets Picked_Angle, Last_Picked_Angle, and Max_Speed_For_Picked_Angle.
		Select_Direction();//选择方向
	}
//	Print_Hist(outfile,4);********************************************************
//	fprintf(alloutfile9,"Picked Angle: %f\n", Picked_Angle);
	//
	// OK, so now we've chosen a direction.  Time to choose a speed.
	//

	// How much can we change our speed by?如何改变机器人的速度？

	float speed_incr = 0.2;


	if (In_front_of_the_barrier)//whl20150528,添加速度放缓；（前方有障碍物时候）应该减速而不是放缓
	{
		if ( (diffSeconds > 0.3) || (diffSeconds < 0) )
		{
			// Either this is the first time we've been updated, or something's a bit screwy and
			// update hasn't been called for a while.  Don't want a sudden burst of acceleration,
			// so better to just pick a small value this time, calculate properly next time.
			speed_incr = -0.5;
			//speed_incr = 2;
		}
		else
		{
			speed_incr = -/*(int)*/ (MAX_ACCELERATION * diffSeconds);
		}

	}
	else 
	{
		if ( (diffSeconds > 0.3) || (diffSeconds < 0) )
		{
			// Either this is the first time we've been updated, or something's a bit screwy and
			// update hasn't been called for a while.  Don't want a sudden burst of acceleration,
			// so better to just pick a small value this time, calculate properly next time.
			speed_incr = 0.5;
			//speed_incr = 2;
		}
		else
		{
			speed_incr = /*(int)*/ (MAX_ACCELERATION * diffSeconds);
		}
	}

	//if (In_front_of_the_barrier)//whl20150528,添加速度放缓；（前方有障碍物时候）应该减速而不是放缓
	//{
	////	speed_incr = 0;
	//	speed_incr = 0;
	//}	




	if ( Cant_Turn_To_Goal() )
	{
		// The goal's too close -- we can't turn tightly enough to get to it,
		// so slow down.目标太近，不能快速转向到达目标，所以要慢下来，减速
		speed_incr = -speed_incr;
	}

	// Accelerate (if we're not already at Max_Speed_For_Picked_Angle).没有能够在选择方向上得到最大速度，加速
	chosen_speed = MIN( last_chosen_speed + speed_incr, Max_Speed_For_Picked_Angle);
	MinSpeed = speed_stated_vfh;
	if (chosen_speed<MinSpeed)
	{
		chosen_speed = MinSpeed;
	}
	/*FILE *aout;
	aout = fopen("aout.txt","a+");*/
	//fprintf(aout,"chosen_speed：%d \n",chosen_speed);
	//fclose(aout);
	// printf("Max Speed for picked angle: %d\n",Max_Speed_For_Picked_Angle);

	// Set the chosen_turnrate, and possibly modify the chosen_speed 设定旋转角速度，可能会改变所选择的速度
	Set_Motion( chosen_speed, chosen_turnrate, current_pos_speed );

	last_chosen_speed = chosen_speed;
	/////////////////////////////////////////
	/*FILE *outfile1;

	outfile1=fopen("zzz.txt","a+");*/

//	fprintf(alloutfile9," Candidate_Angle.size %d\n", Candidate_Angle.size());

	for(int i=0;i<Candidate_Angle.size();i++) 
	{

		//  if (print)
		fprintf(alloutfile9,"%f %f  CHOSEN: SPEED: %f TURNRATE: %f 期望%f 选择%f 候选%f \n ",current_speed,chosen_turnrate, chosen_speed, chosen_turnrate,Desired_Angle,Picked_Angle,Candidate_Angle[i]);

	}

	fclose(alloutfile9);
	return(1);

}

//
// Are we going too fast, such that we'll overshoot before we can turn to the goal? 如果速度过快，则会转过头
//
bool VFH_Algorithm::Cant_Turn_To_Goal()
{
	// Calculate this by seeing if the goal is inside the blocked circles 阻塞圆
	// (circles we can't enter because we're going too fast).  Radii set
	// by Build_Masked_Polar_Histogram.

	// Coords of goal in local coord system:
	float newgoal_x = this->Dist_To_Goal * cos( DTOR(this->Desired_Angle) );
	float newgoal_y = this->Dist_To_Goal * sin( DTOR(this->Desired_Angle) );

	// AlexB: Is this useful?
	//     if ( goal_y < 0 )
	//     {
	//         printf("Goal behind\n");
	//         return true;
	//     }

	// This is the distance between the centre of the goal and
	// the centre of the blocked circle 目标中心和阻塞圆中心间距离
	float dist_between_centres;

	//     printf("Cant_Turn_To_Goal: Dist_To_Goal = %f\n",Dist_To_Goal);
	//     printf("Cant_Turn_To_Goal: Angle_To_Goal = %f\n",Desired_Angle);
	//     printf("Cant_Turn_To_Goal: Blocked_Circle_Radius = %f\n",Blocked_Circle_Radius);

	// right circle 右圆
	dist_between_centres = hypot( newgoal_x - this->Blocked_Circle_Radius, newgoal_y );//A call to _hypot is equivalent to the square root of x2 + y2
	if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
	{
		//        printf("Goal close & right\n");目标靠近右边
		return true;
	}

	// left circle 
	dist_between_centres = hypot( -newgoal_x - this->Blocked_Circle_Radius, newgoal_y );
	if ( dist_between_centres+this->Goal_Distance_Tolerance < this->Blocked_Circle_Radius )
	{
		//        printf("Goal close & left.\n");目标靠近左边
		return true;
	}

	return false;
}

float VFH_Algorithm::Delta_Angle(int a1, int a2) 
{
	return(Delta_Angle((float)a1, (float)a2));
}

float VFH_Algorithm::Delta_Angle(float a1, float a2) 
{
	float diff;

	diff = a2 - a1;//两角之差,between -180-180

	if (diff > 180) {
		diff -= 360;
	} else if (diff < -180) {
		diff += 360;
	}

	return(diff);
}


int VFH_Algorithm::Bisect_Angle(int angle1, int angle2) 
{
	float a;
	int angle;

	a = Delta_Angle((float)angle1, (float)angle2);

	angle = (int)rint(angle1 + (a / 2.0));//角度的平分角
	if (angle < 0) {
		angle += 360;
	} else if (angle >= 360) {
		angle -= 360;
	}

	return(angle);
}
float pick = 0.00;
int VFH_Algorithm::Select_Candidate_Angle() //选择候选角度
{
	FILE *alloutfile9;
	
	alloutfile9 = fopen("allout009.txt","a+");
	
	unsigned int i;
	float weight, min_weight;


	no_angle = false;
	if (Candidate_Angle.size() == 0) //////////////////
	{
		// We're hemmed in by obstacles -- nowhere to go, 
		// so brake hard and turn on the spot.
		Picked_Angle = Last_Picked_Angle;//whl20150525注释//whl20150529注销注释
		Max_Speed_For_Picked_Angle = 0;
		Last_Picked_Angle = Picked_Angle;
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		no_angle = true;

		fprintf(alloutfile9,"00000000:  %f  %f    \n",Last_Picked_Angle,Picked_Angle);
		//停止一下；//whl20150527此处调用mo2，有些问题，先用着。
		//mo2.back_speed(10,10);

		//mo2.Robot_Stop();
		//int a = SUIJISHU(30,90);//whl20150528
		//if (a>0)
		//{
		//	mo2.back_speed(8,19);
		//	//mo2.Robot_Left_arc_back(-10,a*3.14/180/3);
		//
		//}
		//else
		//{
		//	mo2.back_speed(9,18);
		//	//mo2.Robot_Right_arc_back(-10,a*3.14/180/3);
		//	
		//}
		
		//Sleep(12);
		return(1);
	}
	Picked_Angle = 0;
	min_weight = 10000000;
	for(i=0;i<Candidate_Angle.size();i++)
	{
		weight = U1 * fabs(Delta_Angle(Desired_Angle, Candidate_Angle[i])) +
			U2 * fabs(Delta_Angle(Last_Picked_Angle, Candidate_Angle[i]));//
		if (weight <= min_weight) 
		{
			min_weight = weight;
			Picked_Angle = Candidate_Angle[i];
			Max_Speed_For_Picked_Angle = Candidate_Speed[i];
		}

		fprintf(alloutfile9,"Select_Candidate_Angle:  %f  %f   %f %f  %d \n",Desired_Angle,Candidate_Angle[i],Last_Picked_Angle,Picked_Angle,i);

	}

		fprintf(alloutfile9,"Last_Picked_Angle:  %f  %f \n",Desired_Angle,Picked_Angle);
	Last_Picked_Angle = Picked_Angle;
	pick = Last_Picked_Angle;

	fclose(alloutfile9);
	return(1);
}


int VFH_Algorithm::Select_Direction() //判断扇区
{
		FILE *alloutfile9;
	
	alloutfile9 = fopen("allout0019.txt","a+");
	int start, i, left;
	float angle, new_angle;
	std::vector<std::pair<int,int> > border;
	std::pair<int,int> new_border;

	Candidate_Angle.clear();
	Candidate_Speed.clear();

	border.clear();

	//
	// set start to sector of first obstacle 设置第一个障碍物的起始扇区
	start = -1;
	// only look at the forward 180deg for first obstacle.对第一个障碍物只关注前面180度范围
	left = 0;//不定义you边界，先开始找zuo边界
	//	for (int i = 0; i < HIST_SIZE/2; i++)
	for (int i = 0; i < HIST_SIZE; i++)
	{
		//	if ((Hist[i] != 1)&&(!left))//从右向左寻找第一个阻塞扇区的起始边界角度
		if ((Hist[i] <0.60)&&(!left))//从右向左寻找第一个阻塞扇区的起始边界角度
		{
			new_border.first = (i % HIST_SIZE) * SECTOR_ANGLE;//找出新畅通边界的起始边zuo边界
			left = 1;
		}
		if (((Hist[i])>= 0.80) && (left))
		{
			new_border.second = ((i) - 1) * SECTOR_ANGLE;//找出新边界的第二个边，you边界
			left = 0;
			border.push_back(new_border);//形成一个新的畅通区间
		}
		if ((i == HIST_SIZE-1)&&(left))
		{  
			new_border.second = (HIST_SIZE - 1) * SECTOR_ANGLE;//找出新边界的第二个边，左边界
			border.push_back(new_border);//形成一个新的畅通区间
		}

//		fprintf(alloutfile9,"Last_Picked_Angle: %f %d  %d  %d   \n",Hist[i],i,new_border.first,new_border.second);
	}

	//
	// Consider each opening考虑每个开放空间
	//


	
	

	In_front_of_the_barrier = true;

	if((int)border.size()>0)
	{
		for(int i=0;i<(int)border.size();i++) 
		{
	//		fprintf(alloutfile9,"BORDER: %d %d\n", border[i].first, border[i].second);

			double First_Average_Distance=0.0;
			double Second_Average_Distance=0.0;
			if (border[i].first==0)
			{
				First_Average_Distance=Hist_Average_Distance[(int)(border[i].first/SECTOR_ANGLE)];
			}
			else 
			{
			First_Average_Distance=Hist_Average_Distance[(int)(border[i].first/SECTOR_ANGLE)-1];
			}
			if (border[i].second==71)
			{
			Second_Average_Distance=Hist_Average_Distance[(int)(border[i].second/SECTOR_ANGLE)];
			}
			else 
			{
				Second_Average_Distance=Hist_Average_Distance[(int)(border[i].second/SECTOR_ANGLE)+1];
			}

		//	double min_Average_Distance=MIN(First_Average_Distance,Second_Average_Distance);
			double min_Average_Distance=(First_Average_Distance+Second_Average_Distance)/2.0;
			double min_angle_ext_robot=0;
			int Candidate_Number_Free_Sector=0;
			double sss=0.0;
			sss=tan(angle_ext_robot*0.5*PI/180)*Obstacle_Distance_init;//////////////////////通道半径
			min_angle_ext_robot=2.0*atan(sss/(min_Average_Distance))*180/PI;

			angle = Delta_Angle(border[i].first, border[i].second);//求出开放空间的角度

			fprintf(alloutfile9,"BORDER: %f %d %d  %f %f %f %f %f %f %f \n",Desired_Angle,border[i].first, border[i].second, First_Average_Distance,Second_Average_Distance,min_Average_Distance,angle_ext_robot,Obstacle_Distance_init,sss,min_angle_ext_robot);
			//		//fprintf(outfile2,"ANGLE: %f  %d  \n", angle,i);
	//		fprintf(alloutfile9,"BORDER_000: %f %f %d %d %f %d %f  %d %f  %f %f  \n", sss,min_Average_Distance,border[i].first, border[i].second,angle,i,angle_ext_robot,(int)border.size(),First_Average_Distance,Second_Average_Distance,min_angle_ext_robot);

			if (fabs(angle) < min_angle_ext_robot) //空间太窄
			{
				// ignore very narrow openings

				fprintf(alloutfile9,"VVVVVVV: %d    \n", i);
				continue;
			}
			else if ((fabs(angle) < 80)&&(fabs(angle) >=min_angle_ext_robot)) //角度对准其中心角
			{

				// narrow opening: aim for the centre
				//////////xhy

				//new_angle = border[i].first + (border[i].second - border[i].first) / 2.0/*+Info_robot.pianzhuan*/;

				//Candidate_Angle.push_back(new_angle);
				//Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_NARROW_OPENING));

				//fprintf(alloutfile9,"www: %f    \n", new_angle);
				/////////////////////////xhy
				//if( border[i].first!=0)
				//{
					new_angle = border[i].first + (border[i].second - border[i].first) / 2.0/*+Info_robot.pianzhuan*/;

					Candidate_Angle.push_back(new_angle);
					Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_NARROW_OPENING));

					fprintf(alloutfile9,"www: %f    \n", new_angle);
				//}

				////////////////////xhy
			} 
			else //(fabs(angle)>80,较宽的开放扇区
			{
				// wide opening: consider the centre, and 40deg from each border，考虑到该角度的中心角，距离两边至少40度以上
				Candidate_Number_Free_Sector=(int)(border[i].second - border[i].first)/(min_angle_ext_robot*0.5);
				fprintf(alloutfile9,"oookkk: %d   %f   \n", Candidate_Number_Free_Sector,min_angle_ext_robot);
				int i_temp=0;

				for (int i_number=0;i_number<Candidate_Number_Free_Sector;i_number++)
				{
					i_temp=i_number+1;
					if (i_temp<Candidate_Number_Free_Sector)
					{
					new_angle=border[i].first+(i_temp)*min_angle_ext_robot*0.6;
					
					
					}
					Candidate_Angle.push_back(new_angle);
					Candidate_Speed.push_back(Current_Max_Speed);
					fprintf(alloutfile9,"ooo: %f  %d  %f   \n", new_angle,i_number,min_angle_ext_robot);
				}

				//new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;

				//Candidate_Angle.push_back(new_angle);
				//Candidate_Speed.push_back(Current_Max_Speed);

				//new_angle = (float)((border[i].first + 20) % 360);
				//Candidate_Angle.push_back(new_angle);
				//Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
				//new_angle = (float)((border[i].first + 40) % 360);
				//Candidate_Angle.push_back(new_angle);
				//Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));

				//new_angle = (float)(border[i].second - 20);
				//if (new_angle < 0) 
				//	new_angle += 360;
				//Candidate_Angle.push_back(new_angle);
				//Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));//保存了中心角、左边线角+40，右边线角-40三个角度

				// See if candidate dir is in this opening 检验候选角是否在该开放扇区，产生期望角度
				if ((Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-Candidate_Number_Free_Sector]) < 0) && 
					(Delta_Angle(Desired_Angle, Candidate_Angle[Candidate_Angle.size()-1]) > 0)) 
				{
					Candidate_Angle.push_back(Desired_Angle);
					Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
				}
			}
		
		}

	}
	else 
	{
		Candidate_Angle.push_back(Desired_Angle);
		Candidate_Speed.push_back(MIN(Current_Max_Speed,MAX_SPEED_WIDE_OPENING));
	}

	
//	fprintf(alloutfile9,"wwwwww: %f    \n", Desired_Angle);

	Select_Candidate_Angle();//选择候选角度


	fclose(alloutfile9);
	return(1);

}

void VFH_Algorithm::Print_Cells_Dir(FILE *outfile) 
{
	int x, y;

	//fprintf(outfile,"\nCell Directions:\n");
	//fprintf(outfile,"****************\n");
	for(y=0;y<WINDOW_DIAMETER;y++) {
		for(x=0;x<WINDOW_DIAMETER;x++) {
			//fprintf(outfile,"%1.1f\t", Cell_Direction[x][y]);
		}
		//fprintf(outfile,"\n");
	}
}

void VFH_Algorithm::Print_Cells_Mag(FILE *outfile) 
{
	int x, y;


	//fprintf(outfile,"****************\n");
	//for(y=0;y<WINDOW_DIAMETER;y++) {
	//	for(x=0;x<WINDOW_DIAMETER;x++) {

	//	}

	//}

	//	fprintf(outfile2,"%d    \n",WINDOW_DIAMETER);


}

void VFH_Algorithm::Print_Cells_Dist(FILE *outfile) 
{
	int x, y;

//	fprintf(outfile,"\nCell Distances:\n");
	//fprintf(outfile,"****************\n");
//	for(y=0;y<WINDOW_DIAMETER;y++) 
//	{
//		for(x=0;x<WINDOW_DIAMETER;x++)
//		{
////			fprintf(outfile,"%2.2f\t", Cell_Dist[x][y]);
//		}
////		fprintf(outfile,"\n");
//	}
}

void VFH_Algorithm::Print_Cells_Sector(FILE *outfile) 
{
	int x, y;
	unsigned int i;

	//fprintf(outfile,"\nCell Sectors for table 0:\n");
	//fprintf(outfile,"***************************\n");

	//for(y=0;y<WINDOW_DIAMETER;y++) {
	//	for(x=0;x<WINDOW_DIAMETER;x++) {
	//		for(i=0;i<Cell_Sector[0][x][y].size();i++) {
	//			if (i < (Cell_Sector[0][x][y].size() -1 )) {
	//				//fprintf(outfile,"%d,", Cell_Sector[0][x][y][i]);
	//			} else {
	//				//fprintf(outfile,"%d\t", Cell_Sector[0][x][y][i]);
	//			}
	//		}
	//	}
	//	//fprintf(outfile,"\n");
	//}
}

void VFH_Algorithm::Print_Cells_Enlargement_Angle(FILE *outfile) 
{
	int x, y;

	//fprintf(outfile,"\nEnlargement Angles:\n");
	//fprintf(outfile,"****************\n");
	//for(y=0;y<WINDOW_DIAMETER;y++) {
	//	for(x=0;x<WINDOW_DIAMETER;x++) {
	//		//fprintf(outfile,"%1.1f\t", Cell_Enlarge[x][y]);
	//	}
	//	//fprintf(outfile,"\n");
	//}
}

void VFH_Algorithm::Print_Hist(FILE *outfile,int type) 
{
	int x;


	for(x=0;x<(HIST_SIZE);x++) 
	{
		fprintf(outfile,"%d  %2.2f \n", (x * SECTOR_ANGLE), Hist[x]);
	}

}


//whl20150511return用法是不对的，要全局变量返回值；
int VFH_Algorithm::Calculate_Cells_Mag(  int speed/*,FILE *outfile*/ ) 
{
	FILE *outfile009;
	outfile009=fopen("zcs0069.txt","a+");
	//int x, y;//计算栅格的幅值
	int Calculate_Cells_Mag_return = 0;
	int laser_step=0;

	// AB: This is a bit dodgy...  Makes it possible to miss really skinny obstacles, since if the 
	//     resolution of the cells is finer than the resolution of laser_ranges, some ranges might be missed.
	//     Rather than looping over the cells, should perhaps loop over the laser_ranges.
	float r = (ROBOT_RADIUS + Get_Safety_Dist(speed));//the obstacle cells are actually enlarged by r

	//fprintf(outfile,"r,ROBOT_RADIUS:  %f %f  \n",r,ROBOT_RADIUS);
	//for (int i=0;i<1000;i++)
	//{

	//	fprintf(outfile009," %d  %d  \n",m_laser_data_postpro_vfh[i],i);

	//}


	// Only deal with the cells in front of the robot, since we can't sense behind.只管前不管后

	for(int x=(int)floor(WINDOW_DIAMETER/2.0);x<WINDOW_DIAMETER;x++) //whl20150511与下边的180度判断重复；
//	for(int x=0;x<WINDOW_DIAMETER;x++) ///whl20150520;
	{
		
			for(int y=0;y<(int)floor(WINDOW_DIAMETER/2.0);y++) ///whl20150520;第IV象限
		{
			
		//	if ((rint((Cell_Direction[x][y]))<360)&&(rint((Cell_Direction[x][y]))>=270) )
		//	laser_step=-768+(int)(rint((Cell_Direction[x][y]))*1024/360);
			laser_step=(int)(rint((Cell_Direction[x][y]))*1024/360)+128;///////////x=0,y=0,对应栅格地图的左下角，方向角为225度，对应激光扫描数为768
			///////////////////////////////////////////////////////////////////////x=0,y=WINDOW_DIAMETER/2,对应栅格图最左边，方向角180度，对应的激光扫描数为640
			//////////////////////////////////////////////////////////////////////x=0,y=WINDOW_DIAMETER,对应栅格左上角，方向角135度，对应的激光扫描线为512
			//////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER/2,y=WINDOW_DIAMETER,对应栅格最上面，方向角为90度，对应的激光扫描线为384
			//////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=WINDOW_DIAMETER,对应栅格右上角，方向角为45度，对应的激光扫描线为256
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=WINDOW_DIAMETER/2,对应栅格最右边，方向角为0度，对应的激光扫描线为128
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=0,对应栅格右下角，方向角为-45度，对应的激光扫描线为0
	//		fprintf(outfile009,"pppp %f  %d  %d %d %d %f \n",Cell_Direction[x][y],x,y,laser_step,m_laser_data_postpro_vfh[laser_step],Cell_Dist[x][y]);
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER/2,y=0,对应栅格最下边，方向角为-90度，对应的激光扫描线为负数，不存在
	//		if (Cell_Dist[x][y] > r)
			{
				if ((Cell_Dist[x][y] + CELL_WIDTH / 2.0) > m_laser_data_postpro_vfh[laser_step]) /////////该栅格被障碍物占据
					{
			
						if (Cell_Dist[x][y] < r)
							//其到主动窗口中心的距离小于机器人的膨胀距离，
							//表明该栅格无法通行，返回0标记
						{
							if ((x==CENTER_X && y==CENTER_Y))
							{
								Cell_Mag[x][y] = 99999;
								Calculate_Cells_Mag_return=0;
		//					fprintf(outfile009," 000=%f \n ",Cell_Mag[x][y]);
							}
						
							else 
							{
								Cell_Mag[x][y] = 88888;
								Calculate_Cells_Mag_return=0;
		//						fprintf(outfile009," 111=%f \n ",Cell_Mag[x][y]);
							}
						
						}
						else// 大于激光器末端读数的栅格，其到主动窗口中心的距离大于机器人的膨胀距离，表明机器人碰不到该栅格，因此其栅格幅值为其基本幅值,但该栅格为障碍物栅格
						{
							Cell_Mag[x][y] = Cell_Base_Mag[x][y];// Cell_Base_Mag[x][y] = pow((3000.0 - Cell_Dist[x][y]), 4) / 100000000.0;
							Calculate_Cells_Mag_return = 2;
					//		fprintf(outfile009," 222=%f %f  %d %f \n ",Cell_Mag[x][y],(Cell_Dist[x][y] + CELL_WIDTH / 2.0), m_laser_data_postpro_vfh[laser_step],Cell_Mag[x][y]);
						}
					} 
					else 
					{// 小于激光器末端读数的栅格，表明该栅格没有被障碍物占据，因此其栅格幅值为0，非障碍物栅格
						Cell_Mag[x][y] = 0.00;//0.0
						Calculate_Cells_Mag_return = 1;
				//		fprintf(outfile009," 333=%f \n ",Cell_Mag[x][y]);
					}
			}
			//else
			//{
			//	Cell_Mag[x][y] = 777777.00;//0.0
			//	Calculate_Cells_Mag_return=0;
			//}
	//		fprintf(outfile009," %d %f %d %f %d  %d %f \n ",laser_step,Cell_Direction[x][y],m_laser_data_postpro_vfh[laser_step],(Cell_Dist[x][y] + CELL_WIDTH / 2.0),x,y,Cell_Mag[x][y]);
		

		}

		for(int y=(int)floor(WINDOW_DIAMETER/2.0);y<WINDOW_DIAMETER;y++) ///whl20150520;第IV象限
		{
			
		//	if ((rint((Cell_Direction[x][y]))<360)&&(rint((Cell_Direction[x][y]))>=270) )
		//	laser_step=-768+(int)(rint((Cell_Direction[x][y]))*1024/360);
			laser_step=(int)(rint((Cell_Direction[x][y]-90))*1024/360)+384;///////////x=0,y=0,对应栅格地图的左下角，方向角为225度，对应激光扫描数为768
			///////////////////////////////////////////////////////////////////////x=0,y=WINDOW_DIAMETER/2,对应栅格图最左边，方向角180度，对应的激光扫描数为640
			//////////////////////////////////////////////////////////////////////x=0,y=WINDOW_DIAMETER,对应栅格左上角，方向角135度，对应的激光扫描线为512
			//////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER/2,y=WINDOW_DIAMETER,对应栅格最上面，方向角为90度，对应的激光扫描线为384
			//////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=WINDOW_DIAMETER,对应栅格右上角，方向角为45度，对应的激光扫描线为256
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=WINDOW_DIAMETER/2,对应栅格最右边，方向角为0度，对应的激光扫描线为128
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER,y=0,对应栅格右下角，方向角为-45度，对应的激光扫描线为0
	//	fprintf(outfile009,"kkkk  %f  %d  %d %d %d %f \n",Cell_Direction[x][y],x,y,laser_step,m_laser_data_postpro_vfh[laser_step],Cell_Dist[x][y]);
			/////////////////////////////////////////////////////////////////////x=WINDOW_DIAMETER/2,y=0,对应栅格最下边，方向角为-90度，对应的激光扫描线为负数，不存在
	//		if (Cell_Dist[x][y] > r)
			{
				if ((Cell_Dist[x][y] + CELL_WIDTH / 2.0) > m_laser_data_postpro_vfh[laser_step]) /////////该栅格被障碍物占据
					{
			
						if (Cell_Dist[x][y] < r)
							//其到主动窗口中心的距离小于机器人的膨胀距离，
							//表明该栅格无法通行，返回0标记
						{
							if ((x==CENTER_X && y==CENTER_Y))
							{
								Cell_Mag[x][y] = 99999;
								Calculate_Cells_Mag_return=0;
		//					fprintf(outfile009," 000=%f \n ",Cell_Mag[x][y]);
							}
						
							else 
							{
								Cell_Mag[x][y] = 88888;
								Calculate_Cells_Mag_return=0;
		//						fprintf(outfile009," 111=%f \n ",Cell_Mag[x][y]);
							}
						
						}
						else// 大于激光器末端读数的栅格，其到主动窗口中心的距离大于机器人的膨胀距离，表明机器人碰不到该栅格，因此其栅格幅值为其基本幅值,但该栅格为障碍物栅格
						{
							Cell_Mag[x][y] = Cell_Base_Mag[x][y];// Cell_Base_Mag[x][y] = pow((3000.0 - Cell_Dist[x][y]), 4) / 100000000.0;
							Calculate_Cells_Mag_return = 2;
					//		fprintf(outfile009," 222=%f %f  %d %f \n ",Cell_Mag[x][y],(Cell_Dist[x][y] + CELL_WIDTH / 2.0), m_laser_data_postpro_vfh[laser_step],Cell_Mag[x][y]);
						}
					} 
					else 
					{// 小于激光器末端读数的栅格，表明该栅格没有被障碍物占据，因此其栅格幅值为0，非障碍物栅格
						Cell_Mag[x][y] = 0.00;//0.0
						Calculate_Cells_Mag_return = 1;
				//		fprintf(outfile009," 333=%f \n ",Cell_Mag[x][y]);
					}
			}
			//else
			//{
			//	Cell_Mag[x][y] = 777777.00;//0.0
			//	Calculate_Cells_Mag_return=0;
			//}
		//	fprintf(outfile009," %d %f %d %f %d  %d %f \n ",laser_step,Cell_Direction[x][y],m_laser_data_postpro_vfh[laser_step],(Cell_Dist[x][y] + CELL_WIDTH / 2.0),x,y,Cell_Mag[x][y]);
		

		}

	
			
	}		
		
		

			//	fprintf(outfile009,"  %d %f %f %d  \n",laser_step,Cell_Direction[x][y],Cell_Mag[x][y]);
		
		//	fprintf(outfile009,"%d ",m_laser_data_postpro_vfh[laser_step],laser_step);

		
	

			for(int x=0; x<(int)floor(WINDOW_DIAMETER/2.0);x++) //whl20150511与下边的180度判断重复；
			{

				for(int y=0;y<WINDOW_DIAMETER;y++) ///whl20150520;
				{

					Cell_Mag[x][y] = 555555;
				Calculate_Cells_Mag_return=4;
				}
			}
		

	fclose(outfile009);
	return(Calculate_Cells_Mag_return);//碰不到机器人的栅格，返回1
}

FILE *outfile009;

//	
int VFH_Algorithm::Build_Primary_Polar_Histogram( float speed /*,FILE *outfile*/) 
{
	//int Build_Primary_Polar_Histogram_return = 0;
	outfile009=fopen("zcs009.txt","a+");
	int x, y;
	unsigned int i;
	// index into the vector of Cell_Sector tables
	int speed_index = Get_Speed_Index( speed );
	//int   speed_index=0;

	for(x=0;x<HIST_SIZE;x++) {
		Hist[x] = 0;//直方图赋初值，全通	
	}
	// //fprintf(outfile,"all stack_001  \n");
	//Calculate_Cells_Mag(  speed ,outfile);
	if ( Calculate_Cells_Mag(  speed /*,outfile*/) == 0 )//根据栅格幅值判断该栅格与机器人发生碰撞时，
		//设定该直方图为全堵塞，Hist[x] = 1;
	{
	//	Print_Cells_Mag(outfile);//幅值
		// set Hist to all blocked
		for(x=0;x<HIST_SIZE;x++) 
		{
			Hist[x] = 1;
		}
	//
		return 0;//该函数Build_Primary_Polar_Histogram返回0
	}
//	Print_Cells_Dist(outfile);//距离
	//	  Print_Cells_Dir(outfile);//方向
//	Print_Cells_Mag(outfile);//幅值
//	Print_Cells_Sector(outfile);//扇区
	//	  Print_Cells_Enlargement_Angle(outfile);//膨胀角

	// Only have to go through the cells in front.机器人只通过前面的栅格
	
	//for(x=(int)ceil(WINDOW_DIAMETER/2.0);x<WINDOW_DIAMETER;x++)

	int icount=0;
//	for(x=(int)ceil(WINDOW_DIAMETER/2.0);x<WINDOW_DIAMETER;x++)
	
	for(x=0;x<WINDOW_DIAMETER;x++)
	{
		for(y=0;y<WINDOW_DIAMETER;y++)
	
		{		
			for(i=0;i<Cell_Sector[speed_index][x][y].size();i++) 
			{	
				
		//		Hist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];//扇区幅值累加
				Hist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];//扇区幅值累加

				
			
		////		icount++;
		//		if (Cell_Sector[speed_index][x][y][i]<=20 &&Cell_Sector[speed_index][x][y][i]>=16 )
		//		{
		////		fprintf(outfile009," %d  %d  %d  %d %f %f\n",Cell_Sector[speed_index][x][y][i],x,y,i,Hist[Cell_Sector[speed_index][x][y][i]],Cell_Mag[x][y]);
		//		}

			}
	
		}

	} 

	int icount1=0;
	


	for (int ii=0;ii<HIST_SIZE;ii++)
	{
	//	fprintf(outfile009," oooooooooo====%f  %d    \n",Hist[ii],ii);
		double Hist_value=Hist[ii];
		double temp_relsult1=0;
		double temp_relsult2=0;
		double min_x_result=10000000000;
		Compute_Sector_Average_Distance(Hist_value,index_poly);
		for (int i=0;i<4;i++)
		{
			if ((x_result[i].imag()<=0.000001)&& (x_result[i].imag()>=-0.000001))
			{
				if((x_result[i].real()>=0.000001))
				{
					temp_relsult1=x_result[i].real();

					if (temp_relsult1<min_x_result)
					{
						min_x_result=temp_relsult1;
					}
				}
			/*	else
				{
					temp_relsult1=0.000001;

					if (temp_relsult1<min_x_result)
					{
						min_x_result=temp_relsult1;
					}
				}*/

			}

	//		min_x_result=
	//		fprintf(outfile009," %f  %d  %f  %f %f %d %f %f \n",Hist[ii],ii,Hist_Average_Distance[ii],x_result[i].real(),x_result[i].imag(),i,sqrt((x_result[i].real() * x_result[i].real() + x_result[i].imag() *x_result[i].imag())),min_x_result);
		}
		Hist_Average_Distance[ii]=min_x_result;
		icount1=int((ii*5+2.5)*1024/360+128);

		fprintf(outfile009," %f  %d  %f %f %d  \n",Hist[ii],ii,Hist_Average_Distance[ii],speed,m_laser_data_postpro_vfh[icount1]);

	}
	//for(y=0;y<(int)ceil(WINDOW_DIAMETER/2.0);y++)
	//{
	//	for(x=0;x<WINDOW_DIAMETER;x++)	//	{		
	//		for(i=0;i<Cell_Sector[speed_index][x][y].size();i++) 
	//		{		  
	//			Hist[Cell_Sector[speed_index][x][y][i]] += Cell_Mag[x][y];//扇区幅值累加
	//			//	fprintf(outfile07," %f   \n",Hist[Cell_Sector[speed_index][x][y][i]]);

	//	//		fprintf(outfilehist,"Hist_back %f  %d  %f  \n",Hist[Cell_Sector[speed_index][x][y][i]],Cell_Sector[speed_index][x][y][i],Cell_Mag[x][y]);

	//		}
	//		/* //fprintf(outfile,"%f  \n",Cell_Mag[x][y]);*/		
	//	}
	//} 
	//
	//for(x=0;x<HIST_SIZE;x++) 
	//{
	//	fprintf(outfile009,"%f            %d \n",Hist[x],x);
	//}
	fclose(outfile009);
	
	return(1);
}

void  VFH_Algorithm::Compute_Sector_Average_Distance(double Hist_value,double index_poly[5])
{
	/////////////////////////////////////////////以下代码为一元二次方程的求解算法
	//double delta=0;
	//double x_result[2];
	//x_result[0]=0;
	//x_result[1]=0;

	//double last_result=0;
	//delta=index_poly[1]*index_poly[1]-4*index_poly[2]*(index_poly[0]-Hist_value);

	//if (delta>0)
	//{
	//	x_result[0]=(-index_poly[1]+sqrt(delta))/(2*index_poly[2]);
	//	x_result[1]=(-index_poly[1]-sqrt(delta))/(2*index_poly[2]);
	//	if (MIN(x_result[0],x_result[1])<=0) 
	//	{
	//		last_result=MAX(x_result[0],x_result[1]);

	//	}
	//	else 
	//	{
	//		last_result=MIN(x_result[0],x_result[1]);
	//	}
	//	return last_result;
	//}
	//else if (delta==0)
	//{
	//	x_result[0]=(-index_poly[1])/(2*index_poly[2]);
	//	x_result[1]=x_result[0];
	//	return x_result[0];
	//}
	//else 
	//{
	//	return 0;
	//}
	/////////////////////////////////////对于本项目中需要使用一元四次方程的求解函数（使用费拉里法求解一元四次方程）
	
	Ferrari(x_result,index_poly[4],index_poly[3],index_poly[2],index_poly[1],index_poly[0]-Hist_value); //调用费拉里函数法
	
}

int VFH_Algorithm::Build_Binary_Polar_Histogram( float speed ) //创建二元极坐标直方图 avoiding oscillations in the steering command and obtain a smooth trajectory
{

		outfile009=fopen("zcs0019.txt","a+");
	int x;

	//参见《VFH+：Reliable Obstacle avoidance for Fast Mobile Robots》公式（7）
	int aa = Get_Binary_Hist_High(speed);
	int bb = Get_Binary_Hist_Low(speed);
	//	outfile=fopen("ww_pos.txt","a+");
	//	//fprintf(outfile,"11111111111111111111111\n"); 
//	Print_Hist(outfile,2);

	for(int x=0;x<(HIST_SIZE);x++) 
	{
		fprintf(outfile009,"%d  %2.2f %f %f  %f %f  %d \n", (x * SECTOR_ANGLE), Hist[x],Last_Binary_Hist[x],Get_Binary_Hist_High(speed),Get_Binary_Hist_Low(speed),speed,Current_Max_Speed);
	}


	for(x=0;x<(int)(HIST_SIZE);x++) 
	{	 
		if (Hist[x] > Get_Binary_Hist_High(speed)) 
		{
			Hist[x] = 1.0;// 
		} 
		else if ((Hist[x] < Get_Binary_Hist_High(speed))&&(Hist[x] >= Get_Binary_Hist_Low(speed))) 
		{
			
			Hist[x] = Last_Binary_Hist[x];
		}
		else 
		{
			Hist[x] = 0.0;
		}
	}

	

	//for(x=(int)(HIST_SIZE/2);x<HIST_SIZE;x++) 
	//{	 
	//	Hist[x] = 1.0;// 
	//}


	
	//for(x=(int)(3*HIST_SIZE/4);x<HIST_SIZE;x++) 
	//{	 
	//	if (Hist[x] > Get_Binary_Hist_High(speed)) 
	//	{
	//		Hist[x] = 1.0;// 
	//	} 
	//	else if (Hist[x] < Get_Binary_Hist_Low(speed)) 
	//	{
	//		Hist[x] = 0.0;
	//	}
	//	else 
	//	{
	//		Hist[x] = Last_Binary_Hist[x];
	//	}
	//}

	for(x=0;x<HIST_SIZE;x++) {
		Last_Binary_Hist[x] = Hist[x];
	}


	//	//fprintf(outfile,"222222222222222222\n");
	//	fclose(outfile);

	for(x=0;x<HIST_SIZE;x++) 
	{
		fprintf(outfile009," fffffff  %f      %f       %d   %f  %f  \n",Hist[x],Last_Binary_Hist[x],x,Get_Binary_Hist_High(speed),Get_Binary_Hist_Low(speed));
	}
	fclose(outfile009);
	
	return(1);
}

//
// This function also sets Blocked_Circle_Radius.该函数也可以用来设置阻塞圆半径
//
int VFH_Algorithm::Build_Masked_Polar_Histogram(float speed) //考虑到运动学和动力学的约束
{
		outfile009=fopen("zcs0039.txt","a+");
	int x, y;
	float center_x_right, center_x_left, center_y, center_y_left,dist_r, dist_l,dist_cell_r,dist_cell_l;
	float angle_ahead, phi_left, phi_right, angle;

	angle_ahead = 90;
	phi_left  = 180;
	phi_right = 0;

	// center_x_[left|right] is the centre of the circles on either side that
	// are blocked due to the robot's dynamics.  Units are in cells, in the robot's
	// local coordinate system (+y is forward).
	center_x_right = CENTER_X + (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
	center_x_left = CENTER_X - (Min_Turning_Radius[speed] / (float)CELL_WIDTH);
	center_y = CENTER_Y ;
	center_y_left = CENTER_Y ;



	Blocked_Circle_Radius = Min_Turning_Radius[speed] + ROBOT_RADIUS + Get_Safety_Dist(speed);

	//
	// This loop fixes phi_left and phi_right so that they go through the inside-most
	// occupied cells inside the left/right circles.  These circles are centred at the 
	// left/right centres of rotation, and are of radius Blocked_Circle_Radius.
	// 
	// We have to go between phi_left and phi_right, due to our minimum turning radius.
	//

	//
	// Only loop through the cells in front of us.
	//详见蔡则苏的博士论文p100页《》
	for(x=(int)ceil(WINDOW_DIAMETER/2.0);x<WINDOW_DIAMETER;x++) 

	{
		for(y=0;y<WINDOW_DIAMETER;y++) 
		{
			if (Cell_Mag[x][y] == 0) 
				continue;

			if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) > 0) && 
				(Delta_Angle(Cell_Direction[x][y], phi_right) <= 0)) //障碍物的栅格点在第一象限
			{
				// The cell is between phi_right and angle_ahead

				dist_r = hypot(x-center_x_right , center_y - y) * CELL_WIDTH;//右边运动轨迹中心到栅格点（x，y）的距离
				dist_cell_r=hypot(x-CENTER_X , CENTER_Y - y) * CELL_WIDTH;
				if (dist_r < Blocked_Circle_Radius) 
				{ 
					phi_right = Cell_Direction[x][y]+atan((float)(Blocked_Circle_Radius) / (float)(dist_cell_r));
					//		phi_right = Cell_Direction[x][y];
				}
			} 
			else if ((Delta_Angle(Cell_Direction[x][y], angle_ahead) <= 0) && 
				(Delta_Angle(Cell_Direction[x][y], phi_left) > 0)) //障碍物的栅格点在第二象限
			{
				// The cell is between phi_left and angle_ahead

				dist_l = hypot(center_x_left - x, center_y - y) * CELL_WIDTH;//左边运动轨迹中心到栅格点（x，y）的距离
				dist_cell_l=hypot(x-CENTER_X , CENTER_Y - y) * CELL_WIDTH;
				if (dist_l < Blocked_Circle_Radius) 
				{ 
					phi_left = Cell_Direction[x][y]-atan((float)(Blocked_Circle_Radius) / (float)(dist_cell_l));
					//		phi_left = Cell_Direction[x][y];
				}
			}
		}
	}

	//
	// Mask out everything outside phi_left and phi_right
	//


	for(x=0;x<HIST_SIZE;x++) 
	{
		angle = x * SECTOR_ANGLE;//扇区角度


		if ((Hist[x] == 0) && (((Delta_Angle((float)angle, phi_right) <= 0) && 
			(Delta_Angle((float)angle, angle_ahead) >= 0)) || 
			((Delta_Angle((float)angle, phi_left) >= 0) &&
			(Delta_Angle((float)angle, angle_ahead) <= 0)))) 
		{
			Hist[x] = 0;
		} 
		else 
		{
			Hist[x] = 1;
		}

		fprintf(outfile009," fffffff  %f      %f      \n",Hist[x],angle);

	}

	fclose(outfile009);
	return(1);
}


int VFH_Algorithm::Set_Motion( float &speed, float &turnrate, int actual_speed ) //设置运动
{
		outfile009=fopen("zcs0049.txt","a+");
	// This happens if all directions blocked, so just spin in place
	float Picked_Angle_motion=0;
//	Picked_Angle_motion=Picked_Angle-90;/////////////////////////因为本系统设定X轴为横轴，Y轴为纵轴，机器人的前进方向为Y方向，故需要减去90度
	Picked_Angle_motion=Picked_Angle;/////////////////////////因为本系统设定Y轴为横轴，X轴为纵轴，机器人的前进方向为X方向
	if (speed <= 0) 
	{
		fprintf(outfile009," stop    %f  %f   \n",turnrate,speed);
		//   turnrate = GetMaxTurnrate( actual_speed );//速度小于等于0，转向速度为最大转向速度
		//turnrate =2;
		turnrate =0;
		speed = 0;
	}
	FILE *alloutX;
alloutX = fopen("alloxxx.txt","a+");

	
//	else //xhy注销
	{
		//printf("Picked %f\n", Picked_Angle);
		if ((Picked_Angle_motion > 0) && (Picked_Angle_motion <= 90)) 
		{//如果选定的角在270至360度之间，转向速度反向选择
			//		turnrate = -1 * GetMaxTurnrate( actual_speed );
			if (Picked_Angle_motion<=10)
			{
				speed = 0;
				turnrate = -(int)rint(((float)(90-Picked_Angle_motion ) / 240.0) * GetMaxTurnrate( actual_speed ));


			}
			else if (Picked_Angle_motion<=30)
			{
				speed=speed/4;
				turnrate = -(int)rint(((float)(90-Picked_Angle_motion ) / 360.0) * GetMaxTurnrate( actual_speed ));
			}
			else if (fabs(Picked_Angle_motion)<=50)
			{
				speed=speed/2;
				turnrate = -(int)rint(((float)(90-Picked_Angle_motion ) /140.0) * GetMaxTurnrate( actual_speed ));

			}
			else if (Picked_Angle_motion<=70)
			{
				turnrate =-(int)rint(((float)(90-Picked_Angle_motion ) / 140.0) * GetMaxTurnrate( actual_speed ));

			}
			
			else 
			{
				turnrate = -(int)rint(((float)(90-Picked_Angle_motion ) /180.0) * GetMaxTurnrate( actual_speed ));

			}

			fprintf(outfile009," turnrate   %f  %f  %f  \n",turnrate,Picked_Angle_motion,GetMaxTurnrate( actual_speed ));
		} 

		else if((Picked_Angle_motion > 90) && (Picked_Angle_motion <= 180)) //0~180//如果选定的角在0至180度之间，转向速度选择下面的值//xhy:90-180
		{
			// 
			if (fabs(Picked_Angle_motion)<=110)
			{
				turnrate = (int)rint(((float)(Picked_Angle_motion -90) / 180.0) * GetMaxTurnrate( actual_speed ));

			}
			else if (fabs(Picked_Angle_motion)<=130)
			{
				turnrate = (int)rint(((float)(Picked_Angle_motion-90 ) / 140.0) * GetMaxTurnrate( actual_speed ));
			}
			else if (fabs(Picked_Angle_motion)<=150)
			{
				speed=speed/2;
				turnrate = (int)rint(((float)(Picked_Angle_motion-90 ) / 140.0) * GetMaxTurnrate( actual_speed ));

			}
			else if (fabs(Picked_Angle_motion)<=170)
			{
				speed=speed/4;
				turnrate = (int)rint(((float)(Picked_Angle_motion-90 ) / 240.0) * GetMaxTurnrate( actual_speed ));

			}
			//else if (fabs(Picked_Angle_motion)<=70)
			//{
			//	turnrate = (int)rint(((float)(Picked_Angle_motion ) / 360.0) * GetMaxTurnrate( actual_speed ));

			//}
			//else if (fabs(Picked_Angle_motion)<=120)
			//{
			//	turnrate = (int)rint(((float)(Picked_Angle_motion ) / 360.0) * GetMaxTurnrate( actual_speed ));
			//}
			else 
			{
				speed = 0;
				turnrate = (int)rint(((float)(Picked_Angle_motion -90) / 240.0) * GetMaxTurnrate( actual_speed ));

			}


			//	turnrate = /*(int)*/rint(((float)(Picked_Angle ) / 360.0) * GetMaxTurnrate( actual_speed ));

			//	turnrate = rint((float)(Picked_Angle)/75);
		
			//			//fprintf(outfile,"turnrate %d \n",turnrate);
			/*	if (turnrate > GetMaxTurnrate( actual_speed ))
			{
			turnrate = GetMaxTurnrate( actual_speed );
			} 
			else if (turnrate < (-1 * GetMaxTurnrate( actual_speed ))) 
			{
			turnrate = -1 * GetMaxTurnrate( actual_speed );
			}
			*/
			//			//fprintf(outfile,"turnrate_new %d \n",turnrate);
			//fprintf(allo1,"turnrate2 %d \n\n",turnrate);
			/*fclose(allo1);*/
			//      if (abs(turnrate) > (0.9 * GetMaxTurnrate( actual_speed ))) {
			//        speed = 0;
			//      }
	}
		else
		{
			speed = 0;
		}
		//if ((Picked_Angle_motion > 90)&&(Picked_Angle_motion <270))
		//{
		//	turnrate = 0.8* GetMaxTurnrate( actual_speed );
		//}
		//else 
		//{
		//	turnrate = -0.8* GetMaxTurnrate( actual_speed );
		//}
		//	fprintf(outfile009," turnrate 000000  %f  %f  %d  \n",turnrate,Picked_Angle_motion,GetMaxTurnrate( actual_speed ));
		
	}
	if(plan.speed_line_pos==0&&(Picked_Angle_motion<75||Picked_Angle_motion>115))
	{
		speed = 0;
		turnrate=6*turnrate;
	}
				fprintf(alloutX ," %d  %f  %f %f   %f\n",plan.speed_line_pos,plan.speed_line,Picked_Angle_motion,turnrate);
			fclose(alloutX );
	turnrate=3*turnrate;
	//  speed and turnrate have been set for the calling function -- return.
	fclose(outfile009);
	return(1);
}

