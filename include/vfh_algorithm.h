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
#include "Plan_Path_VFH.h"
#include <WinSock2.h>
#ifndef VFH_ALGORITHM_H
#define VFH_ALGORITHM_H


#define PLAYER_LASER_MAX_SAMPLES 682
#define PLAYER_SONAR_MAX_SAMPLES  64;

 #define MIN(a,b) ((a < b) ? a : b)
 #define MAX(a,b) ((a > b) ? a : b)
#define rint(R) (((R)>0?((R)+0.5):((R)-0.5)))

#ifndef M_PI
	#define M_PI        3.14159265358979323846
#endif

// Convert radians to degrees
#ifndef RTOD
#define RTOD(r) ((r) * 180 / M_PI)
#endif

// Convert degrees to radians
#ifndef DTOR
#define DTOR(d) ((d) * M_PI / 180)
#endif

// Normalize angle to domain -pi, pi
#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif




#include<vector>
#include <iostream>
#include <highgui.h>
#include <cv.h>
//#include <libplayercore/playercore.h>
/*
@par Configuration file options

- cell_size (length)
  - Default: 0.1 m
  - Local occupancy map grid size
- window_diameter (integer)
  - Default: 61
  - Dimensions of occupancy map (map consists of window_diameter X
    window_diameter cells).
- sector_angle (integer)
  - Default: 5
  - Histogram angular resolution, in degrees.
- safety_dist_0ms (length)
  - Default: 0.1 m
  - The minimum distance the robot is allowed to get to obstacles when stopped.
- safety_dist_1ms (length)
  - Default: safety_dist_0ms
  - The minimum distance the robot is allowed to get to obstacles when
    travelling at 1 m/s.
- max_speed (length / sec)
  - Default: 0.2 m/sec
  - The maximum allowable speed of the robot.
- max_speed_narrow_opening (length / sec)
  - Default: max_speed
  - The maximum allowable speed of the robot through a narrow opening
- max_speed_wide_opening (length / sec)
  - Default: max_speed
  - The maximum allowable speed of the robot through a wide opening
- max_acceleration (length / sec / sec)
  - Default: 0.2 m/sec/sec
  - The maximum allowable acceleration of the robot.
- min_turnrate (angle / sec)
  - Default: 10 deg/sec
  - The minimum allowable turnrate of the robot.
- max_turnrate_0ms (angle / sec)
  - Default: 40 deg/sec
  - The maximum allowable turnrate of the robot when stopped.
- max_turnrate_1ms (angle / sec)
  - Default: max_turnrate_0ms
  - The maximum allowable turnrate of the robot when travelling 1 m/s.
- min_turn_radius_safety_factor (float)
  - Default: 1.0
  - ?
- free_space_cutoff_0ms (float)
  - Default: 2000000.0
  - Unitless value.  The higher the value, the closer the robot will
    get to obstacles before avoiding (while stopped).
- free_space_cutoff_1ms (float)
  - Default: free_space_cutoff_0ms
  - Unitless value.  The higher the value, the closer the robot will
    get to obstacles before avoiding (while travelling at 1 m/s).
- obs_cutoff_0ms (float)
  - Default: free_space_cutoff_0ms
  - ???
- obs_cutoff_1ms (float)
  - Default: free_space_cutoff_1ms
  - ???
- weight_desired_dir (float)
  - Default: 5.0
  - Bias for the robot to turn to move toward goal position.
- weight_current_dir (float)
  - Default: 3.0
  - Bias for the robot to continue moving in current direction of travel.
- distance_epsilon (length)
  - Default: 0.5 m
  - Planar distance from the target position that will be considered
    acceptable.
- angle_epsilon (angle)
  - Default: 10 deg
  - Angular difference from target angle that will considered acceptable.
- Stall escape options.  If the underlying position2d device reports a
  stall, this driver can attempt a blind escape procedure.  It does so by
  driving forward or backward while turning for a fixed amount of time.  If
  the escape fails (i.e., the stall is still in effect), then it will try again.
  - escape_speed (length / sec)
    - Default: 0.0
    - If non-zero, the translational velocity that will be used while trying
      to escape.
  - escape_time (float)
    - Default: 0.0
    - If non-zero, the time (in seconds) for which an escape attempt will be 
      made.
  - escape_max_turnspeed (angle / sec)
    - Default: 0.0
    - If non-zero, the maximum angular velocity that will be used when 
      trying to escape.

@par Example
@verbatim

*/

//class PlayerTime
//{
//public:
//	// fills in the timeval struct with the current time, from the right
//	// source.
//	//
//	// returns 0 on success, -1 on error    
//	virtual int GetTime(struct timeval* time) = 0;
//	virtual int GetTimeDouble(double* time) = 0;
//	virtual ~PlayerTime() {};
//};
//
//PlayerTime* GlobalTime;

using namespace std;

#define TIMESUB(a, b, result)                                                 \
	do {                                                                        \
	(result)->tv_sec = (a)->tv_sec - (b)->tv_sec;                             \
	(result)->tv_usec = (a)->tv_usec - (b)->tv_usec;                          \
	if ((result)->tv_usec < 0) {                                              \
	--(result)->tv_sec;                                                     \
	(result)->tv_usec += 1000000;                                           \
	}                                                                         \
	} while (0)



class VFH_Algorithm
{
public:
    VFH_Algorithm( double cell_size=20,
                   int window_diameter=301,
                   int sector_angle=5,
                   double safety_dist_0ms=300,
                   double safety_dist_1ms=300, 
                   int max_speed=800,
                   int max_speed_narrow_opening=30,
                   int max_speed_wide_opening=30,
                   int max_acceleration=3,
                   int min_turnrate=10,
                   int max_turnrate_0ms=20,
                   int max_turnrate_1ms=20,
                   double min_turn_radius_safety_factor=1.0,
                   double free_space_cutoff_0ms=5000000.0,
                   double obs_cutoff_0ms=8000000.0,
                   double free_space_cutoff_1ms=8000000.0,
                   double obs_cutoff_1ms=9000000.0,
                   double weight_desired_dir=5.0,
                   double weight_current_dir=3.0 );

	
    ~VFH_Algorithm();

    int Init();
    
    // Choose a new speed and turnrate based on the given laser data and current speed.
    //
    // Units/Senses:
    //  - goal_direction in degrees, 0deg is to the right.
    //  - goal_distance  in mm.
    //  - goal_distance_tolerance in mm.
    //
    int Update_VFH( float current_speed,  
                    float goal_direction,
                    float goal_distance,
                    float goal_distance_tolerance,
                    float &chosen_speed, 
                    float &chosen_turnrate );

    // Get methods
    int   GetMinTurnrate() { return MIN_TURNRATE; }
    // Angle to goal, in degrees.  0deg is to our right.
    float GetDesiredAngle() { return Desired_Angle; }
    float GetPickedAngle() { return Picked_Angle; }

    // Max Turnrate depends on speed
    int GetMaxTurnrate( int speed );
    int GetCurrentMaxSpeed() { return Current_Max_Speed; }

    // Set methods
    void SetRobotRadius( float robot_radius ) { this->ROBOT_RADIUS = robot_radius; }
    void SetMinTurnrate( int min_turnrate ) { MIN_TURNRATE = min_turnrate; }
    void SetCurrentMaxSpeed( int Current_Max_Speed );
	
    // The Histogram.
    // This is public so that monitoring tools can get at it; it shouldn't
    // be modified externally.
    // Sweeps in an anti-clockwise direction.
    float *Hist;
	float *Hist_Average_Distance;
	double index_poly[5];
	void Compute_Sector_Average_Distance(double Hist_value,double index_poly[5]);
	std::complex<double>  x_result[4];

private:

    // Functions

    int VFH_Allocate();

    float Delta_Angle(int a1, int a2);
    float Delta_Angle(float a1, float a2);
    int Bisect_Angle(int angle1, int angle2);

    bool Cant_Turn_To_Goal();

    // Returns 0 if something got inside the safety distance, else 1.
    int Calculate_Cells_Mag(  int speed /*,FILE *outfile*/);
    // Returns 0 if something got inside the safety distance, else 1.
    int Build_Primary_Polar_Histogram(  float speed /*,FILE *outfile*/);
    int Build_Binary_Polar_Histogram(float speed);
    int Build_Masked_Polar_Histogram(float speed);
    int Select_Candidate_Angle();
    int Select_Direction();
    int Set_Motion( float &speed, float &turnrate, int current_speed );

    // AB: This doesn't seem to be implemented anywhere...
    // int Read_Min_Turning_Radius_From_File(char *filename);

    void Print_Cells_Dir(FILE *outfile);
    void Print_Cells_Mag(FILE *outfile);
    void Print_Cells_Dist(FILE *outfile);
    void Print_Cells_Sector(FILE *outfile);
    void Print_Cells_Enlargement_Angle(FILE *outfile);
    void Print_Hist(FILE *outfile,int type);

    // Returns the speed index into Cell_Sector, for a given speed in mm/sec.
    // This exists so that only a few (potentially large) Cell_Sector tables must be stored.
    int Get_Speed_Index( float speed );

    // Returns the safety dist in mm for this speed.
    int Get_Safety_Dist( float speed );

    float Get_Binary_Hist_Low( float speed );
    float Get_Binary_Hist_High( float speed );

	void fittingCurve(int n,double xx[],double yy[],int polu_n, double index[]);
//	void gauss_solve(int n,double A[],double x[],double b[]);
	
    // Data

    float ROBOT_RADIUS;           // millimeters
    int CENTER_X;                 // cells
    int CENTER_Y;                 // cells
    int HIST_SIZE;                // sectors (over 360deg)

    float CELL_WIDTH;             // millimeters
    int WINDOW_DIAMETER;          // cells
    int SECTOR_ANGLE;             // degrees
    float SAFETY_DIST_0MS;        // millimeters
    float SAFETY_DIST_1MS;        // millimeters
    int Current_Max_Speed;        // mm/sec
    int MAX_SPEED;                // mm/sec
    int MAX_SPEED_NARROW_OPENING; // mm/sec
    int MAX_SPEED_WIDE_OPENING;   // mm/sec
    int MAX_ACCELERATION;         // mm/sec/sec
    int MIN_TURNRATE;             // deg/sec -- not actually used internally

    int NUM_CELL_SECTOR_TABLES;

    // Scale turnrate linearly between these two
    int MAX_TURNRATE_0MS;       // deg/sec
    int MAX_TURNRATE_1MS;       // deg/sec
    double MIN_TURN_RADIUS_SAFETY_FACTOR;
    float Binary_Hist_Low_0ms, Binary_Hist_High_0ms;
    float Binary_Hist_Low_1ms, Binary_Hist_High_1ms;
    float U1, U2;
    float Desired_Angle, Dist_To_Goal, Goal_Distance_Tolerance;
    float Picked_Angle, Last_Picked_Angle;
    int   Max_Speed_For_Picked_Angle;

    // Radius of dis-allowed circles, either side of the robot, which
    // we can't enter due to our minimum turning radius.
    float Blocked_Circle_Radius;

    std::vector<std::vector<float> > Cell_Direction;
    std::vector<std::vector<float> > Cell_Base_Mag;
    std::vector<std::vector<float> > Cell_Mag;
    std::vector<std::vector<float> > Cell_Dist;      // millimetres
    std::vector<std::vector<float> > Cell_Enlarge;

    // Cell_Sector[x][y] is a vector of indices to sectors that are effected if cell (x,y) contains
    // an obstacle.  
    // Cell enlargement is taken into account.
    // Acess as: Cell_Sector[speed_index][x][y][sector_index]
    std::vector<std::vector<std::vector<std::vector<int> > > > Cell_Sector;
    std::vector<float> Candidate_Angle;
    std::vector<int> Candidate_Speed;

    double dist_eps;
    double ang_eps;

    float *Last_Binary_Hist;

    // Minimum turning radius at different speeds, in millimeters
    std::vector<int> Min_Turning_Radius;

    // Keep track of last update, so we can monitor acceleration
    timeval last_update_time;

    int last_chosen_speed;
	
	//FILE *alloutfile9;
//	FILE *outfile;
	//FILE *outfilehist;
//	FILE *outfile3;

	float angle_ext_robot;
};

#endif
