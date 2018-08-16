//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Project: Obstacle Avoindace for Pioneer robot 														//
// Written by: Vi Dang 																					//
// Project Description: This project was written during Summer Internship 								//
//						at Dr. Holly Yanco's Robotics lab (Umass Lowell University)						//
//						This project helps pioneer robot avoid obstacles around it when it's moving.	//
//						It is NOT a project that leads the robot from one point to another point. 		//
//						It is not guarantee the robot will follow one path only							//
//						But it helps reduce the chance robot hits to obstacles on the way				//
// File name: robot_control.h																			//
// File description: this is the deader file that defines all constants, global variables				//
//					and functions as needed														 		//
// Other files in this project: robot_control.cpp 														//
//////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef robot_control_h
#define robot_control_h

const float DANGER_ZONE = 0.5;
const float OBSTACLE_DIST = 1;
const float ROBOT_BODY_WIDTH = 0.4;

//these constants define index in ranges array that match with angles we use in real life
const int INDEX_AT_0_DEGREE   = 90;
const int INDEX_AT_30_DEGREE  = 180;
const int INDEX_AT_45_DEGREE  = 225;
const int INDEX_AT_80_DEGREE  = 333;
const int INDEX_AT_90_DEGREE  = 363;
const int INDEX_AT_100_DEGREE = 393;
const int INDEX_AT_135_DEGREE = 500;
const int INDEX_AT_150_DEGREE = 545;
const int INDEX_AT_180_DEGREE = 636;
const int INDEX_AT_240_DEGREE = 725;


bool isMustTurnOperation = false;

//mustTurnRads indicates the how many radians the robot must turn before doing next operation 
//this number should be calculated based on different obstacles that robot sees.
//but I just assigned a fixed number to it for now to make it simple.
float mustTurnToRads;

//haveTurnedRads is an accumulator to check if the robot has turned enough radians it needs
float haveTurnedRads = 0.0;
float currentTurningAngular = 0.0;

ros::Publisher pubGlobal;
geometry_msgs::Twist msgPub;

/****************************************************************
Function: callback
*******************
-Para: 
	const sensor message
-Description: 
	This function garantees the robot turn enough specific rads before
	doing the next step if it needs to.
	Otherwise, it will call doNormalOperation
-Return value: none
*****************************************************************/
void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

/****************************************************************
Function: doNormalOperation
****************************
-Para: 
	const sensor message
-Description: 
	This function is called in call back when the robot doesn't need to turn 
	at least a specific rads before it does the next step.
	In this function, robot will check is there any obstacle in front of it or
	on the left or on the right and decide to do an appropriate operation
	to avoid that obstacle.
-Return value: none
*****************************************************************/
void doNormalOperation(const sensor_msgs::LaserScan::ConstPtr& msg);

/****************************************************************
Function: checkRightObstacle
*****************************
-Para: 
	const sensor message
-Description: 
	Check to see if there is any obstacle between 45 degree and 80 degree
-Return value: return true if it see an obstacle, otherwise return false
*****************************************************************/
bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr&);

/****************************************************************
Function: checkLeftObstacle
****************************
-Para: 
	const sensor message
-Description: 
	Check to see if there is any obstacle between 100 degree and 135 degree
-Return value: return true if it see an obstacle, otherwise return false
*****************************************************************/
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr&);

/****************************************************************
Function: findLeftGap
**********************
-Para: 
	const sensor message
-Description: 
	If there is an obstacle in fron of the robot, this function calculate
	the distance between the right edge of the obstacle and the left wall.
-Return value: return the distance from the left edge of the obstacle to the wall
*****************************************************************/
float findLeftGap (const sensor_msgs::LaserScan::ConstPtr&);

/****************************************************************
Function: findRightGap
***********************
-Para: 
	const sensor message
-Description: 
	If there is an obstacle in fron of the robot, this function calculate
	the distance between the right edge of the obstacle and the right wall.
-Return value: return the distance between the right edge of the obstacle and the wall
*****************************************************************/
float findRightGap (const sensor_msgs::LaserScan::ConstPtr&);

/*****************************************************************
Fucntion: keepBalance
**********************
-Para: 
	const sensor message
-Description
	This function helps the robot be stable at the middle of the path,
  	making it turn as soon as possible at a corner.
-Return value: none
*****************************************************************/
void keepBalance (const sensor_msgs::LaserScan::ConstPtr&);

/****************************************************************
Function: moveToLeft
*********************
-Para: 
	const msg, bool value that defines if robot needs to turn
    at least 5 rads to avoid the obstacle before it keeps balance again.  
-Description:
	This function helps robot avoid the right obstacle by moving to the left.
  	If the obstale's in front of the robot, it needs to turn exact 5 rads before balancing again  
  	If the obstacle is completely on the right, robot just lightly turns left and
  	then keeps balance as soon as it sees clear at 45 degree.
-Return value: none
****************************************************************/
void moveToLeft (const sensor_msgs::LaserScan::ConstPtr&, bool);

/****************************************************************
Fuction: moveToRight
*********************
-Para: 
	const sensor message, bool value that defines if robot needs to turn
    at least 5 rads to avoid the obstacle before it keeps balance again.  
-Description:
	This function helps robot avoid the right obstacle by moving to the left.
  	If the obstale is in front of the robot, it need to turn exact 5 rads before balancing again  
  	If the obstacle is completely on the left, robot just lightly turns right and
 	then keeps balance as soon as it sees clear at 135 degree.
-Return value: none
****************************************************************/
void moveToRight (const sensor_msgs::LaserScan::ConstPtr&, bool);

/****************************************************************
Fuction: tunrAround
********************
-Para: const sensor message
-Description:
	This function helps robot reduce the chance hitting to the wall or obstacles 
   	when it makes a mistake and get too close to something
  	It will compare the distance at 0 degree and 180 degree and move towards the bigger one.
-Return value: none
****************************************************************/
void turnAround (const sensor_msgs::LaserScan::ConstPtr&);




#endif
