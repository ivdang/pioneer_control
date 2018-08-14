#ifndef robot_control.h
#define robot_control.h

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
void callback(const sensor_msgs::LaserScan::ConstPtr& msg)

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
	const msg, bool value that defines if robot needs to turn
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
-Para: const msg
-Description:
	This function helps robot reduce the chance hitting to the wall or obstacles 
   	when it makes a mistake and get too close to something
  	It will compare the distance at 0 degree and 180 degree and move towards the bigger one.
-Return value: none
****************************************************************/
void turnAround (const sensor_msgs::LaserScan::ConstPtr&);


const float DANGER_ZONE = 0.5;
const float OBSTACLE_DIST = 1;
const float ROBOT_BODY_WIDTH = 0.4;

const int INDEX_AT_0_DEGREE = 90;
const int INDEX_AT_30_DEGREE = 180;
const int INDEX_AT_45_DEGREE = 225;
const int INDEX_AT_80_DEGREE = 333;
const int INDEX_AT_90_DEGREE = 363;
const int INDEX_AT_100_DEGREE = 393;
const int INDEX_AT_135_DEGREE = 500;
const int INDEX_AT_150_DEGREE = 545;
const int INDEX_AT_180_DEGREE = 636;
const int INDEX_AT_240_DEGREE = 725;


bool isTurning = false;
bool isMustTurnOperation = false;

float mustTurnToRads;
float haveTurnedRads = 0.0;
float currentTurningAngular = 0.0;

ros::Publisher pubGlobal;
geometry_msgs::Twist msgPub;

#endif