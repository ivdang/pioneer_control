
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <robot_control.h>

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 	if (isMustTurnOperation) 
  	{
    	if (haveTurnedRads < mustTurnToRads) 
    	// check if the robot turn enough or not
    	{
      	haveTurnedRads += currentTurningAngular;
    	} 
	    else
	    // if robot've turned enough, I reset all these variables 
	    {
	    	isMustTurnOperation = false;
	    	mustTurnToRads = 0.0;
	      	haveTurnedRads = 0.0;
	      	currentTurningAngular = 0.0;
	    }
  	} 
  	else 
  	{
  		doNormalOperation(msg);
  	}
}

void doNormalOperation(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float rw, lw, m, rightGap, leftGap;
	bool hasRiO = false, hasLeO = false;
	// m store the distance in froNt of the robot (90 degree)
	m = msg -> ranges[INDEX_AT_90_DEGREE];

	// check if there is any obstacle in right or left
	hasRiO = checkRightObstacle (msg);
	hasLeO = checkLeftObstacle (msg);

	keepBalance(msg);

  	if ( m  < DANGER_ZONE)
  	// if robot makes a mistake and it gets too close to something, 
  	// it will turn to get out of that zone
  	{
   		turnAround (msg); 
  	}
  	else if (m < OBSTACLE_DIST)
  	//there is an obstacle in front of robot
  	{
  		// leftGap/rightGap store the gap from the obstacle to the left/right wall
	    leftGap = findLeftGap (msg); 
	    rightGap = findRightGap (msg);
	    
	    // checking which gap is bigger and if that gap is big enough for the robot moving through
	    if (leftGap >= rightGap && leftGap > ROBOT_BODY_WIDTH)
	    {
	    	// to avoid an obstacle in front of robot, it must turn enough rads -> passing true
	      	moveToLeft(msg, true);
	    }
	    else if ( leftGap < rightGap && rightGap > ROBOT_BODY_WIDTH)
	    {
	    	// to avoid an obstacle in front of robot, it must turn enough rads -> passing true
	      	moveToRight(msg, true);
	    }
  	}
  	/**
	else if (hasRiO && hasLeO)
	// this case means there obstacles on both sides but there is nothing in front of the robot,
	// so it just need to keep balance to get through
	{
    	keepBalance(msg);
	}
	**/

	// if there is no obstacle in front of it, look at the right then the left
	else if(hasRiO)
	{
		moveToLeft (msg, false);
	}
	else if (hasLeO)
	{
		moveToRight(msg, false);
	}
}

void keepBalance (const sensor_msgs::LaserScan::ConstPtr& ms)
{
 	float rw, lw;
  	
  	// the robot will check on the left and the right by looking
  	// two supplementary angles at the same time, so it can modify its velocity.
  	// It starts looking from 30 to 90 degree.
  	for (int i = INDEX_AT_30_DEGREE; i <= INDEX_AT_90_DEGREE; i++)
  	{
  		//rw is the distance robot can see at i degree
  		//lw is the distance robbot can see at (180 - i) degree
	    rw = ms -> ranges[i];
	    lw = ms -> ranges[INDEX_AT_240_DEGREE - i];

	    //if robot sees nan value, it means there is a lot of space in that direction
	    //I let the robot go to that direction
	    if (isnan(rw))
	    {  
	      msgPub.linear.x = 0.3;
	      msgPub.angular.z = -0.5; // -0.3 then - 0.5
	      pubGlobal.publish(msgPub);
	      break;
	    }
	    else if (isnan(lw))
	    {  
	      msgPub.linear.x = 0.3;
	      msgPub.angular.z = 0.3; // 0.3
	      pubGlobal.publish(msgPub);
	      break;
	    }
	    /*
	    else if (isnan(lw) && isnan(rw))
	    {
	      msgPub.angular.z = 0; break;
	    }
	    */
	    else if (lw!=rw)
	    {
	      msgPub.linear.x = 0.3;
	      msgPub.angular.z = (lw-rw)*0.5;
	      pubGlobal.publish(msgPub);

	      // when the difference < 0.1,
	      // robot is quite at the center of the road, no need to turn
	      if ( abs(lw - rw) < 0.1) 
	          break; 
	    }
  	}
}

void turnAround (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  	msgPub.linear.x = -0.3;
  	float lw = ms -> ranges [INDEX_AT_180_DEGREE];
  	float rw = ms -> ranges [INDEX_AT_0_DEGREE];
  	if (lw > rw)
    	msgPub.angular.z = 0.5;
  	else
    	msgPub.angular.z = -0.5;
  	pubGlobal.publish(msgPub);
}

bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  	float rw;
  	//robot look from the center to 45 degree
  	for (int i = INDEX_AT_80_DEGREE; i > INDEX_AT_45_DEGREE ; i--)
  	{
    	rw = ms -> ranges[i];
    	if (rw < OBSTACLE_DIST)
      		return true;
 	}
  	return false;
}
//check to see is there any obstacle on the right of robot (from 45 to 80)
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  	float lw;
  	for (int i= INDEX_AT_100_DEGREE; i < INDEX_AT_135_DEGREE; i++)
  	{
    	lw = ms -> ranges[i];
   		if (lw < OBSTACLE_DIST )
      		return true;
  	}
  	return false;
}

void moveToLeft (const sensor_msgs::LaserScan::ConstPtr& ms, bool shouldCheckTurnedRads)
{
 	float rw;
  	if (shouldCheckTurnedRads) 
  	{
	    msgPub.linear.x = 0.2;
	    msgPub.angular.z = 0.3; //0.4
	    pubGlobal.publish(msgPub);
	    isMustTurnOperation = true;
	    currentTurningAngular = 0.3;
	    mustTurnToRads = 5;
  	} 
  	else 
  	{  
	    msgPub.linear.x = 0.3;
	    msgPub.angular.z = 0.3; //0.4
	    pubGlobal.publish(msgPub);
	    rw = ms -> ranges[INDEX_AT_45_DEGREE];//WAS 240 - update rw to stop turning at 45 degree
	    if (rw > 1.8 || isnan(rw)) // clear at the angle 45 on the right, continue keep balance
		    keepBalance(ms);
  	}
}

void moveToRight (const sensor_msgs::LaserScan::ConstPtr& ms, bool shouldCheckTurnedRads)
{ 
    float lw;
    if (shouldCheckTurnedRads) 
    { 
      	msgPub.linear.x = 0.2; //0.4
	    msgPub.angular.z = -0.3;   //0.4 
	    pubGlobal.publish(msgPub);
	    isMustTurnOperation = true;
	    currentTurningAngular = 0.3;
	    mustTurnToRads = 5;
    } 
    else 
    { 
      msgPub.linear.x = 0.3; //0.4
      msgPub.angular.z = -0.3;   //0.4 
      pubGlobal.publish(msgPub);
      lw = ms -> ranges[INDEX_AT_135_DEGREE];//update rw to stop turning at 135 degree
      if (lw > 1.8 || isnan(lw))
        keepBalance(ms);
    }
}

float findRightGap (const sensor_msgs::LaserScan::ConstPtr& ms)
{
	float currentLaser, nextLaser, rightGap=0;
	float robot_ob_diff; //difference between right side of the robot and right border of the obstacle
	float maxRight = ms -> ranges[INDEX_AT_0_DEGREE];
	float middle = ms -> ranges[INDEX_AT_90_DEGREE];
	int i;
  	for ( i = INDEX_AT_90_DEGREE; i >= INDEX_AT_30_DEGREE;i--)
  	{
    	currentLaser = ms -> ranges[i];
    	nextLaser = ms -> ranges[i-1];
    	if ((abs(currentLaser-nextLaser)/currentLaser)> 0.2)
    	// the difference is bigger than 20%
    	{
      		if (currentLaser > middle)
      		{
        		robot_ob_diff = sqrt(pow(currentLaser,2)-pow(middle,2));
        		if (maxRight > robot_ob_diff)
      				rightGap = maxRight - robot_ob_diff;
      		}
      		break;
    	}
  	}
  	return rightGap;
}

float findLeftGap (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  	float currentLaser, nextLaser, leftGap = 0;
  	float robot_ob_diff; //difference between left side of the robot and left border of the obstacle
  	float maxLeft = ms -> ranges[636];
  	float middle = ms -> ranges[363];
  	int j;
  	for (j= INDEX_AT_90_DEGREE;j <= INDEX_AT_150_DEGREE;j++)
  	{
    	currentLaser = ms -> ranges[j];
    	nextLaser = ms -> ranges[j+1];
    	if ((abs(currentLaser-nextLaser)/currentLaser) > 0.2)
		{
      		if(currentLaser > middle)
      		//avoid 
      		{
		        robot_ob_diff = sqrt(pow(currentLaser,2)-pow(middle,2));
		        if (maxLeft >= robot_ob_diff)
		        	leftGap = maxLeft - robot_ob_diff;
      		}
      		break;
    	}
  	}
  	return leftGap;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "raceSolver");
  ros::NodeHandle nodeHd;
  pubGlobal = nodeHd.advertise<geometry_msgs::Twist>("/cmd_vel",100);
  ros::Subscriber sub = nodeHd.subscribe("/direct/scan",100, callback);
  ros::spin();
  return 0;
}



