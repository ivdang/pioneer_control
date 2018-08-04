#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
ros::Publisher pubGlobal;
geometry_msgs::Twist msgPub;

void keepBalance (const sensor_msgs::LaserScan::ConstPtr&);
bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr&);
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr&);
float findLeftGap (const sensor_msgs::LaserScan::ConstPtr&);
float findRightGap (const sensor_msgs::LaserScan::ConstPtr&);
void moveToLeft (const sensor_msgs::LaserScan::ConstPtr& );
void moveToRight (const sensor_msgs::LaserScan::ConstPtr& );
void turnAround (); 

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  const float DANGER_ZONE = 0.5;
  const float OBSTACLE_DIST = 2;
  const float ROBOT_BODY_WIDTH = 0.4;
  float rw, lw, m,rightGap, leftGap;
  bool hasRiO, hasLeO;
  m = msg -> ranges[90];
  rw = msg -> ranges[0];
  hasRiO = checkRightObstacle (msg);//check to see is there any obstacle on the right of robot (from 45 to 80
  hasLeO = checkLeftObstacle (msg);
  
  keepBalance(msg);
  ROS_INFO ("\nMiddle: [%.2f]", m);
  
  if ( m  < DANGER_ZONE) 
    turnAround (); // if robot makes a mistake and it gets too close to the wall or when it go to the dead end, it will turn around to find another way
  else if (m < OBSTACLE_DIST) //there is an obstacle in front of robot
  {
    leftGap = findLeftGap (msg); // find the gap between the ob and the left wall
    rightGap = findRightGap (msg); // find the gap between the ob and the right wall
    ROS_INFO ("\nLeft - Right: [%.2f][%.2f]",leftGap, rightGap);
    
    // this if-else statement to check which one is wider to go through
    // the robot can also know if the bigger gap is fit its body or not, then it can make right decision
    if (leftGap >= rightGap && leftGap > ROBOT_BODY_WIDTH)
      moveToLeft(msg);
    else if ( leftGap < rightGap && rightGap > ROBOT_BODY_WIDTH)
      moveToRight(msg);
  }
  // if there is no obstacle in front of it, look at the right then the left
  else if(hasRiO)
    moveToLeft (msg);
  else if (hasLeO)
    moveToRight(msg);
}

void keepBalance (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float rw, lw;
  msgPub.linear.x = 0.3;
  for (int i= 30; i <=90; i++)
  {
    rw = ms -> ranges[i];
    lw = ms -> ranges[179-i];
    if (lw!= rw)
    {
      msgPub.angular.z = (lw-rw)*0.75;
      pubGlobal.publish(msgPub);
       if ( abs(lw - rw) < 0.005)
      	break; //robot is quite at the center of the road, no need to turn
    }
  }
}
void turnAround ()
{
  msgPub.linear.x = -0.1;
  msgPub.angular.z = 0.5;
  pubGlobal.publish(msgPub);
}

bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float rw;
  for (int i = 80; i > 45 ; i--)
  {
      rw = ms -> ranges[i];
      if (rw < .8)
      {
	ROS_INFO ("\nOb on right:[%d] [%.2f]", i, rw);
	return true;
      }
  }
  return false;
}
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float lw;
  for (int i= 100; i < 135; i++)
  {
    lw = ms -> ranges[i];
    if (lw < .8)
    {
      ROS_INFO ("\nOb on left:[%d] [%.2f]", i, lw);
      return true;
    }
  }
  return false;
}
void moveToLeft (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float rw;
   msgPub.linear.x = 0.4;
   msgPub.angular.z = 0.4;	 
   pubGlobal.publish(msgPub);
      
   rw = ms -> ranges[50];//update rw to stop turning
   if (rw > 1.8) // clear at the angle 50 on the right, continue moving straight
       keepBalance(ms);
     ROS_INFO ("\n Left turn");
}
void moveToRight (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float lw;
   msgPub.linear.x = 0.4;
   msgPub.angular.z = -0.4;	 
   pubGlobal.publish(msgPub);
    
   lw = ms -> ranges[110];//update rw to stop turning
   if (lw > 1.8)
     keepBalance(ms);
   ROS_INFO ("\n Right turn");
}
float findRightGap (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float firstBorder, secondBorder, rightGap=0; 
  float robot_ob_diff; //difference between right side of the robot and right border of the obstacle 
  float maxRight = ms -> ranges[0];
  float middle = ms -> ranges[90];
  int i;
  for ( i=90;i>=30;i--)
    {
      firstBorder = ms -> ranges[i];
      secondBorder = ms -> ranges[i-1];
      if ((abs(firstBorder-secondBorder)/firstBorder)> 0.2)
      {
	if(firstBorder > middle)
	{
	  ROS_INFO ("\n[Right]\nAngle 1: [%.d] - FirstBorder: [%.2f] \nAngel 2: [%.d] - SecondBorder:[%.2f]",i,firstBorder, i-1, secondBorder);
	  robot_ob_diff = sqrt(pow(firstBorder,2)-pow(middle,2));
	  if (maxRight > robot_ob_diff)
	    rightGap = maxRight - robot_ob_diff;
	  // ROS_INFO ("\n -Right Gap: [%.2f] [%.2f]", secondBorder,rightGap);
	}
	break;
      }
    }
  return rightGap;
}
float findLeftGap (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float firstBorder, secondBorder, leftGap = 0; 
  float robot_ob_diff; //difference between left side of the robot and left border of the obstacle 
  float maxLeft = ms -> ranges[179];
  float middle = ms -> ranges[90];
  int j;
  for (j=90;j <= 150;j++)
  {
    firstBorder = ms -> ranges[j];
    secondBorder = ms -> ranges[j+1];
    if ((abs(firstBorder-secondBorder)/firstBorder) > 0.2)
      {
	if(firstBorder > middle)
	{
	  ROS_INFO ("\n[Left]\nAngle 1: [%.d] - FirstBorder: [%.2f] \nAngel 2: [%.d] - SecondBorder:[%.2f]",j,firstBorder, j-1, secondBorder);
	  robot_ob_diff = sqrt(pow(firstBorder,2)-pow(middle,2));
	  if (maxLeft >= robot_ob_diff)
	    leftGap = maxLeft - robot_ob_diff;
	  // ROS_INFO ("\nHyp - Left Gap: [%.2f] [%.2f]", firstBorder,leftGap);
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
