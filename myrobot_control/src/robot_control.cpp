#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr&);
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr&);

float findLeftGap (const sensor_msgs::LaserScan::ConstPtr&);
float findRightGap (const sensor_msgs::LaserScan::ConstPtr&);

void keepBalance (const sensor_msgs::LaserScan::ConstPtr&);
void moveToLeft (const sensor_msgs::LaserScan::ConstPtr&, bool);
void moveToRight (const sensor_msgs::LaserScan::ConstPtr&, bool);
void turnAround (const sensor_msgs::LaserScan::ConstPtr&);
void doNormalOperation(const sensor_msgs::LaserScan::ConstPtr& msg);

ros::Publisher pubGlobal;
geometry_msgs::Twist msgPub;

const float DANGER_ZONE = 0.5;
const float OBSTACLE_DIST = 1;
const float ROBOT_BODY_WIDTH = 0.4;

bool isTurning = false;
bool isMustTurnOperation = false;

float mustTurnToRads;
float haveTurnedRads = 0.0;
float currentTurningAngular = 0.0;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
 if (isMustTurnOperation)
  {
    if (haveTurnedRads < mustTurnToRads) {
      haveTurnedRads += currentTurningAngular;
    } 
    else 
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
  m = msg -> ranges[363];
  hasRiO = checkRightObstacle (msg);//check to see is there any obstacle on the right of robot (from 45 to 80
  hasLeO = checkLeftObstacle (msg);

  keepBalance(msg);

  if ( m  < DANGER_ZONE)
   turnAround (msg); // if robot makes a mistake and it gets too close to the wall or when it go to the dead end, it will turn to find another way
  else if (m < OBSTACLE_DIST) //there is an obstacle in front of robot
  {
    leftGap = findLeftGap (msg); // find the gap between the ob and the left wall
    rightGap = findRightGap (msg); // find the gap between the ob and the right wall
    
    if (leftGap >= rightGap && leftGap > ROBOT_BODY_WIDTH)
    {
      moveToLeft(msg, true); // ROS_INFO ("\nMove to Left");
    }
    else if ( leftGap < rightGap && rightGap > ROBOT_BODY_WIDTH)
    {
      moveToRight(msg, true);
    }
  }
  else if (hasRiO && hasLeO)
  {
    keepBalance(msg);
  }
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
  
  for (int i= 180; i <=363; i++)
  {
    rw = ms -> ranges[i];
    lw = ms -> ranges[725-i];
    if (isnan(rw))
    {  
      msgPub.linear.x = 0.3;
      msgPub.angular.z = -0.5; // -0.3 then - 0.5
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
      if ( abs(lw - rw) < 0.1) 
      //robot is quite at the center of the road, no need to turn
          break; 
    }
  }
}

void turnAround (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  msgPub.linear.x = -0.3;
  float lw = ms -> ranges [636];
  float rw = ms -> ranges [180];
  if ( lw > rw)
     msgPub.angular.z = 0.5;
  else
     msgPub.angular.z = -0.5;
  ROS_INFO ("\nTurning Around");
  pubGlobal.publish(msgPub);
}

bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float rw;
  for (int i = 340; i > 225 ; i--)
  {
    rw = ms -> ranges[i];
    if (rw < OBSTACLE_DIST )
      return true;
  }
  return false;
}
bool checkLeftObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float lw;
  for (int i= 390; i < 500; i++)
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
    rw = ms -> ranges[225];//WAS 240 - update rw to stop turning at 45 degree
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
      lw = ms -> ranges[500];//update rw to stop turning at 135 degree
      if (lw > 1.8 || isnan(lw))
        keepBalance(ms);
    }
}

float findRightGap (const sensor_msgs::LaserScan::ConstPtr& ms)
{
  float currentLaser, nextLaser, rightGap=0;
  float robot_ob_diff; //difference between right side of the robot and right border of the obstacle
  float maxRight = ms -> ranges[90];
  float middle = ms -> ranges[363];
  int i;
  for ( i=360;i>=180;i--)
  {
    currentLaser = ms -> ranges[i];
    nextLaser = ms -> ranges[i-1];
    if ((abs(currentLaser-nextLaser)/currentLaser)> 0.2)
    {
      if(currentLaser > middle)
      {
        robot_ob_diff = sqrt(pow(currentLaser,2)-pow(middle,2));
        if (maxRight > robot_ob_diff)
          rightGap = maxRight - robot_ob_diff;
      break;
      }
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
  for (j=365;j <= 540;j++)
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



