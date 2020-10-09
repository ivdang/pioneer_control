//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Project: Obstacle Avoindace for Pioneer robot 										//				
// Written by: Vi Dang 														//							//
// Project Description: This project was written during Summer Internship 							//	
//			at Dr. Holly Yanco's Robotics lab (Umass Lowell University)						//			
//			This project helps pioneer robot avoid obstacles around it when it's moving.				//
//			It is NOT a project that leads the robot from one point to another point. 				//	
//			It is not guarantee the robot will follow one path only							//			
//			But it helps reduce the chance robot hits to obstacles on the way					//		
// File name: robot_control.cpp													//						//
// File description: this is the source file that define all operations of the robot to obstacles 				//
// Other files in this project: robot_control.h 										//				
// Challenges: - some calculation just apply for right triangle only. It will not be reliable for other cases.			//										//								//
//	       - converting index of ranges array to match with the angle in trig easy to be confused				//
//	       - Robot sees nan-value(or infinive value),these values can't be used to calculate things				//
//																//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "math.h"
#include "robot_control.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float m = msg ->ranges [INDEX_AT_90_DEGREE];
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
    ROS_INFO("\nMiddle: [%.2f]: \n-----------------",m);
}

void doNormalOperation(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float rw, lw, m, rightGap, leftGap;
    bool hasRiO = false, hasLeO = false;
   
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
        else if ( rightGap > leftGap && rightGap > ROBOT_BODY_WIDTH)
        {
            // to avoid an obstacle in front of robot, it must turn enough rads -> passing true
              moveToRight(msg, true);
        }
      }
      
    else if (hasRiO && hasLeO)
    // this case means there obstacles on both sides but there is nothing in front of the robot,
    // so it just need to keep balance to get through
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
      
	// the robot will check on the left and the right to keep balance by looking
	// two supplementary angles at the same time, so it can modify its velocity.
	// It starts looking from 30 to 90 degree on the right, from 150 back to 90 degree on the left
	for (int i = INDEX_AT_30_DEGREE; i <= INDEX_AT_90_DEGREE; i++)
	{
	  	//rw is the distance robot can see at i degree
	  	//lw is the distance robbot can see at (180 - i) degree
        rw = ms -> ranges[i];
        lw = ms -> ranges[INDEX_AT_240_DEGREE - i];

        //if robot sees nan value, it means there is a lot of space in that direction,
        //so I let the robot go to that direction
        if (isnan(rw))
        {  
          msgPub.linear.x = 0.3;
          msgPub.angular.z = -0.4; // -0.3
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
        //robot has to modify its angular velocity when it is not at the center of the path
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
	// when robot is too close to something, 
	// it has to choose one way to turn,
	// so I let it use the distance at 0 degree and 180 degree to make a decision 
	msgPub.linear.x = -0.3;
	float lw = ms -> ranges [INDEX_AT_180_DEGREE];
	float rw = ms -> ranges [INDEX_AT_0_DEGREE];
	if (lw > rw)
		msgPub.angular.z = 0.5;
	else
		msgPub.angular.z = -0.5;
	pubGlobal.publish(msgPub);
}
//check to see is there any obstacle on the right of robot (from 80 back to 45 degree)
bool checkRightObstacle (const sensor_msgs::LaserScan::ConstPtr& ms)
{
	float rw;
	for (int i = INDEX_AT_80_DEGREE; i > INDEX_AT_45_DEGREE ; i--)
	{
		rw = ms -> ranges[i];
		if (rw < OBSTACLE_DIST)
	    	return true;
	}		
	return false;
}
//check to see is there any obstacle on the left of robot (from 100 to 135 degree)
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
	// this operation is executed when robot see obstacle in front of it.
	// in this case, it's better to turn enough to avoid the obstacle	
	{
		msgPub.linear.x = 0.1;///was 0.2
		msgPub.angular.z = 0.3; //0.4
		pubGlobal.publish(msgPub);
		isMustTurnOperation = true;
		currentTurningAngular = 0.3;
		 //I used 7 as a fixed number for mustTurnRads to make it simple for now.
        //(it should be calculated depends on how big the obstacle is)		
		mustTurnToRads = 7; // was 6
	}
	else
	// if the robot see nothing in front of it, but it see obstacle on the right
    // it needs to turn left lightly and keep balance when it see clear at 45 degree
	{  
		msgPub.linear.x = 0.3;
		msgPub.angular.z = 0.3; //0.4
		pubGlobal.publish(msgPub);
		rw = ms -> ranges[INDEX_AT_45_DEGREE];//WAS 240 - update rw to stop turning at 45 degree
		// clear at the angle 45 on the right, continue balancing
		if (rw > 1.8 || isnan(rw))
		    keepBalance(ms);
	}
}

void moveToRight (const sensor_msgs::LaserScan::ConstPtr& ms, bool shouldCheckTurnedRads)
{
    float lw;
    if (shouldCheckTurnedRads)
    // this operation is executed when robot see obstacle in front of it.
	// in this case, it's better to turn enough to avoid the obstacle
    {
        msgPub.linear.x = 0.1; /// was 0.2
        msgPub.angular.z = -0.3;   //0.4
        pubGlobal.publish(msgPub);
        isMustTurnOperation = true;
        currentTurningAngular = 0.3;
        //I used 7 as a fixed number for mustTurnRads to make it simple for now.
        //(it should be calculated depends on how big the obstacle is)
        mustTurnToRads = 7; // was 6
    }
    else
    // if the robot see nothing in front of it, but it see obstacle on the left
    // it needs to turn right lightly and keep balance when it see clear at 135 degree
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
    //robot_ob_diff is the difference from the point that 90-degree-laser points to on the obstacle and right side of the obstacle
    float robot_ob_diff; 
    float maxRight = ms -> ranges[INDEX_AT_0_DEGREE];
    float middle = ms -> ranges[INDEX_AT_90_DEGREE];
    int i;
    for ( i = INDEX_AT_90_DEGREE; i >= INDEX_AT_30_DEGREE;i--)
    {
        currentLaser = ms -> ranges[i];
        nextLaser = ms -> ranges[i-1];
        if ((abs(currentLaser-nextLaser)/currentLaser)> 0.15|| (!isnan(currentLaser) && isnan(nextLaser)))
        // if the difference is bigger than 15% or currentLaser is not a nan-value but nextLaser is a nan-value
        // it means 'i' is the position of the side of the obstacle at which the value of laser changes suddenly
        {
            //if (currentLaser > middle)
            //if(currentLaser > middle) 
			//--> this if to make sure I will get approriate number for the special case with right triangle
			// it helps eliminate some situations that robot_ob_diff is a square root of a negative number  
			{
				// I apply Pythagorean Theorem for right triangle with currentLaser as a hypotenuse, and middle is one leg
				// this formula's just true when the obstacle's right in front of the robot, making a right triangle from 90-degree laser
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
	//robot_ob_diff is the difference from the point that 90-degree-laser points to on the obstacle and left side of the obstacle
	float robot_ob_diff;
	float maxLeft = ms -> ranges[INDEX_AT_180_DEGREE];
	float middle = ms -> ranges[INDEX_AT_90_DEGREE];
	int j;
      
    for (j= INDEX_AT_100_DEGREE;j <= INDEX_AT_150_DEGREE;j++)
    {
        currentLaser = ms -> ranges[j];
        nextLaser = ms -> ranges[j+1];
        if ((abs(nextLaser-currentLaser)/currentLaser) > 0.15 || (!isnan(currentLaser) && isnan(nextLaser)))
        // if the difference is bigger than 15% or currentLaser is not a nan-value but nextLaser is a nan-value
        // it means 'j' is the position of the side of the obstacle at which the value of laser changes suddenly
        {        
			//if(currentLaser > middle) 
			//--> this if to make sure I will get approriate number for the special case with right triangle
			// it helps eliminate some situations that robot_ob_diff is a square root of a negative number 
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

