#include <iostream>
#include <stdlib.h>
#include <array>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>

using namespace std;

#ifndef TESTPIONEER_H_
#define TESTPIONEER_H_

int flag = 0;
int arraySize = 0;
float reqPosition = 5;

class TestPioneer
{
 public:
  void controlDrive();
  void initialize();
  geometry_msgs::Twist vel;
  
 private:
  void laserCall(const sensor_msgs::LaserScan::ConstPtr& laser);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber laser_sub_;
};

void TestPioneer::initialize()
{
  vel_pub_ = n_.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TestPioneer::laserCall, this);
  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/joy", 10, &TestPioneer::joyCallback, this);
}

void TestPioneer::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if((joy->buttons[8] == 1) && (joy->buttons[9] == 1))
    {
      flag = 0;
      laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TestPioneer::laserCall, this);
    }
}

void laserCall(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  if(flag == 0)
    {  
      arraySize = sizeof(laser->ranges[1])/sizeof(laser->ranges[0]);
      float pi = 3.1415;
      int sideLimits = 2;
      float angleArray[arraySize];
      float yDistance[arraySize];
      float leftSideDistance = 0;
      float rightSideDistance = 0;
      int midPoint = arraySize/2;
      int arrayPosition = 0;
      float linearError = reqPosition - laser-ranges[midPoint];
      
      float kp_linear_ = 0.6;
      float kp_angular_ = 0.2;
      
      float incrementAngle = laser->angle_increment;
      float minAngle = laser->angle_min;
      float maxAngle = laser->angle_max;
      
      for(arrayPosition = 1; arrayPosition < arraySize ; arrayPosition++)
	{
	  angleArray[arrayPosition] = (minAngle + (arrayPosition*incrementAngle))*(pi/180);
	  yDistance[arrarPosition] = cos(angleArray[arrayPosition]);
	  if(yDistance[arrayPosition] >= sideLimits)
	    {
	      yDistance[arrayPosition] = sidelimits;
	    }
	  while(arrayPosition <= midPoint)
	    {
	      if(angleArray[arrayPosition] < angleArray[arrayPosition - 1])
		rigthSideDistance = angleArray[arrayPosition];
	    }
	  while(arrayPosition > midPoint)
	    {
	      if(angleArray[arrayPosition - 1] > angleArray[arrayPosition])
		leftSideDistance = angleArray[arrayPosition - 1];
	    }
	}
      
      float angularError = leftSideDistance - rightSideDistance;
      
      if(linearError > 0)
	{
	  vel.linear.x = kp_linear_*linearError;
	  if(angularError > 0)
	    {
	      vel.angular.z = kp_angular_*angularError;
	    }
	  else if(angularError < 0)
	    {
	      vel.angular.z = kp_angular_*angularError;
	    }
	  vel_pub_.publish(vel);
	}
      else if(linearError < 0)
	{
	  vel.linear.x = kp_linear_*linearError;
	  if(angularError > 0)
	    {
	      vel.angular.z = kp_angular_*angularError;
	    }
	  else if(angularError < 0)
	    {
	      vel.angular.z = kp_angular_*angularError;
	    }
	  vel_pub_.publish(vel);
	}
      else
	{
	  vel.linear.x = 0;
	  vel.angular.z = 0;
	  flag = 1;
	  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/joy", 10, &TestPioneer::joyCallback, this);
	}
      vel_pub_.publish(vel);
    }
  else
    {
      vel.linear.x = 0;
      vel.angular.z = 0;
      flag = 1;
      joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/joy", 10, &TestPioneer::joyCallback, this);
      vel_pub_.publish(vel);
    }
  vel_pub_.publish(vel);
}

#endif /* testPioneer.h */
