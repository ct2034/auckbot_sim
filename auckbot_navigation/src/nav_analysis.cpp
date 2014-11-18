/*************************************************
nav_analysis.cpp                          
-------------------
                                              
Copyright 2014 Christian Henkel
Contact: post@henkelchristian.de 
   
This file is part of Auckbot.

Auckbot is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 2 of the License, or (at your option) any later version.

Auckbot is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Auckbot. If not, see http://www.gnu.org/licenses/.             
***************************************************/
 
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "move_base_msgs/MoveBaseActionResult.h"
//#include "move_base_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

void resultCallback(const move_base_msgs::MoveBaseActionResult msg)
{
  ROS_INFO("I heard a result message\n");
}


//void statusCallback(const move_base_msgs::GoalStatusArray msg)
//{
//  ROS_INFO("I heard a status message\n");
//}


void goalCallback(const move_base_msgs::MoveBaseActionGoal msg)
{
  ROS_INFO("I heard a goal message\n");
}

int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");
	ros::init(argc, argv, "nav_analysis");
	ros::NodeHandle nh;
	ros::Subscriber resultSub = nh.subscribe("/move_base/result", 1000, resultCallback);
 	//ros::Subscriber statusSub = nh.subscribe("/move_base/status", 1000, statusCallback);
 	ros::Subscriber goalSub = nh.subscribe("/move_base/goal", 1000, goalCallback);
  
  ros::spin();

	ROS_INFO("STOPPING NODE");
  return 0;
}
