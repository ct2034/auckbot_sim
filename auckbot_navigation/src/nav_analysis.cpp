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
#include "move_base_msgs/MoveBaseActionGoal.h"

#include <tf/transform_listener.h>

#define TIME 1

class NavAnalysis 
{
  private:
    float length;
    ros::Time timeLast;
 
  public:
    NavAnalysis();
    void addLength(float in);
    float getLength();
    void resetLength();
    
  private:
    void setCurrents(float* speeds, float duration);
};

// class functions
NavAnalysis::NavAnalysis()
{
  length = 0;
}

void NavAnalysis::addLength(float in)
{
  length += in;
}

float NavAnalysis::getLength()
{
  return length;
}

void NavAnalysis::resetLength()
{
  length = 0;
}

// callbacks
void resultCallback(const move_base_msgs::MoveBaseActionResult msg)
{
  ROS_INFO("Reached goal. Length was: %f\n", 0.0);
}

void goalCallback(const move_base_msgs::MoveBaseActionGoal msg)
{
  ROS_INFO("I heard a goal message\n");
}

int main(int argc, char** argv)
{
  char* frame1 = "/base_link";
  char* frame2 = "/map";
  // TODO: get frames from params 


	ROS_INFO("Starting node...");
	ros::init(argc, argv, "nav_analysis");
  ros::NodeHandle nh;
  ros::Rate rate(TIME);
	
	ros::Subscriber resultSub = nh.subscribe("/move_base/result", 1000, resultCallback);
 	ros::Subscriber goalSub = nh.subscribe("/move_base/goal", 1000, goalCallback);
  tf::TransformListener listener;
  
  NavAnalysis na = NavAnalysis();
  
  ROS_INFO("Waiting for transformation from '%s' to '%s'", frame1, frame2);
  listener.waitForTransform(frame1, frame2, ros::Time::now(), ros::Duration(60.0));
  
  while (nh.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
    }
    
    na.addLength(0.1);
    
    ros::spinOnce();
    rate.sleep();
  }

	ROS_INFO("STOPPING NODE");
  return 0;
}
