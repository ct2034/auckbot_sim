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
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "move_base_msgs/MoveBaseActionResult.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

#include <tf/transform_listener.h>

#define TIME 1
#define THD 0.001

class NavAnalysis 
{
  private:
    float length;
    ros::Time timeLast;
    geometry_msgs::Vector3 oldPoint;
 
  public:
  //constructors
    NavAnalysis();
  //setter/getter
    void addLength(float in);
    float getLength();
    void resetLength();
    void addPoint(tf::StampedTransform transform);
    void setOldPoint(tf::Vector3& point);
  //callbacks  
    void resultCallback(const move_base_msgs::MoveBaseActionResult msg);
    void goalCallback(const move_base_msgs::MoveBaseActionGoal msg);
};

// class functions
//constructors
NavAnalysis::NavAnalysis()
{
  length = 0;
  oldPoint.x = 0;
  oldPoint.y = 0;
  oldPoint.z = 0;
}

//setter/getter
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

void NavAnalysis::addPoint(tf::StampedTransform transform)
{
  float dist = sqrt( pow(transform.getOrigin().x() - oldPoint.x, 2) +
                     pow(transform.getOrigin().y() - oldPoint.y, 2) );
  if (dist>THD) 
  {
    ROS_INFO("New Point: %f, %f, d: %f", transform.getOrigin().x(), transform.getOrigin().y(), dist);
    this->addLength(dist);
    this->setOldPoint(transform.getOrigin());
  }
}

void NavAnalysis::setOldPoint(tf::Vector3& point)
{
  oldPoint.x = point.x();
  oldPoint.y = point.y();
  oldPoint.z = point.z();
}

// callbacks
void NavAnalysis::resultCallback(const move_base_msgs::MoveBaseActionResult msg)
{
  ROS_INFO("Reached goal. Length was: %f", this->getLength());
}

void NavAnalysis::goalCallback(const move_base_msgs::MoveBaseActionGoal msg)
{
  ROS_INFO("I heard a goal message");
  this->resetLength();
}

int main(int argc, char** argv)
{
  char* map_frame = "/map";
  char* robot_frame = "/base_link";
  // TODO: get frames from params 


	ROS_INFO("Starting node...");
	ros::init(argc, argv, "nav_analysis");
  ros::NodeHandle nh;
  ros::Rate rate(TIME);
  NavAnalysis na = NavAnalysis();
	
	ros::Subscriber resultSub = nh.subscribe("/move_base/result", \
	  1000, &NavAnalysis::resultCallback, &na);
 	ros::Subscriber goalSub = nh.subscribe("/move_base/goal", \
 	  1000, &NavAnalysis::goalCallback, &na);
  tf::TransformListener listener;
  
  ROS_INFO("Waiting for transformation from '%s' to '%s'", map_frame, robot_frame);
  listener.waitForTransform(map_frame, robot_frame, ros::Time::now(), ros::Duration(60.0));
  
  while (nh.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_INFO("%s",ex.what());
    }
    
    na.addPoint(transform);
    
    ros::spinOnce();
    rate.sleep();
  }

	ROS_INFO("STOPPING NODE");
  return 0;
}
