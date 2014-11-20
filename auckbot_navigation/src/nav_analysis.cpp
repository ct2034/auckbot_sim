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
 
#include <string>

#include <ros/ros.h>  

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <tf/transform_listener.h>

#define TIME 1
#define THD 0.0001

class Metric {
  private:
    char name[10];
    char description[120];
    float value;
    
  public:
    Metric() {}
    Metric(char* _name, char* _description);
    void toString(char*);
    float getValue(){ return value; }
    void resetValue(){ value = 0; }
    void addValue(float _val){ value += _val; }
};

Metric::Metric(char* _name, char* _description) {
  strcpy(name, _name);
  strcpy(description, _description);
  resetValue();
}
    
void Metric::toString(char* message) {
  sprintf(message, "Metric '%s': >%s<.\nValue: %f", name, \
    description, value);
  return;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class metricListener {
  private:
    ros::Time timeLast;
    geometry_msgs::Vector3 oldPoint;
    Metric lengthMetric;
 
  public:
  //constructors
    metricListener();
  //setter / getter
    void addPoint(tf::StampedTransform transform);
    void setOldPoint(tf::Vector3& point);
  //callbacks  
    void resultCallback(const move_base_msgs::MoveBaseActionResult msg);
    void goalCallback(const move_base_msgs::MoveBaseActionGoal msg);
};

// class functions
//constructors
metricListener::metricListener() {
  lengthMetric = Metric((char*) "Length", (char*) "The total length of the route");
  oldPoint.x = 0;
  oldPoint.y = 0;
  oldPoint.z = 0;
}

void metricListener::addPoint(tf::StampedTransform transform) {
  float dist = sqrt( pow(transform.getOrigin().x() - oldPoint.x, 2) +
                     pow(transform.getOrigin().y() - oldPoint.y, 2) );
  if (dist>THD) 
  {
    //ROS_INFO("New point: %f, %f, d: %f", \
      transform.getOrigin().x(), transform.getOrigin().y(), dist);
    lengthMetric.addValue(dist);
    this->setOldPoint(transform.getOrigin());
  }
}

void metricListener::setOldPoint(tf::Vector3& point) {
  oldPoint.x = point.x();
  oldPoint.y = point.y();
  oldPoint.z = point.z();
}
// callbacks
void metricListener::resultCallback(const move_base_msgs::MoveBaseActionResult msg) {
  char stringInfo[100];
  lengthMetric.toString(stringInfo);
  ROS_INFO(stringInfo);
}

void metricListener::goalCallback(const move_base_msgs::MoveBaseActionGoal msg) {
  ROS_INFO("New route ..");
  lengthMetric.resetValue();
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv) {
  char* map_frame ((char*) "/map");
  char* robot_frame ((char*) "/base_link");
  // TODO: get frames from params 

	ROS_INFO("Starting node...");
	ros::init(argc, argv, "nav_analysis");
  ros::NodeHandle nh;
  ros::Rate rate(TIME);
	metricListener ml = metricListener();

	ros::Subscriber resultSub = nh.subscribe("/move_base/result", \
	  1000, &metricListener::resultCallback, &ml);
 	ros::Subscriber goalSub = nh.subscribe("/move_base/goal", \
 	  1000, &metricListener::goalCallback, &ml);
  tf::TransformListener listener;
  
  ROS_INFO("Waiting for transformation from '%s' to '%s'", \
    map_frame, robot_frame);
  listener.waitForTransform(map_frame, robot_frame, \
    ros::Time::now(), ros::Duration(60.0));
  
  while (nh.ok()){
    tf::StampedTransform transform;
    try {
      listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_INFO("%s",ex.what());
    }
    
    ml.addPoint(transform);
    
    ros::spinOnce();
    rate.sleep();
  }

	ROS_INFO("STOPPING NODE");
  return 0;
}