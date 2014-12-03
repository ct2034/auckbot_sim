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
#include <exception>

#include <ros/ros.h> 
#include <ros/time.h>  

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose.h>

#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <auckbot_gazebo/MotorCurrents.h>

#include <tf/transform_listener.h>

#define TIME      1
#define THD       0.0001
#define PI        3.14159265359
#define NSECPSEC  1 000 000 000

//MongoDB ----
#define MONGO_HOST  "localhost"
//#define MONGO_DB    "nav_analysis"
#define MONGO_COL   "nav_analysis.trips"

#include <cstdlib>
#include <iostream>
#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"

using namespace mongo;
//-------------

class Metric {
  private:
    char name[20];
    char description[120];
    float value;
    char sValue[200];
    ros::Time timeLast;
    
  public:
    Metric() {}
    Metric(char* _name, char* _description);
    void toString(char*);
    void getValue(float* _val){ *_val = value; }
    void getValue(char* _val){ strcpy(sValue, _val); }
    void setSValue(char* _val){ strcpy(_val, sValue); }
    void resetValueAndTime();
    void addValue(float _val){ value += _val; }
    float passedTime(ros::Time now);
};

Metric::Metric(char* _name, char* _description) {
  strcpy(name, _name);
  strcpy(description, _description);
  resetValueAndTime();
}

void Metric::resetValueAndTime(){
  value = 0.0;
  strcpy(sValue, "N/A");
  timeLast = ros::Time::now();
}

    
void Metric::toString(char* message) {
  sprintf(message, "Metric '%s': >%s<.\nValue: %f", name, \
    description, value);
  return;
}

// Returns the time that passed since this was called the last time. (in seconds)
float Metric::passedTime(ros::Time now) {
  float sec = now.toSec() - timeLast.toSec();
  timeLast = ros::Time( now );
  return sec;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class metricListener {
  private:
    tf::Transform oldPose;
    Metric metrics[4];
    DBClientConnection* c;
    BSONObjBuilder* b;
    bool builderCreated;
 
  public:
  //constructors
    metricListener();
  //setter / getter
    void addPoint(tf::StampedTransform transform);
    void setOldPose(tf::StampedTransform transform){ \
      oldPose = tf::StampedTransform( transform ); }
  //callbacks  
    void resultCallback(const move_base_msgs::MoveBaseActionResult msg);
    void goalCallback(const move_base_msgs::MoveBaseActionGoal msg);
    void currentsCallback(const auckbot_gazebo::MotorCurrents msg);
  //DB access
    void saveToDB(char* name, char* value);
    void saveToDB(char* name, float value);
    void finalize(void);
};

// class functions
//constructors
metricListener::metricListener() {
  metrics[0] = Metric((char*) "Length [m]", \
    (char*) "The total length of the route");
  metrics[1] = Metric((char*) "Rotation [rad]", \
    (char*) "The total rotation along the route");
  metrics[2] = Metric((char*) "Current [As]", \
    (char*) "Consumed motor current");
  oldPose = tf::Transform();

  try{
    c = new DBClientConnection();
    c->connect(MONGO_HOST);

  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }

  builderCreated = false;
}

void metricListener::addPoint(tf::StampedTransform transform) {
  float dist = sqrt( pow(transform.getOrigin().x() - oldPose.getOrigin().x(), 2) +
                     pow(transform.getOrigin().y() - oldPose.getOrigin().y(), 2) );
  double roll, pitch, yaw;
  float rot = fabs( getYaw(transform.getRotation()) - getYaw(oldPose.getRotation()) );
  if (rot > PI) rot -= 2 * PI;
  rot = fabs(rot);

  if (dist>THD) 
  {
    //ROS_INFO("New point: %f, %f, %f\n d: %f, r: %f", \
      transform.getOrigin().x(), transform.getOrigin().y(), \
      getYaw(transform.getRotation()), dist, rot);
    metrics[0].addValue(dist);
    metrics[1].addValue(rot);
    this->setOldPose(transform);
  }
}

// callbacks
void metricListener::resultCallback(const move_base_msgs::MoveBaseActionResult msg) {
  char stringInfo[200];
  metrics[0].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[1].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[2].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[3].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[3].~Metric();
}

void metricListener::goalCallback(const move_base_msgs::MoveBaseActionGoal msg) {
  ROS_INFO("New route ..");
  metrics[0].resetValueAndTime();
  metrics[1].resetValueAndTime();
  metrics[2].resetValueAndTime();
  char params[100];
  sprintf( params, "MB_USE_GRID_PATH: %s\nMB_USE_GRID_PATH: %s\n", \
    getenv ("MB_BASE_GLOBAL_PLANNER"), getenv ("MB_USE_GRID_PATH"));
  metrics[3] = Metric((char*) "Planner Parameters", params);
}

void metricListener::currentsCallback(const auckbot_gazebo::MotorCurrents msg) {
  float currents = msg.current1 + msg.current2 + msg.current3 + msg.current4;
  metrics[2].addValue( currents * metrics[2].passedTime(msg.time) );
}

void metricListener::saveToDB(char* name, char* value) {
  if (!builderCreated) { 
    b = new BSONObjBuilder();
    //b->append("start_time", DATENOW); 
    builderCreated = true;
  }
  b->append(name, value);
}

void metricListener::saveToDB(char* name, float value) {
  if (!builderCreated) { 
    b = new BSONObjBuilder();
    //b->append("start_time", DATENOW); 
    builderCreated = true;
  }
  b->append(name, value);
}

void metricListener::finalize(void) {
  try{
    //b->append("goal_time", DATENOW);
    BSONObj p = b->obj();
    c->insert(MONGO_COL, p);

    ROS_INFO("pre: %lu", (long unsigned) b);
    b->~BSONObjBuilder();
    ROS_INFO("after: %lu", (long unsigned) b);

    builderCreated = false;
  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv) { 
  ROS_INFO("Starting node...");
  ros::init(argc, argv, "nav_analysis");
  ros::NodeHandle nh;
  ros::Rate rate(TIME);

  metricListener ml = metricListener();
   
  std::string map_frame;
  std::string robot_frame;
  if(!ros::param::get("~map_frame", map_frame)) ROS_ERROR("Can not get param map_frame");
  if(!ros::param::get("~robot_frame", robot_frame)) ROS_ERROR("Can not get param robot_frame");
  
  ros::Subscriber resultSub = nh.subscribe("/move_base/result", \
	  1000, &metricListener::resultCallback, &ml);
  ros::Subscriber goalSub = nh.subscribe("/move_base/goal", \
    1000, &metricListener::goalCallback, &ml);
  ros::Subscriber currentSub = nh.subscribe("/current", \
    1000, &metricListener::currentsCallback, &ml);
  tf::TransformListener listener;
  
  ROS_INFO("Waiting for transformation from '%s' to '%s'", \
    map_frame.c_str(), robot_frame.c_str());
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