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
// Numer of metrics float
#define NO_MET_F  4
// Numer of metrics string
#define NO_MET_S  1
// String length for name
#define STRL_NAME 20
// String length for description
#define STRL_DESC 200

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
    char name[STRL_NAME];
    char description[STRL_DESC];
    float value;
    char sValue[STRL_DESC];
    ros::Time timeLast;
    
  public:
    Metric() {}
    Metric(char* _name, char* _description);
    void toString(char*);
    void getName(char* _nam){ strcpy(_nam, name); }
    void getValue(float* _val){ *_val = value; }
    void getValue(char* _val){ strcpy(_val, sValue); }
    void setSValue(char* _val){ strcpy(sValue, _val); }
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
  if(strcmp(sValue, "N/A") == 0 & value >= 0.0) { // this is a float Metric
    sprintf(message, "Metric '%s': >%s<.\nValue: %f", name, \
      description, value);
  }
  else if (strcmp(sValue, "N/A") != 0 & value == 0.0) { // this is a String Metric
    sprintf(message, "Metric '%s': >%s<.\nValue: %s", name, \
      description, sValue);
  } else {
    ROS_ERROR("String and value set for Metric '%s'", name);
  }
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
    Metric metrics[NO_MET_F+NO_MET_S];
    DBClientConnection* c;
    BSONObjBuilder* b;
    bool builderCreated;
    bool debug;
    mongo::Date_t startTime;
   
  public:
  //constructors
    metricListener();
    metricListener(bool debug);
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
  //helper
    mongo::Date_t convertTime(const boost::posix_time::ptime& time);
};

// class functions
//constructors
metricListener::metricListener() {
}

metricListener::metricListener(bool _debug) {
  metrics[0] = Metric((char*) "Length [m]", \
    (char*) "The total length of the route");
  metrics[1] = Metric((char*) "Rotation [rad]", \
    (char*) "The total rotation along the route");
  metrics[2] = Metric((char*) "Current [As]", \
    (char*) "Consumed motor current");
  metrics[3] = Metric((char*) "Duration [ms]", \
    (char*) "Total time used");
  metrics[4] = Metric((char*) "Planner Setup", \
    (char*) "Values that are set for setup of move_base");
  oldPose = tf::Transform();

  try{
    c = new DBClientConnection();
    c->connect(MONGO_HOST);

  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }

  builderCreated = false;
  debug = _debug;
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
    //  transform.getOrigin().x(), transform.getOrigin().y(), \
    //  getYaw(transform.getRotation()), dist, rot);
    metrics[0].addValue(dist);
    metrics[1].addValue(rot);
    this->setOldPose(transform);
  }
}

// callbacks
void metricListener::resultCallback(const move_base_msgs::MoveBaseActionResult msg) {
  // ROS_INFO("1");
  float check;
  metrics[0].getValue(&check); // evaluate to filter out to short trips

  // capture duration
  metrics[3].addValue(startTime - convertTime(ros::Time::now().toBoost()));

  // ROS_INFO("2");
  if(!debug & check>0.0) { // save to DB
    char name[STRL_NAME];
    char des[STRL_DESC];
    float val;
    // ROS_INFO("3");
    for(int i=0; i<NO_MET_F; i++) {
      // ROS_INFO("3.1");
      metrics[i].getName(name);
      metrics[i].getValue(&val);
      // ROS_INFO("3.2");   
      saveToDB((char *) name, val);  
      // ROS_INFO("3.3");    
      // saveToDB((char *) "name", 0.1); 
    }
    // ROS_INFO("4");
    for(int i=NO_MET_F; i<NO_MET_F+NO_MET_S; i++) {
      metrics[i].getName(name);
      metrics[i].getValue(des);
      saveToDB((char *) name, (char *) des);
      // saveToDB((char *) "name", (char *) "des");      
    }
    // ROS_INFO("5");
    finalize(); 
  } // terminal only

  // ROS_INFO("6");
  if(check == 0.0) ROS_INFO("(This is not saved in the DB ..)");
  
  char stringInfo[200];
  metrics[0].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[1].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[2].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[3].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[4].toString(stringInfo);
  ROS_INFO("%s", stringInfo);
  metrics[4].~Metric();
  metrics[4] = Metric((char*) "Planner Setup", \
    (char*) "Values that are set for setup of move_base");

  ROS_INFO("7");
}

void metricListener::goalCallback(const move_base_msgs::MoveBaseActionGoal msg) {
  ROS_INFO("New route ..");
  metrics[0].resetValueAndTime();
  metrics[1].resetValueAndTime();
  metrics[2].resetValueAndTime();
  char params[100];
  sprintf( params, "MB_USE_GRID_PATH: %s \nMB_USE_GRID_PATH: %s \n", \
    getenv ("MB_BASE_GLOBAL_PLANNER"), getenv ("MB_USE_GRID_PATH"));
  metrics[4].setSValue(params);
  startTime = convertTime(ros::Time::now().toBoost());
}

void metricListener::currentsCallback(const auckbot_gazebo::MotorCurrents msg) {
  float currents = msg.current1 + msg.current2 + msg.current3 + msg.current4;
  metrics[2].addValue( currents * metrics[2].passedTime(msg.time) );
}

void metricListener::saveToDB(char* name, char* value) {
  if (!builderCreated) { 
    b = new BSONObjBuilder();
    b->appendDate("start_time", startTime); 
    builderCreated = true;
  }
  b->append(name, value);
}

void metricListener::saveToDB(char* name, float value) {
  if (!builderCreated) { 
    b = new BSONObjBuilder();
    b->appendDate("start_time", startTime); 
    builderCreated = true;
  }
  b->append(name, value);
}

void metricListener::finalize(void) {
  try{
    BSONObj p = b->obj();
    c->insert(MONGO_COL, p);

    b->~BSONObjBuilder();
    builderCreated = false;
  } catch( const mongo::DBException &e ) {
    ROS_ERROR("caught %s", e.what());
  }
}

// converting a boost time into a mongo time
mongo::Date_t metricListener::convertTime(const boost::posix_time::ptime& time) {
  std::tm pt_tm = boost::posix_time::to_tm(time);
  std::time_t t = mktime(&pt_tm);  
  mongo::Date_t d(t);
  return d;
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char** argv) { 
  ROS_INFO("Starting node...");
  ros::init(argc, argv, "nav_analysis");
  ros::NodeHandle nh;
  ros::Rate rate(TIME);
   
  std::string map_frame;
  std::string robot_frame;
  bool debug;
  if(!ros::param::get("~map_frame", map_frame)) \
    ROS_ERROR("Can not get param map_frame");
  if(!ros::param::get("~robot_frame", robot_frame)) \
    ROS_ERROR("Can not get param robot_frame");
  if(!ros::param::get("~debug", debug)) \
    ROS_ERROR("Can not get param debug");

  metricListener ml = metricListener(debug);
  
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