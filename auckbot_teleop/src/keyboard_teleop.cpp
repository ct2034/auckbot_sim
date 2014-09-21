/* ************************************************
   * keyboard_teleop.cpp                          *
   *                                              *
   *  by:                                         *
   *   Christian Henkel                           *
   *   post@henkelchristian.de                    *
   ************************************************/
   
#include <ros/ros.h>   
  
int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");

	ros::init(argc, argv, "keyboard_teleop");
	
	ROS_INFO("INIT");
	ros::spin();

  return 0;
}

