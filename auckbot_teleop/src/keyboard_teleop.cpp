/* ************************************************
   * keyboard_teleop.cpp                          *
   *                                              *
   *  by:                                         *
   *   Christian Henkel                           *
   *   post@henkelchristian.de                    *
   ************************************************/
   
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#define NO_INPUT 0 // TODO: find out actual values
#define SHIFT_Q  1
#define SHIFT_W  2
#define SHIFT_E  3
#define SHIFT_A  4
#define SHIFT_S  5
#define SHIFT_D  6
#define SHIFT    7

#define LOOP_RATE 100
#define ZERO_THRD .1


bool stop();
bool set_speed(float, float, float);
float stop_traj(float grad, float prev);
float acc_traj(float grad, float prev, float goal);

ros::Publisher speedPub;
  
int main(int argc, char** argv)
{
	ROS_INFO("Starting node...");
	ros::init(argc, argv, "keyboard_teleop");
	//ros::Publisher speedPub;
	ros::NodeHandle nh;
	speedPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Rate loop_rate(LOOP_RATE);
	
	char input = NO_INPUT; 
	
	float max_sp_lin = 1; // TODO read from launchfile
	float max_sp_rot = 1; // TODO read from launchfile
	float grad = .4; // TODO read from launchfile
	
	float sp_x = 0;
	float sp_y = 0;
	float sp_th = 0;

  while(1) // TODO: checkh weather roscore running
  {
    input = 0; //TODO Check for input
      
    switch(input){
      case (NO_INPUT):
        sp_x = 0;
        sp_y = 0;
        sp_th = 0;
        stop();
      break;
      case (SHIFT_Q):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = acc_traj(grad, sp_th, max_sp_rot);
      break;
      case (SHIFT_W):
        sp_x  = acc_traj(grad, sp_x, max_sp_lin);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_E):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = acc_traj(grad, sp_th, -1*max_sp_rot);
      break;
      case (SHIFT_A):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = acc_traj(grad, sp_y, -1*max_sp_lin);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_S):
        sp_x  = acc_traj(grad, sp_x, -1*max_sp_lin);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT_D):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = acc_traj(grad, sp_y, max_sp_lin);
        sp_th = stop_traj(grad, sp_th);
      break;
      case (SHIFT):
        sp_x  = stop_traj(grad, sp_x);
        sp_y  = stop_traj(grad, sp_y);
        sp_th = stop_traj(grad, sp_th);
      break;
    }
    
    set_speed(sp_x, sp_y, sp_th);
  
    loop_rate.sleep();
    ros::spinOnce();
  }
  
  stop();

  return 0;
}

bool stop(){
  geometry_msgs::Vector3Stamped cmd_robot;
  geometry_msgs::Twist msg;
  
  cmd_robot.vector.x = 0.0;
  cmd_robot.vector.y = 0.0;
  msg.angular.z = 0.0;
  
  msg.linear = cmd_robot.vector;
  msg.linear.z = 0.0; msg.angular.x = 0.0; msg.angular.y = 0.0;
  
  speedPub.publish(msg);
  
  return true;
}

bool set_speed(float _x, float _y, float _th){
  geometry_msgs::Vector3Stamped cmd_robot;
  geometry_msgs::Twist msg;
  
  cmd_robot.vector.x = _x;
  cmd_robot.vector.y = _y;
  msg.angular.z = _th;
  
  msg.linear = cmd_robot.vector;
  msg.linear.z = 0.0; msg.angular.x = 0.0; msg.angular.y = 0.0;
  
  speedPub.publish(msg);
  
  return true;
}

float stop_traj(float grad, float prev){
  if( prev < ZERO_THRD & prev > -ZERO_THRD ) return 0;
  else return prev * grad;  
}

float acc_traj(float grad, float prev, float goal){
  float diff = goal-prev;
  if( diff < ZERO_THRD & diff > -ZERO_THRD ) return goal;
  else return prev + diff * grad;  
}
  


