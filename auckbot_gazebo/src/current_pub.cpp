#include "ros/ros.h"
#include "auckbot_gazebo/MotorCurrents.h"
#include "geometry_msgs/Twist.h"

class CurrentPub 
{
  private:
    float speed;
    ros::Publisher publisher;
 
  public:
    CurrentPub();
    void velocityCallback(const geometry_msgs::Twist& msg);
    void setPub(ros::Publisher publisher);
  
};

// class functions
CurrentPub::CurrentPub()
{
  speed = 0;
}
  
void CurrentPub::velocityCallback(const geometry_msgs::Twist& speedMsg)
{
  ROS_INFO("I heard x-speed = %f", speedMsg.linear.x);
  
  auckbot_gazebo::MotorCurrents msg;
  msg.current1 = 0.0;
  msg.current2 = 0.0;
  msg.current3 = 0.0;
  msg.current4 = 0.0;
  
  publisher.publish(msg);
}

void CurrentPub::setPub(ros::Publisher publisher_)
{
  publisher = publisher_;
}

// MAIN
int main(int argc, char **argv)
{
  ros::init(argc, argv, "current_pub");
  ros::NodeHandle n;
  
  CurrentPub cb = CurrentPub();
  cb.setPub( n.advertise<auckbot_gazebo::MotorCurrents>("current", 1000) );
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, &CurrentPub::velocityCallback, &cb);
  
  ros::spin();

  return 0;
}

