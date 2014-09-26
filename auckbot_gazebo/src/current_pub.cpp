#include "ros/ros.h"
#include "auckbot_gazebo/MotorCurrents.h"
#include "geometry_msgs/Twist.h"

class CurrentPub 
{
  private:
    float speed;
    ros::Publisher publisher;
    ros::Time timeLast;
 
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
  float secDuration;
  ros::Time _now;
  
  _now = ros::Time::now();
  secDuration = _now.toSec() - timeLast.toSec();
    
  ROS_INFO("I heard x-speed = %f, Duration: %f s", speedMsg.linear.x, secDuration);
  
  auckbot_gazebo::MotorCurrents msg;
  msg.time = _now;
  msg.current1 = 0.0;
  msg.current2 = 0.0;
  msg.current3 = 0.0;
  msg.current4 = 0.0;
  
  publisher.publish(msg);
  timeLast = _now;
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

