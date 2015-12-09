#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

    
void callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Speed detected: linear= %lg *** angular= %lg",msg->linear.x,msg->angular.z);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_checker");
  ros::NodeHandle n;
  ros::Subscriber speed_sub = n.subscribe("/cmd_vel",10,callback);
  ros::spin();

  return 0;
}
