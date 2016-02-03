#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include <sstream>

void listener_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	ROS_INFO("child frame id: %s",msg->child_frame_id.c_str());
	ROS_INFO("pose position x: %f, pose position y: %f, pose position z: %f",msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
	ROS_INFO("pose orientation x: %f,pose orientation y: %f, pose orientation z: %f, pose orientation w: %f",msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("POSE COVARIANCE MATRIX");
	for (int i=0; i<msg->pose.covariance.size(); i+=6)
	{
		ROS_INFO("%f %f %f %f %f %f",msg->pose.covariance.elems[i],msg->pose.covariance.elems[i+1],msg->pose.covariance.elems[i+2],msg->pose.covariance.elems[i+3],msg->pose.covariance.elems[i+4],msg->pose.covariance.elems[i+5]);

	}
	//ROS_INFO("Pose Covariance: %f", msg->pose.covariance.)
	ROS_INFO("twist linear x: %f, twist linear y: %f, twist linear z: %f",msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z);
	ROS_INFO("twist angular x: %f,twist angular y: %f,twist angular z: %f",msg->twist.twist.angular.x,msg->twist.twist.angular.y,msg->twist.twist.angular.z);
	ROS_INFO("TWIST COVARIANCE MATRIX");
	for (int i=0; i<msg->twist.covariance.size(); i+=6)
	{
		ROS_INFO("%f %f %f %f %f %f",msg->twist.covariance.elems[i],msg->twist.covariance.elems[i+1],msg->twist.covariance.elems[i+2],msg->twist.covariance.elems[i+3],msg->twist.covariance.elems[i+4],msg->twist.covariance.elems[i+5]);

	}
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"odom_test");
	ros::NodeHandle nh;
	ros::Subscriber listener = nh.subscribe<nav_msgs::Odometry>("/odom",10,listener_callback);
	ros::spin();
	return 0;
}
