#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <sstream>

nav_msgs::Odometry odo_msg;
geometry_msgs::PoseStamped posestamped_msg;
nav_msgs::Path path_msg;

void groud_truth_sub_callback(const nav_msgs::Odometry msg)
{
	odo_msg = msg;
	odo_msg.pose.pose.position.x -=18.50682276464927;
	odo_msg.pose.pose.position.y -=6.5108016138058185;
	odo_msg.pose.pose.position.z -=1.0029971060048706;

	posestamped_msg.pose.position.x = odo_msg.pose.pose.position.x;
	posestamped_msg.pose.position.y = odo_msg.pose.pose.position.y;
	posestamped_msg.pose.position.z = odo_msg.pose.pose.position.z;
	
	path_msg.poses.push_back(posestamped_msg);
	path_msg.header.frame_id = "world";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_in_rviz_flightgoogles");

  ros::NodeHandle n;

  ros::Publisher ground_truth_pub_odo = n.advertise<nav_msgs::Odometry>("/ground_truth_flightgoogles_odo", 1000);
  ros::Publisher ground_truth_pub_path = n.advertise<nav_msgs::Path>("/ground_truth_flightgoogles_path", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/uav/odometry",1000,groud_truth_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ground_truth_pub_odo.publish(odo_msg);
    ground_truth_pub_path.publish(path_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
