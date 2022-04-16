//Visualize output and groundtruth data in Rviz when running Vinsmono with Flightgoogles dataset on Jetson (only have odo topic)
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <sstream>

nav_msgs::Odometry gt_odo_msg;
geometry_msgs::PoseStamped gt_posestamped_msg;
nav_msgs::Path gt_path_msg;

nav_msgs::Odometry vins_odo_msg;
geometry_msgs::PoseStamped vins_posestamped_msg;
nav_msgs::Path vins_path_msg;

void groud_truth_sub_callback(const nav_msgs::Odometry msg)
{
	gt_odo_msg = msg;
	gt_odo_msg.pose.pose.position.x -=18.50682276464927;
	gt_odo_msg.pose.pose.position.y -=6.5108016138058185;
	gt_odo_msg.pose.pose.position.z -=1.0029971060048706;

	gt_posestamped_msg.pose.position.x = gt_odo_msg.pose.pose.position.x;
	gt_posestamped_msg.pose.position.y = gt_odo_msg.pose.pose.position.y;
	gt_posestamped_msg.pose.position.z = gt_odo_msg.pose.pose.position.z;
	
	gt_path_msg.poses.push_back(gt_posestamped_msg);
	gt_path_msg.header.frame_id = "world";
}
void vins_odo_sub_callback(const nav_msgs::Odometry msg)
{
	vins_odo_msg = msg;

	vins_posestamped_msg.pose.position.x = vins_odo_msg.pose.pose.position.x;
	vins_posestamped_msg.pose.position.y = vins_odo_msg.pose.pose.position.y;
	vins_posestamped_msg.pose.position.z = vins_odo_msg.pose.pose.position.z;
	
	vins_path_msg.poses.push_back(vins_posestamped_msg);
	vins_path_msg.header.frame_id = "world";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_in_rviz_jetson");

  ros::NodeHandle n;

  ros::Publisher ground_truth_pub_odo = n.advertise<nav_msgs::Odometry>("/ground_truth_flightgoogles_odo", 1000);
  ros::Publisher ground_truth_pub_path = n.advertise<nav_msgs::Path>("/ground_truth_flightgoogles_path", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/uav/odometry",1000,groud_truth_sub_callback);

  ros::Publisher vins_pub_odo = n.advertise<nav_msgs::Odometry>("/vins_flightgoogles_odo", 1000);
  ros::Publisher vins_pub_path = n.advertise<nav_msgs::Path>("/vins_flightgoogles_path", 1000);
	ros::Subscriber vins_odo_sub = n.subscribe("/vins_estimator/odometry",1000,vins_odo_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ground_truth_pub_odo.publish(gt_odo_msg);
    ground_truth_pub_path.publish(gt_path_msg);
    vins_pub_odo.publish(vins_odo_msg);
    vins_pub_path.publish(vins_path_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
