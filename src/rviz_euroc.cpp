//Apply transform for different dataset file
//This file is for MH_easy_01 Euroc dataset
//Visualize output and groundtruth data in Rviz when running Vinsmono with Euroc dataset

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"

#include <sstream>

nav_msgs::Path path_msg;
geometry_msgs::PointStamped pointstamped_msg;
geometry_msgs::PoseStamped posestamped_msg;

void groud_truth_sub_callback(const geometry_msgs::PointStamped msg)
{
	posestamped_msg.header = msg.header;
	posestamped_msg.header.frame_id = "ground_truth_frame";

	posestamped_msg.pose.position.x = msg.point.x-4.782299717479681;
	posestamped_msg.pose.position.y = msg.point.y+1.815573627707949;
	posestamped_msg.pose.position.z = msg.point.z-0.844627073703417;

	path_msg.poses.push_back(posestamped_msg);
	path_msg.header.frame_id = "ground_truth_frame";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_in_rviz_euroc");
  ros::NodeHandle n;

  ros::Publisher ground_truth_pub = n.advertise<nav_msgs::Path>("/ground_truth_flightgoogles", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/leica/position",1000,groud_truth_sub_callback);

	tf::TransformBroadcaster br;
	tf::Transform transform(tf::Quaternion(0,0,0.9659258,-0.258819),tf::Vector3(0,0,0));

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
		//transform.setOrigin(tf::Vector3(-4.78221187,1.814786,-0.85767));
		//transform.setRotation(tf::Quaternion(0,0,0.9659258,-0.258819));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","ground_truth_frame"));

    ground_truth_pub.publish(path_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
