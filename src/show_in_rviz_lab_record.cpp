#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_broadcaster.h"
#include <sstream>

nav_msgs::Odometry odo_msg;
geometry_msgs::PoseStamped posestamped_msg;
nav_msgs::Path path_msg;

void groud_truth_sub_callback(const nav_msgs::Odometry msg)
{
	odo_msg = msg;
	odo_msg.pose.pose.position.x -=0.9876729464347902;
	odo_msg.pose.pose.position.y -=(-2.081473903268754);
	odo_msg.pose.pose.position.z -=0.128;

	posestamped_msg.pose.position.x = odo_msg.pose.pose.position.x;
	posestamped_msg.pose.position.y = odo_msg.pose.pose.position.y;
	posestamped_msg.pose.position.z = odo_msg.pose.pose.position.z;
	
	path_msg.poses.push_back(posestamped_msg);
	path_msg.header.frame_id = "ground_truth_frame";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_in_rviz_lab_record");

  ros::NodeHandle n;

  ros::Publisher ground_truth_pub_odo = n.advertise<nav_msgs::Odometry>("/ground_truth_lab_record_odo", 1000);
  ros::Publisher ground_truth_pub_path = n.advertise<nav_msgs::Path>("/ground_truth_lab_record_path", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/mavros/global_position/local",1000,groud_truth_sub_callback);

	tf::TransformBroadcaster br;
	tf::Transform transform(tf::Quaternion(0.0412151,-0.0150899,0.0250079,-0.9987232),tf::Vector3(0,0,0));

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","ground_truth_frame"));
    ground_truth_pub_odo.publish(odo_msg);
    ground_truth_pub_path.publish(path_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
