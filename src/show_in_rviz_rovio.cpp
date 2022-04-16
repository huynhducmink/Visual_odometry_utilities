#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sstream>

nav_msgs::Odometry odo_msg;
geometry_msgs::PoseStamped posestamped_msg;
nav_msgs::Path path_msg;

geometry_msgs::PoseWithCovarianceStamped rovio_msg;
geometry_msgs::PoseStamped rovio_posestamped_msg;
nav_msgs::Path rovio_path_msg;


void groud_truth_sub_callback(const nav_msgs::Odometry msg)
{
	odo_msg = msg;

	posestamped_msg.pose.position.x = odo_msg.pose.pose.position.x;
	posestamped_msg.pose.position.y = odo_msg.pose.pose.position.y;
	posestamped_msg.pose.position.z = odo_msg.pose.pose.position.z;
	
	path_msg.poses.push_back(posestamped_msg);
	path_msg.header.frame_id = "world";
}

void rovio_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	rovio_msg = msg;

	rovio_posestamped_msg.pose.position.x = rovio_msg.pose.pose.position.x;
	rovio_posestamped_msg.pose.position.y = rovio_msg.pose.pose.position.y;
	rovio_posestamped_msg.pose.position.z = rovio_msg.pose.pose.position.z;

	rovio_path_msg.poses.push_back(rovio_posestamped_msg);
	rovio_path_msg.header.frame_id = "world";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_in_rviz_rovio");

  ros::NodeHandle n;

  ros::Publisher ground_truth_pub_odo = n.advertise<nav_msgs::Odometry>("/ground_truth_flightgoogles_odo", 1000);
  ros::Publisher ground_truth_pub_path = n.advertise<nav_msgs::Path>("/ground_truth_flightgoogles_path", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/uav/odometry",1000,groud_truth_sub_callback);

	ros::Subscriber rovio_sub = n.subscribe("/rovio/pose_with_covariance_stamped",1000,rovio_sub_callback);
  ros::Publisher rovio_pub_path = n.advertise<nav_msgs::Path>("/rovio_flightgoogles_path", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ground_truth_pub_odo.publish(odo_msg);
    ground_truth_pub_path.publish(path_msg);

		rovio_pub_path.publish(rovio_path_msg);
		
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
