//Visualize output and groundtruth data in Rviz when running Vinsmono with flightgoogles dataset on Rovio
//@Quyen
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sstream>

geometry_msgs::PoseStamped posestamped_msg;
nav_msgs::Path path_msg;

geometry_msgs::PoseWithCovarianceStamped rovio_msg;
geometry_msgs::PoseStamped rovio_posestamped_msg;
nav_msgs::Path rovio_path_msg;


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
  ros::init(argc, argv, "rviz_rovio_euroc");

  ros::NodeHandle n;

  ros::Publisher ground_truth_pub = n.advertise<nav_msgs::Path>("/ground_truth_flightgoogles", 1000);
	ros::Subscriber groud_truth_sub = n.subscribe("/leica/position",1000,groud_truth_sub_callback);

	ros::Subscriber rovio_sub = n.subscribe("/rovio/pose_with_covariance_stamped",1000,rovio_sub_callback);
  ros::Publisher rovio_pub_path = n.advertise<nav_msgs::Path>("/rovio_flightgoogles_path", 1000);

	tf::TransformBroadcaster br;
	tf::Transform transform(tf::Quaternion(0, 0, 0.258819, 0.9659258),tf::Vector3(0,0,0));

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world","ground_truth_frame"));

    ground_truth_pub.publish(path_msg);

		rovio_pub_path.publish(rovio_path_msg);
		
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
