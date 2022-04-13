#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>

#define CSV_PATH "/home/huynhmink/catkin_ws/src/read_topic/src/result.csv"

double uav_x=0,uav_y=0,uav_z=0;
double vins_x=0,vins_y=0,vins_z=0;

void append_to_csv();
void uav_odo_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  uav_x = msg->pose.pose.position.x-18.502;
  uav_y = msg->pose.pose.position.y-6,496;
  uav_z = msg->pose.pose.position.z-1,052;
}
void vins_odo_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  vins_x = msg->pose.pose.position.x;
  vins_y = msg->pose.pose.position.y;
  vins_z = msg->pose.pose.position.z;
	append_to_csv();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_topic");
  ros::NodeHandle n;
  ros::Subscriber sub_uav_odo = n.subscribe("/uav/odometry", 1, uav_odo_Callback);
  ros::Subscriber sub_vins_odo = n.subscribe("/vins_estimator/odometry", 1, vins_odo_Callback);
  ros::spin();
  return 0;
}

void append_to_csv(){
	std::ofstream csv_file(CSV_PATH,std::ios::app);
	csv_file.precision(5);
	csv_file << uav_x << "," << uav_y << "," << uav_z << "," << vins_x << "," << vins_y << "," << vins_z << "," << std::endl;
	csv_file.close();
}
