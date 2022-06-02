// Transform output of VIO from body frame into world frame
// Visualize both output and groundtruth in RVIZ
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <string>
#include <iostream>

// Ground truth messages
nav_msgs::Odometry gt_odo_msg;
geometry_msgs::PoseStamped gt_posestamped_msg;
nav_msgs::Path gt_path_msg;

// Visual odometry messages
nav_msgs::Odometry vio_odo_msg;
geometry_msgs::PoseStamped vio_posestamped_msg;
nav_msgs::Path vio_path_msg;

// Use to calculate transformation matrix from VIO frame to world frame
Eigen::Vector4f vio_output;
Eigen::Vector4f vio_output_world;
Eigen::Matrix4f world_to_body;
Eigen::Quaternionf q;
Eigen::Matrix3f R;

// Run 1 time to take the first message of the ground truth odometry to construct the transformation matrix
// In case of running in real time, set this value to false then modify the world_to_body matrix in the main function to the correct matrix
// Work in progress, move this setting to a launch file
bool init_transformation_matrix = true;

void ground_truth_sub_callback(const nav_msgs::Odometry msg)
{
  if (init_transformation_matrix == true)
  {
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;
    q.w() = msg.pose.pose.orientation.w;
    R = q.toRotationMatrix();
    world_to_body << R(0, 0), R(0, 1), R(0, 2), msg.pose.pose.position.x,
        R(1, 0), R(1, 1), R(1, 2), msg.pose.pose.position.y,
        R(2, 0), R(2, 1), R(2, 2), msg.pose.pose.position.z,
        0, 0, 0, 1;
    init_transformation_matrix = false;
    ROS_INFO("Initialize transformation matrix successfully!");
  }
  else
  {
    gt_odo_msg = msg;

    gt_posestamped_msg.pose.position.x = gt_odo_msg.pose.pose.position.x;
    gt_posestamped_msg.pose.position.y = gt_odo_msg.pose.pose.position.y;
    gt_posestamped_msg.pose.position.z = gt_odo_msg.pose.pose.position.z;

    gt_path_msg.poses.push_back(gt_posestamped_msg);
    gt_odo_msg.header.frame_id = "world";
    gt_path_msg.header.frame_id = "world";
  }
}

void ground_truth_sub_callback_temp(const geometry_msgs::PoseWithCovarianceStamped msg)
{
  if (init_transformation_matrix == true)
  {
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;
    q.w() = msg.pose.pose.orientation.w;
    R = q.toRotationMatrix();
    world_to_body << R(0, 0), R(0, 1), R(0, 2), msg.pose.pose.position.x,
        R(1, 0), R(1, 1), R(1, 2), msg.pose.pose.position.y,
        R(2, 0), R(2, 1), R(2, 2), msg.pose.pose.position.z,
        0, 0, 0, 1;
    init_transformation_matrix = false;
    ROS_INFO("Initialize transformation matrix successfully!");
  }
  else
  {
    gt_posestamped_msg.pose.position.x = msg.pose.pose.position.x;
    gt_posestamped_msg.pose.position.y = msg.pose.pose.position.y;
    gt_posestamped_msg.pose.position.z = msg.pose.pose.position.z;

    gt_path_msg.poses.push_back(gt_posestamped_msg);
    gt_odo_msg.header.frame_id = "world";
    gt_path_msg.header.frame_id = "world";
  }
}

void vins_vio_odo_sub_callback(const nav_msgs::Odometry msg)
{
  vio_output << msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,1;
  vio_output_world = world_to_body*vio_output; // Transform from VIO frame to world frame

	vio_odo_msg.pose.pose.position.x = vio_output_world(0,0);
	vio_odo_msg.pose.pose.position.y = vio_output_world(1,0);
	vio_odo_msg.pose.pose.position.z = vio_output_world(2,0);

	vio_posestamped_msg.pose.position.x = vio_odo_msg.pose.pose.position.x;
	vio_posestamped_msg.pose.position.y = vio_odo_msg.pose.pose.position.y;
	vio_posestamped_msg.pose.position.z = vio_odo_msg.pose.pose.position.z;
	
	vio_path_msg.poses.push_back(vio_posestamped_msg);
  vio_odo_msg.header.frame_id = "world";
	vio_path_msg.header.frame_id = "world";
}

void rovio_vio_odo_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
  vio_output << msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,1;
  vio_output_world = world_to_body*vio_output; // Transform from VIO frame to world frame

	vio_odo_msg.pose.pose.position.x = vio_output_world(0,0);
	vio_odo_msg.pose.pose.position.y = vio_output_world(1,0);
	vio_odo_msg.pose.pose.position.z = vio_output_world(2,0);

	vio_posestamped_msg.pose.position.x = vio_odo_msg.pose.pose.position.x;
	vio_posestamped_msg.pose.position.y = vio_odo_msg.pose.pose.position.y;
	vio_posestamped_msg.pose.position.z = vio_odo_msg.pose.pose.position.z;
	
	vio_path_msg.poses.push_back(vio_posestamped_msg);
  vio_odo_msg.header.frame_id = "world";
	vio_path_msg.header.frame_id = "world";
}

void orbslam3_vio_odo_sub_callback(const geometry_msgs::PoseStamped msg)
{
  vio_output << msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,1;
  vio_output_world = world_to_body*vio_output; // Transform from VIO frame to world frame

	vio_odo_msg.pose.pose.position.x = vio_output_world(0,0);
	vio_odo_msg.pose.pose.position.y = vio_output_world(1,0);
	vio_odo_msg.pose.pose.position.z = vio_output_world(2,0);

	vio_posestamped_msg.pose.position.x = vio_odo_msg.pose.pose.position.x;
	vio_posestamped_msg.pose.position.y = vio_odo_msg.pose.pose.position.y;
	vio_posestamped_msg.pose.position.z = vio_odo_msg.pose.pose.position.z;
	
	vio_path_msg.poses.push_back(vio_posestamped_msg);
  vio_odo_msg.header.frame_id = "world";
	vio_path_msg.header.frame_id = "world";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vio_transform");

  ros::NodeHandle n;

	// Default transformation matrix
	// Modify if necessary
  world_to_body << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  int groundtruth_choice;
  ROS_INFO("Please input groundtruth topic");
  ROS_INFO("Input 1 for flightgoogle");
  ROS_INFO("Input 2 for euroc (Not working yet as groundtruth of EUROC is in different format)");
  ROS_INFO("Input 3 for airsim");
  ROS_INFO("Input 4 for flightmare");
  ROS_INFO("Input 5 for flightmare (temp) (pose_with_covariance groundtruth)");

  std::string groundtruth_topic = "/uav/odometry"; //default
  std::string groundtruth_topic_temp = "/uav/odometry"; //default

  std::cin >> groundtruth_choice;
  if (groundtruth_choice == 1){
    groundtruth_topic = "/uav/odometry";
    ROS_INFO("Transforming output of flightgoogles dataset");
  }
  else if (groundtruth_choice == 2){
    groundtruth_topic = "/leica/position";
    ROS_INFO("Transforming output of euroc dataset (Not working yet as groundtruth of EUROC is in different format)");
  }
  else if (groundtruth_choice == 3){
    groundtruth_topic = "/mavros/local_position/odom";
    ROS_INFO("Transforming output of airsim dataset");
  }
  else if (groundtruth_choice == 4){
    groundtruth_topic = "/hummingbird/ground_truth/odometry";
    ROS_INFO("Transforming output of flightmare dataset");
  }
  else if (groundtruth_choice == 5){
    groundtruth_topic_temp = "/hummingbird/ground_truth/pose_with_covariance";
    ROS_INFO("Transforming output of flightmare dataset (temp) (pose_with_covariance groundtruth)");
  }
  else{
    groundtruth_topic = "/uav/odometry";
    ROS_INFO("Transforming output of flightgoogles dataset");
  }


  ros::Publisher ground_truth_pub_odo = n.advertise<nav_msgs::Odometry>("/ground_truth_odo", 1);
  ros::Publisher ground_truth_pub_path = n.advertise<nav_msgs::Path>("/ground_truth_path", 1);
	ros::Subscriber groud_truth_sub = n.subscribe(groundtruth_topic,1,ground_truth_sub_callback);
	ros::Subscriber groud_truth_sub_temp = n.subscribe(groundtruth_topic_temp,1,ground_truth_sub_callback_temp);

  ros::Publisher vio_pub_odo = n.advertise<nav_msgs::Odometry>("/vio_odo", 1);
  ros::Publisher vio_pub_path = n.advertise<nav_msgs::Path>("/vio_path", 1);
	ros::Subscriber vins_vio_odo_sub = n.subscribe("/vins_estimator/odometry",1,vins_vio_odo_sub_callback);
	ros::Subscriber rovio_vio_odo_sub = n.subscribe("/rovio/pose_with_covariance_stamped",1,rovio_vio_odo_sub_callback);
	ros::Subscriber orbslam3_vio_odo_sub = n.subscribe("/out_odom",1,orbslam3_vio_odo_sub_callback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ground_truth_pub_odo.publish(gt_odo_msg);
    ground_truth_pub_path.publish(gt_path_msg);
    vio_pub_odo.publish(vio_odo_msg);
    vio_pub_path.publish(vio_path_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
