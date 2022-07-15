#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <sstream>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Geometry>
#include <string>
#include <iostream>

class VIO_transform
{
public:
    VIO_transform(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    ~VIO_transform();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // From mavros
    ros::Publisher ground_truth_pub_odo;
    ros::Publisher ground_truth_pub_path;
    ros::Subscriber groud_truth_sub;
    ros::Subscriber groud_truth_sub_temp;

    // From Gazebo API
    ros::Subscriber gazebo_sub_odo_pose;
    ros::Publisher gazebo_pub_odo;
    ros::Publisher gazebo_pub_path;

    ros::Publisher vio_pub_odo;
    ros::Publisher vio_pub_odo_to_px4;
    ros::Publisher vio_pub_odo_posestamped;
    ros::Publisher vio_pub_odo_posestamped_to_px4;
    ros::Publisher vio_pub_path;
    ros::Subscriber vins_vio_odo_sub;
    ros::Subscriber rovio_vio_odo_sub;
    ros::Subscriber orbslam3_vio_odo_sub;

    ros::Rate loop_rate = 10;

    std::string groundtruth_topic = "/default1";      // default
    std::string groundtruth_topic_temp = "/default2"; // default

    // Ground truth messages (in world frame) (from mavros)
    nav_msgs::Odometry gt_odo_msg;
    geometry_msgs::PoseStamped gt_posestamped_msg;
    nav_msgs::Path gt_path_msg;

    // Ground truth message (from Gazebo API)
    gazebo_msgs::GetModelState gazebo_getmodelstate;
    gazebo_msgs::ModelState gazebo_state;
    geometry_msgs::PoseStamped gazebo_posestamped_msg;
    nav_msgs::Odometry gazebo_odo_msg;
    nav_msgs::Path gazebo_path_msg;

    // Visual odometry messages
    // In vio frame (in the first camera image frame)
    nav_msgs::Odometry vio_odo_msg;
    nav_msgs::Odometry vio_odo_to_px4_msg;
    geometry_msgs::PoseStamped vio_posestamped_msg;
    geometry_msgs::PoseStamped vio_posestamped_to_px4_msg;
    nav_msgs::Path vio_path_msg;
    // In world frame
    nav_msgs::Odometry world_vio_odo_msg;
    geometry_msgs::PoseStamped world_vio_posestamped_msg;
    nav_msgs::Path world_vio_path_msg;

    // Use to transform from VIO frame to world frame
    tf::Quaternion world_to_body_rotation;
    tf::Vector3 world_to_body_origin;
    tf::TransformBroadcaster broadcaster_transform;
    tf::Transform br_transform;
    tf::TransformListener listener_transform;
    tf::StampedTransform ln_transform;

    // Run 1 time to take the first message of the ground truth odometry to construct the transformation matrix
    bool init_transformation = false;

    void groundtruth_type_choice();
    void init_pub_sub();
    void transforming_VIO_output();
    void sendTransform(tf::Vector3 input_origin, tf::Quaternion input_rotation);

    void ground_truth_sub_callback(const nav_msgs::Odometry msg);
    void ground_truth_sub_callback_temp(const geometry_msgs::PoseWithCovarianceStamped msg);

    ros::ServiceClient gazebo_state_client;

    void vins_vio_odo_sub_callback(const nav_msgs::Odometry msg);
    void rovio_vio_odo_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg);
    void orbslam3_vio_odo_sub_callback(const geometry_msgs::PoseStamped msg);
};