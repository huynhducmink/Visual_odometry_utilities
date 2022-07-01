#include "vio_transform.h"

VIO_transform::VIO_transform(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    groundtruth_type_choice();
}

VIO_transform::~VIO_transform(){}

void VIO_transform::groundtruth_type_choice(){
    int groundtruth_choice;
    ROS_INFO("Please input groundtruth topic");
    ROS_INFO("Input 1 for flightgoogle");
    ROS_INFO("Input 2 for euroc (Not working yet (work in progress))");
    ROS_INFO("Input 3 for airsim");
    ROS_INFO("Input 4 for flightmare");
    ROS_INFO("Input 5 for flightmare (temp) (pose_with_covariance groundtruth)");

    std::cin >> groundtruth_choice;
    if (groundtruth_choice == 1)
    {
        groundtruth_topic = "/uav/odometry";
        ROS_INFO("Transforming output of flightgoogles dataset");
    }
    else if (groundtruth_choice == 2)
    {
        groundtruth_topic = "/leica/position";
        ROS_INFO("Transforming output of euroc dataset (Not working yet as groundtruth of EUROC is in different format)");
    }
    else if (groundtruth_choice == 3)
    {
        groundtruth_topic = "/mavros/local_position/odom";
        ROS_INFO("Transforming output of airsim dataset");
    }
    else if (groundtruth_choice == 4)
    {
        groundtruth_topic = "/hummingbird/ground_truth/odometry";
        ROS_INFO("Transforming output of flightmare dataset");
    }
    else if (groundtruth_choice == 5)
    {
        groundtruth_topic_temp = "/hummingbird/ground_truth/pose_with_covariance";
        ROS_INFO("Transforming output of flightmare dataset (temp) (pose_with_covariance groundtruth)");
    }
    else
    {
        groundtruth_topic = "/uav/odometry";
        ROS_INFO("Transforming output of flightgoogles dataset");
    }

    ROS_INFO("FINISH CHOOSING SIMULATION PROGRAM");
    init_pub_sub();
}

void VIO_transform::init_pub_sub(){
    ground_truth_pub_odo = nh_.advertise<nav_msgs::Odometry>("/ground_truth_odo", 10);
    ground_truth_pub_path = nh_.advertise<nav_msgs::Path>("/ground_truth_path", 10);
    groud_truth_sub = nh_.subscribe(groundtruth_topic, 10, &VIO_transform::ground_truth_sub_callback, this);
    groud_truth_sub_temp = nh_.subscribe(groundtruth_topic_temp, 10, &VIO_transform::ground_truth_sub_callback_temp, this);

    vio_pub_odo_posestamped = nh_.advertise<geometry_msgs::PoseStamped>("/vio_odo_posestamped", 1);
    vio_pub_path = nh_.advertise<nav_msgs::Path>("/vio_path", 10);
    vins_vio_odo_sub = nh_.subscribe("/vins_estimator/odometry", 10, &VIO_transform::vins_vio_odo_sub_callback, this);
    rovio_vio_odo_sub = nh_.subscribe("/rovio/pose_with_covariance_stamped", 10, &VIO_transform::rovio_vio_odo_sub_callback, this);
    orbslam3_vio_odo_sub = nh_.subscribe("/out_odom", 10, &VIO_transform::orbslam3_vio_odo_sub_callback, this);

    std::cout << groundtruth_topic << std::endl;
    ROS_INFO("FINISH INITIALIZATION ros publisher and subscriber");
    ROS_INFO("START TRANSFORMING OUTPUT OF VIO FROM VIO_FRAME TO WORLD FRAME");
    transforming_VIO_output();
}

void VIO_transform::transforming_VIO_output(){
  while (ros::ok())
  {
    ground_truth_pub_odo.publish(gt_odo_msg);
    ground_truth_pub_path.publish(gt_path_msg);
    vio_pub_odo_posestamped.publish(world_vio_posestamped_msg);
    vio_pub_path.publish(world_vio_path_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void VIO_transform::sendTransform(tf::Vector3 input_origin, tf::Quaternion input_rotation)
{
    br_transform.setOrigin(input_origin);
    br_transform.setRotation(input_rotation);
    broadcaster_transform.sendTransform(tf::StampedTransform(br_transform, ros::Time::now(), "world", "vio_frame"));
}

void VIO_transform::ground_truth_sub_callback(const nav_msgs::Odometry msg)
{
    if (init_transformation == true)
    {
        sendTransform(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                      tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
        init_transformation = false;
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

void VIO_transform::ground_truth_sub_callback_temp(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    if (init_transformation == true)
    {
        sendTransform(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                      tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
        init_transformation = false;
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

void VIO_transform::vins_vio_odo_sub_callback(const nav_msgs::Odometry msg)
{
    vio_posestamped_msg.pose = msg.pose.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
}

void VIO_transform::rovio_vio_odo_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    vio_posestamped_msg.pose = msg.pose.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
}

void VIO_transform::orbslam3_vio_odo_sub_callback(const geometry_msgs::PoseStamped msg)
{
    vio_posestamped_msg.pose = msg.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
}