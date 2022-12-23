#include "vio_transform.h"

VIO_transform::VIO_transform(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    sendTransform(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1)); //to init transformation to not show transformation error before the state estimation initialize
    groundtruth_type_choice();
}

VIO_transform::~VIO_transform(){}

void VIO_transform::groundtruth_type_choice(){
		groundtruth_topic = "/mavros/local_position/odom";
		ROS_INFO("Use /mavros/local_position/odom as groundtruth");
    init_pub_sub();
}

void VIO_transform::init_pub_sub(){
    // groundtruth taken from mavros
    ground_truth_pub_odo = nh_.advertise<nav_msgs::Odometry>("/ground_truth_odo", 10);
    ground_truth_pub_path = nh_.advertise<nav_msgs::Path>("/ground_truth_path", 10);
    ground_truth_pub_tf = nh_.advertise<geometry_msgs::TransformStamped>("/ground_truth_tfstamped", 10);
    groud_truth_sub = nh_.subscribe(groundtruth_topic, 10, &VIO_transform::ground_truth_sub_callback, this);

    // groundtruth taken from Gazebo API
    gazebo_pub_odo = nh_.advertise<nav_msgs::Odometry>("/gazebo_groundtruth_odo",10);
    gazebo_pub_posestamped = nh_.advertise<geometry_msgs::PoseStamped>("/gazebo_groundtruth_posestamped",10);
    gazebo_pub_path = nh_.advertise<nav_msgs::Path>("/gazebo_groundtruth_path",10);
    VIO_transform::gazebo_state_client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

    // state estimation
    vio_pub_odo = nh_.advertise<nav_msgs::Odometry>("/vio_odo",1);
    vio_pub_odo_to_px4 = nh_.advertise<nav_msgs::Odometry>("/mavros/odometry/out",1);
    vio_pub_odo_posestamped_to_px4 = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1);
    vio_pub_odo_posestamped = nh_.advertise<geometry_msgs::PoseStamped>("/vio_odo_posestamped", 1);
    vio_pub_path = nh_.advertise<nav_msgs::Path>("/vio_path", 10);
    vins_vio_odo_sub = nh_.subscribe("/vins_estimator/odometry", 10, &VIO_transform::vins_vio_odo_sub_callback, this);
    vins_bool_receive_first_image = nh_.subscribe("/vins_estimator/bool_receive_first_image", 10, &VIO_transform::vins_bool_receive_first_image_callback, this);
    rovio_vio_odo_sub = nh_.subscribe("/rovio/pose_with_covariance_stamped", 10, &VIO_transform::rovio_vio_odo_sub_callback, this);
    orbslam3_vio_odo_sub = nh_.subscribe("/out_odom", 10, &VIO_transform::orbslam3_vio_odo_sub_callback, this);
    msf_vio_odo_sub = nh_.subscribe("/msf_core/pose", 10, &VIO_transform::msf_vio_odo_sub_callback, this);

    ROS_INFO("Finish init ROS publisher and subscriber");
    transforming_VIO_output();
}

void VIO_transform::transforming_VIO_output(){
  world_vio_posestamped_msg.pose.orientation.w=1;

  tf::TransformListener lidar_tf_listener;
  tf::TransformBroadcaster lidar_tf_broadcaster;
  tf::StampedTransform lidar_tf_msg; 

  while (ros::ok())
  {

    //gazebo_pub_odo.publish(gazebo_odo_msg);
    //gazebo_pub_path.publish(gazebo_path_msg);

    //vio_pub_odo_posestamped_to_px4.publish(vio_posestamped_to_px4_msg);
    //std::cout << "Current VIO position in world: " << world_vio_odo_msg.pose.pose.position.x << " | " 
    //    << world_vio_odo_msg.pose.pose.position.y << " | " 
    //    << world_vio_odo_msg.pose.pose.position.z << std::endl;
    //std::cout << "Current groundtruth position in world: " << gt_odo_msg.pose.pose.position.x << " | " 
    //    << gt_odo_msg.pose.pose.position.y << " | " 
    //    << gt_odo_msg.pose.pose.position.z << std::endl;
    // gazebo_getmodelstate.request.model_name="iris";
    // if (gazebo_state_client.call(gazebo_getmodelstate)){
    //     gazebo_odo_msg.pose.pose = gazebo_getmodelstate.response.pose;
    //     gazebo_odo_msg.twist.twist = gazebo_getmodelstate.response.twist;
    //     gazebo_odo_msg.header.frame_id = "world";
    //     gazebo_posestamped_msg.pose.position = gazebo_getmodelstate.response.pose.position;
    //     gazebo_posestamped_msg.pose.orientation = gazebo_getmodelstate.response.pose.orientation;
    //     gazebo_posestamped_msg.header.frame_id = "world";
    //     gazebo_path_msg.poses.push_back(gazebo_posestamped_msg);
    //     gazebo_path_msg.header.frame_id = "world";

    //     gazebo_pub_odo.publish(gazebo_odo_msg);
    //     gazebo_pub_posestamped.publish(gazebo_posestamped_msg);
    //     gazebo_pub_path.publish(gazebo_path_msg);
    // }
    // else{}

    // Lidar tranform
//    try{
//      lidar_tf_listener.lookupTransform("/camera_init","aft_mapped",ros::Time(0),lidar_tf_msg);
//      lidar_tf_msg.frame_id_ = "world";
//      lidar_tf_msg.child_frame_id_ = "os_sensor";
//      lidar_tf_msg.stamp_ = ros::Time::now();
//      lidar_tf_broadcaster.sendTransform(lidar_tf_msg);
//    }
//    catch (tf::TransformException ex) {
//      ROS_ERROR("%s", ex.what());
//      ros::Duration(1.0).sleep();
//    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void VIO_transform::sendTransform(tf::Vector3 input_origin, tf::Quaternion input_rotation)
{
    br_transform.setOrigin(input_origin);
    br_transform.setRotation(input_rotation);
    broadcaster_transform.sendTransform(tf::StampedTransform(br_transform, ros::Time::now(), "world", "vio_frame"));
    init_transformation = false;
}

// use for groundtruth taken from mavros (with odometry format)

void VIO_transform::ground_truth_sub_callback(const nav_msgs::Odometry msg)
{
    if (init_transformation == true)
    {
        sendTransform(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                      tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
        ROS_INFO("Receive initialization message from visual odometry");
    }
    gt_odo_msg.pose.pose = msg.pose.pose;

    gt_posestamped_msg.pose.position.x = gt_odo_msg.pose.pose.position.x;
    gt_posestamped_msg.pose.position.y = gt_odo_msg.pose.pose.position.y;
    gt_posestamped_msg.pose.position.z = gt_odo_msg.pose.pose.position.z;

    gt_path_msg.poses.push_back(gt_posestamped_msg);
    gt_odo_msg.header.frame_id = "world";
    gt_path_msg.header.frame_id = "world";

    gt_tfstamped_msg.header = msg.header;
    gt_tfstamped_msg.header.frame_id = "/world";
    gt_tfstamped_msg.child_frame_id = "/camera_link";
    gt_tfstamped_msg.transform.translation.x = msg.pose.pose.position.x;
    gt_tfstamped_msg.transform.translation.y = msg.pose.pose.position.y;
    gt_tfstamped_msg.transform.translation.z = msg.pose.pose.position.z;
    gt_tfstamped_msg.transform.rotation.x = msg.pose.pose.orientation.x;
    gt_tfstamped_msg.transform.rotation.y = msg.pose.pose.orientation.y;
    gt_tfstamped_msg.transform.rotation.z = msg.pose.pose.orientation.z;
    gt_tfstamped_msg.transform.rotation.w = msg.pose.pose.orientation.w;

    static tf::TransformBroadcaster map_to_base_tf_br;
    tf::Transform map_to_base_tf;
    map_to_base_tf.setOrigin(tf::Vector3(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z));
    map_to_base_tf.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w));
    map_to_base_tf_br.sendTransform(tf::StampedTransform(map_to_base_tf,msg.header.stamp,"world","base_link"));

    ground_truth_pub_odo.publish(gt_odo_msg);
    ground_truth_pub_path.publish(gt_path_msg);
    ground_truth_pub_tf.publish(gt_tfstamped_msg);
}


void VIO_transform::vins_vio_odo_sub_callback(const nav_msgs::Odometry msg)
{
    vio_posestamped_msg.pose = msg.pose.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_odo_msg.pose.pose = world_vio_posestamped_msg.pose;
    world_vio_odo_msg.header = world_vio_posestamped_msg.header;
    world_vio_odo_msg.header.frame_id = "world";
    world_vio_odo_msg.header.stamp = msg.header.stamp;
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_posestamped_msg.header.stamp = msg.header.stamp;
    world_vio_path_msg.header.frame_id = "world";
    world_vio_path_msg.header.stamp = msg.header.stamp;
}

void VIO_transform::rovio_vio_odo_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    vio_posestamped_msg.pose = msg.pose.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_odo_msg.pose.pose = world_vio_posestamped_msg.pose;
    world_vio_odo_msg.header = world_vio_posestamped_msg.header;
    world_vio_odo_msg.header.frame_id = "world";
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
    vio_pub_odo.publish(world_vio_odo_msg);
    vio_pub_odo_to_px4.publish(vio_odo_to_px4_msg);
    vio_pub_odo_posestamped.publish(world_vio_posestamped_msg);
    vio_pub_path.publish(world_vio_path_msg);
}

void VIO_transform::orbslam3_vio_odo_sub_callback(const geometry_msgs::PoseStamped msg)
{
    vio_posestamped_msg.pose = msg.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_odo_msg.pose.pose = world_vio_posestamped_msg.pose;
    world_vio_odo_msg.header = world_vio_posestamped_msg.header;
    world_vio_odo_msg.header.frame_id = "world";
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
    vio_pub_odo.publish(world_vio_odo_msg);
    vio_pub_odo_to_px4.publish(vio_odo_to_px4_msg);
    vio_pub_odo_posestamped.publish(world_vio_posestamped_msg);
    vio_pub_path.publish(world_vio_path_msg);
}

void VIO_transform::msf_vio_odo_sub_callback(const geometry_msgs::PoseWithCovarianceStamped msg) //convert from odometry to pose //PoseWithCovarianceStamped
{
    vio_posestamped_msg.pose = msg.pose.pose;
    vio_posestamped_msg.header.frame_id = "vio_frame";
    listener_transform.transformPose("world", vio_posestamped_msg, world_vio_posestamped_msg);
    world_vio_path_msg.poses.push_back(world_vio_posestamped_msg);
    world_vio_odo_msg.pose.pose = world_vio_posestamped_msg.pose;
    world_vio_odo_msg.header = world_vio_posestamped_msg.header;
    world_vio_odo_msg.header.frame_id = "world";
    world_vio_odo_msg.header.stamp = msg.header.stamp;
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_posestamped_msg.header.stamp = msg.header.stamp;
    world_vio_path_msg.header.frame_id = "world";
    world_vio_path_msg.header.stamp = msg.header.stamp;
    vio_pub_odo.publish(world_vio_odo_msg);
    vio_pub_odo_to_px4.publish(vio_odo_to_px4_msg);
    vio_pub_odo_posestamped.publish(world_vio_posestamped_msg);
    vio_pub_path.publish(world_vio_path_msg);
}

void VIO_transform::vins_bool_receive_first_image_callback(const std_msgs::Bool msg){
    if (msg.data == true){
        init_transformation = false;
    }
}