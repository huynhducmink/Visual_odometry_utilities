#include "vio_transform.h"

VIO_transform::VIO_transform(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private)
{
    sendTransform(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1)); //to init transformation to not show transformation error before the state estimation initialize
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
        groundtruth_topic = "/gazebo_groundtruth_posestamped";
        ROS_INFO("Transforming output of flightmare dataset");
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
    // groundtruth taken from mavros
    ground_truth_pub_odo = nh_.advertise<nav_msgs::Odometry>("/ground_truth_odo", 10);
    ground_truth_pub_path = nh_.advertise<nav_msgs::Path>("/ground_truth_path", 10);
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

    std::cout << groundtruth_topic << std::endl;
    ROS_INFO("FINISH INITIALIZATION ros publisher and subscriber");
    ROS_INFO("START TRANSFORMING OUTPUT OF VIO FROM VIO_FRAME TO WORLD FRAME");
    transforming_VIO_output();
}

void VIO_transform::transforming_VIO_output(){
    world_vio_posestamped_msg.pose.orientation.w=1;
  while (ros::ok())
  {
    ground_truth_pub_odo.publish(gt_odo_msg);
    ground_truth_pub_path.publish(gt_path_msg);

    gazebo_pub_odo.publish(gazebo_odo_msg);
    gazebo_pub_path.publish(gazebo_path_msg);

    vio_pub_odo.publish(world_vio_odo_msg);
    vio_pub_odo_to_px4.publish(vio_odo_to_px4_msg);
    vio_pub_odo_posestamped.publish(world_vio_posestamped_msg);
    //vio_pub_odo_posestamped_to_px4.publish(vio_posestamped_to_px4_msg);
    vio_pub_path.publish(world_vio_path_msg);
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

// use for groundtruth taken from Gazebo API (with posestamped format)
// void VIO_transform::ground_truth_sub_callback(const geometry_msgs::PoseStamped msg)
// {
//     if (init_transformation == true)
//     {
//         sendTransform(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
//                       tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
//         std::cout << msg.pose.position.x << " | " << msg.pose.position.y << " | " << msg.pose.position.z << std::endl;
//     }
//     gt_odo_msg.pose.pose = msg.pose;

//     gt_posestamped_msg.pose.position.x = gt_odo_msg.pose.pose.position.x;
//     gt_posestamped_msg.pose.position.y = gt_odo_msg.pose.pose.position.y;
//     gt_posestamped_msg.pose.position.z = gt_odo_msg.pose.pose.position.z;

//     gt_path_msg.poses.push_back(gt_posestamped_msg);
//     gt_odo_msg.header.frame_id = "world";
//     gt_path_msg.header.frame_id = "world";
// }

// use for groundtruth taken from mavros (with odometry format)

void VIO_transform::ground_truth_sub_callback(const nav_msgs::Odometry msg)
{
    if (init_transformation == true)
    {
        sendTransform(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                      tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
        std::cout << msg.pose.pose.position.x << " | " << msg.pose.pose.position.y << " | " << msg.pose.pose.position.z << std::endl;
    }
    gt_odo_msg.pose.pose = msg.pose.pose;

    gt_posestamped_msg.pose.position.x = gt_odo_msg.pose.pose.position.x;
    gt_posestamped_msg.pose.position.y = gt_odo_msg.pose.pose.position.y;
    gt_posestamped_msg.pose.position.z = gt_odo_msg.pose.pose.position.z;

    gt_path_msg.poses.push_back(gt_posestamped_msg);
    gt_odo_msg.header.frame_id = "world";
    gt_path_msg.header.frame_id = "world";
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
    world_vio_posestamped_msg.header.frame_id = "world";
    world_vio_path_msg.header.frame_id = "world";
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
}

void VIO_transform::vins_bool_receive_first_image_callback(const std_msgs::Bool msg){
    if (msg.data == true){
        init_transformation = true;
    }
}