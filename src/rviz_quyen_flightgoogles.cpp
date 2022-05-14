// Visualize output and groundtruth data in Rviz when running ROVIO with Flightgoogles dataset
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <sstream>

nav_msgs::Odometry odo_msg;

void orb_sub_callback(const nav_msgs::Odometry msg)
{
    odo_msg = msg;
    odo_msg.pose.pose.position.x += 18.50682276464927;
    odo_msg.pose.pose.position.y += 6.5108016138058185;
    odo_msg.pose.pose.position.z += 1.0029971060048706;
    odo_msg.header.frame_id = "world";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_quyen_flightgoogles");

    ros::NodeHandle n;

    ros::Publisher orb_pub_odo = n.advertise<nav_msgs::Odometry>("/orb_odom", 1000);
    ros::Subscriber orb_sub_odo = n.subscribe("/out_odom", 1000, orb_sub_callback);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        orb_pub_odo.publish(odo_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
