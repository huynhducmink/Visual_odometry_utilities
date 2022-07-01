// Transform output of VIO from body frame into world frame
// Visualize both output and groundtruth in RVIZ
#include "vio_transform.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vio_transform");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  VIO_transform VIO_transform_instance(nh,nh_private);
  ros::spin();
}
