# VINSMONO_groundtruth_visualization

This is a ROS package use to transform output of VIO (VINS_MONO, ROVIO, ORB_SLAM_3) to the world frame and visualize it in RVIZ

Clone the repository into catkin_ws/src and change package name to read_topic


To run:
```php
rosrun read_topic vio_transform
```

Then choose the appropiate dataset type using input 1,2,3 or 4 then Enter. The same with VIO program type.

Run this package before running the dataset

Work in progress: real time running with simulation program
