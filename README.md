# VINSMONO_groundtruth_visualization

This is a ROS package use to transform output of VIO (VINS_MONO, ROVIO, ORB_SLAM_3) to the world frame and visualize it in RVIZ if needed

Clone the repository into catkin_ws/src and change package name to "read_topic"

## To run the transform node:
```
rosrun read_topic vio_transform
```

Then choose the appropiate dataset type using input 1,2,3 or 4 then press Enter.

## If you need to visualize in RVIZ: (VINSMONO have RVIZ built in)
```
roslaunch read_topic vio_transform_rviz.launch
```

The red line is the ground truth path in the world frame

The white line is the VIO path in the world frame

## Workflow when working with IMU+Camera dataset:

- Run the VIO package
- Run this package (and RVIZ if needed) (as VINSMONO have RVIZ built in)
- Run the dataset

## Workflow when working with live data from simulation program:

- Run the VIO package
- Run this package (and RVIZ if needed) (as VINSMONO have RVIZ built in)
- Run the simulation program

## Workflow when working with drone: (currently not support init when drone is moving, please start when drone is stationary)

- Run the VIO package
- Run this package
- Move the drone
