# TII_Polaris_Simulation

## Container
- Build
docker build -t ros-gazebo-nvidia-tii:20.04 .

- Run
    ./run.sh ${PATH_TO_REPO}/gazebo_workshop/Project_1-Building/ ros-gazebo-nvidia-tii:20.04

    ./run.sh /home/ekumen/Camilo_Repos/TII_Polaris_Simulation/catkin_ws/src ros-gazebo-nvidia-tii:20.04


roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"

## Requirements
1. Development of Path-Tracking Algorithm
  a. Develop two distinct path-tracking controllers in C++ that have the capability to follow a specified path.
    i. Input data for your controller will include:
      1. Path ROS message, with GPS coordinates nav_msgs/Path Documentation
    2. Odometry ROS message nav_msgs/Odometry Documentation
  b. The system should allow switching between the developed path-tracking algorithms or adding a new algorithm with minimal code
  changes. It is not necessarily required for the algorithm to switch during run time.

- I can create a ROS package that contains this implementation
  - It should contains a set of standard or base methods, so I can create an interface for this

  Controllers -> package pure_pursuit and package stanley