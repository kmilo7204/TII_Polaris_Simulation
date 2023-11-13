# TII_Polaris_Simulation
Challenge solution

## Container
The solution includes a container with Ubuntu 20.04, Nvidia capabilities and ROS Noetic, desktop full.

### Build
Build the container with the following command:

```bash
docker build -t ros-gazebo-nvidia-tii:20.04 .
```

### Run
Run the container with the provided bash script `run.sh`

```bash
  ./run.sh /home/ekumen/Camilo_Repos/TII_Polaris_Simulation/catkin_ws/src
```

## Robot stack


roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"

Generate the path
rosservice call /generate_path


roslaunch trajectory_navigation navigation.launch

# Tests
catkin_make run_tests


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