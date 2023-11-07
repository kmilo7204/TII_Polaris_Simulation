cmake_minimum_required(VERSION 3.0.2)
project(stanley)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(stanley src/stanley.cpp)
target_link_libraries(stanley ${catkin_LIBRARIES})
