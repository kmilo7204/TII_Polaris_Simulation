cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pure_pursuit src/tracker.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES})
