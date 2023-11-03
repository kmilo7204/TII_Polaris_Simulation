cmake_minimum_required(VERSION 3.0.2)
project(path_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  nav_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(path_generator src/path_generator.cpp)
target_link_libraries(path_generator ${catkin_LIBRARIES})

