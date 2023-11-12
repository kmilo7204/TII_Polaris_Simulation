cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  nav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# For tc_simple_world_plugin plugin
add_library(simple_world_plugin src/simple_world_plugin.cpp)
target_link_libraries(simple_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(path_drawer_plugin src/path_drawer_plugin.cpp)
target_link_libraries(path_drawer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
