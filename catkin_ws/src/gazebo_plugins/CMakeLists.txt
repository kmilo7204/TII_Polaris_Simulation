cmake_minimum_required(VERSION 3.0.2)
project(gazebo_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  nav_msgs
)

# Search gazebo package, required for the plugin
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(path_drawer_plugin src/path_drawer_plugin.cpp)
target_link_libraries(path_drawer_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
