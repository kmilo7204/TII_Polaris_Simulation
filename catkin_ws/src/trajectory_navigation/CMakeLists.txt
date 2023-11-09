cmake_minimum_required(VERSION 3.0.2)
project(trajectory_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

# catkin_package()

# include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### Navigation State Machine
add_library(navigation_state_machine
  src/nav_state_machine.cpp
)

target_link_libraries(navigation_state_machine
  ${catkin_LIBRARIES}
)

add_dependencies(navigation_state_machine
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS}
)

### Navigation node
add_executable(navigation_node
  src/navigation_node.cpp
)

target_link_libraries(navigation_node
  navigation_state_machine
  ${catkin_LIBRARIES}
)

add_dependencies(navigation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
