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
  rostest
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

### ===== Utils header library =====
add_library(utils INTERFACE)
target_include_directories(utils INTERFACE include)

### ===== Navigation State Machine =====
add_library(navigation_state_machine
  src/nav_state_machine.cpp
)

target_link_libraries(navigation_state_machine
  ${catkin_LIBRARIES}
  pure_pursuit_tracker
)

add_dependencies(navigation_state_machine
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

### ===== Navigation node =====
add_executable(navigation_node
  src/navigation_node.cpp
)

target_link_libraries(navigation_node
  navigation_state_machine
  pure_pursuit_tracker
  stanley_tracker
  ${catkin_LIBRARIES}
  utils
)

add_dependencies(navigation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

### ===== Stanley tracker as library =====
add_library(stanley_tracker
  src/stanley_tracker.cpp
)

target_link_libraries(stanley_tracker
  ${catkin_LIBRARIES}
)

add_dependencies(navigation_state_machine
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

### ====== Pure pursuit tracker as library ======
add_library(pure_pursuit_tracker
  src/pure_pursuit_tracker.cpp
)

target_link_libraries(pure_pursuit_tracker
  ${catkin_LIBRARIES}
)

add_dependencies(pure_pursuit_tracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Add GTest cpp test target
catkin_add_gtest(test_navigation_trajectory test/test_utils.cpp)

# Link target
target_link_libraries(test_navigation_trajectory
  ${catkin_LIBRARIES}
  utils
)
