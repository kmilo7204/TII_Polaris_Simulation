cmake_minimum_required(VERSION 3.0.2)
project(path_generator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  rostest
  nav_msgs
  std_srvs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

### ===== Path Generator =====
add_library(path_generator
  src/path_generator.cpp
)

target_link_libraries(path_generator
  ${catkin_LIBRARIES}
)

add_dependencies(path_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

### ===== Path Generator node =====
add_executable(path_generator_node
  src/path_generator_node.cpp
)

target_link_libraries(path_generator_node
  path_generator
  ${catkin_LIBRARIES}
)

add_dependencies(path_generator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

### ===== Build tests =====
catkin_add_gtest(test_path_generator
  test/test_path_generator.cpp
  src/path_generator.cpp
)

# Link target
target_link_libraries(test_path_generator
  ${catkin_LIBRARIES}
)
