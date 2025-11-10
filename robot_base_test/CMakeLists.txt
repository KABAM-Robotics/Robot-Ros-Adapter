cmake_minimum_required(VERSION 3.0.2)
project(robot_base_test)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_base_test_node src/test_node.cpp)
target_link_libraries(robot_base_test_node
  robot_base
  ${catkin_LIBRARIES}
)
