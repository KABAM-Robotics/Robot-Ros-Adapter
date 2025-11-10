cmake_minimum_required(VERSION 3.0.2)
project(robot_base)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)
find_package(CURL REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Header-only interface
add_library(robot_base INTERFACE)
target_include_directories(robot_base INTERFACE include)
target_link_libraries(robot_base INTERFACE ${catkin_LIBRARIES} CURL::libcurl)
