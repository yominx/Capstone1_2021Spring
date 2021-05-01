cmake_minimum_required(VERSION 2.8.3)
project(ball_harvesting)
add_compile_options(-std=c++11)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  message_generation
)

find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  DEPENDS system_lib
)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  #include
)
add_executable(ball_harvest_node src/ball_harvest.cpp)
add_dependencies(ball_harvest_node core_msgs_generate_messages_cpp)

target_link_libraries(ball_harvest_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
