cmake_minimum_required(VERSION 3.0.2)
project(line_detection)
find_package(catkin REQUIRED COMPONENTS
  core_msgs
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  core_msgs
  image_transport
  message_generation
)

find_package( OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES line_detection
  CATKIN_DEPENDS core_msgs
  image_transport
#  message_generation
  roscpp
#  sensor_msgs
#  std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(line_detect_node src/line_detect.cpp)
add_dependencies(line_detect_node core_msgs_generate_messages_cpp)

target_link_libraries(line_detect_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)


