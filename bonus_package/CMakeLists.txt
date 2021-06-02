cmake_minimum_required(VERSION 2.8.3)
project(bonus_package)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  std_msgs
  core_msgs
      pcl_conversions
    pcl_ros
)

find_package(OpenCV REQUIRED )
find_package( cv_bridge REQUIRED )

catkin_package(
  CATKIN_DEPENDS roscpp
  core_msgs
  image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(bonus_data_show_node src/data_show_bonus.cpp)
add_dependencies(bonus_data_show_node core_msgs_generate_messages_cpp)

add_executable(bonus_path_planner src/path_planner.cpp)
add_dependencies(bonus_path_planner core_msgs_generate_messages_cpp)

add_executable(data_integration_bonus src/data_integration_bonus.cpp)
add_dependencies(data_integration_bonus core_msgs_generate_messages_cpp)

add_executable(localization_bonus src/localization_bonus.cpp)
add_dependencies(localization_bonus core_msgs_generate_messages_cpp)

#add_dependencies(imu_tutorial_node imu_tutorial_generate_messages_cpp)


target_link_libraries(bonus_data_show_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

target_link_libraries(bonus_path_planner
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)
target_link_libraries(data_integration_bonus
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}
)

target_link_libraries(localization_bonus
  ${catkin_LIBRARIES}
)
