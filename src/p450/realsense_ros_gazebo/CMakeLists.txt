cmake_minimum_required(VERSION 2.8.3)
project(realsense_ros_gazebo)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  gazebo_dev
  gazebo_ros
  image_transport
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES realsense_gazebo_plugin
  CATKIN_DEPENDS
  camera_info_manager
  gazebo_dev
  image_transport
  roscpp
  sensor_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(
  realsense_gazebo_plugin
  src/RealSensePlugin.cpp
  src/gazebo_ros_realsense.cpp
)
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  realsense_gazebo_plugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS realsense_gazebo_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/realsense_gazebo_plugin/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/realsense_gazebo_plugin
)

install(
  DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
