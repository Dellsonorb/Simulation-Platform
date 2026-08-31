cmake_minimum_required(VERSION 3.0.2)
project(livox_laser_gazebo_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  pcl_conversions
  prometheus_msgs
  roscpp
  sensor_msgs
  tf
)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)
find_package(Protobuf REQUIRED)

find_library(
  GAZEBO_RAY_PLUGIN
  NAMES RayPlugin
  PATHS ${GAZEBO_LIBRARY_DIRS}
  NO_DEFAULT_PATH
)
if(NOT GAZEBO_RAY_PLUGIN OR NOT IS_ABSOLUTE "${GAZEBO_RAY_PLUGIN}")
  message(FATAL_ERROR "Gazebo RayPlugin was not found at an absolute host library path")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES livox_laser_gazebo_plugins
  CATKIN_DEPENDS
  gazebo_dev
  pcl_conversions
  prometheus_msgs
  roscpp
  sensor_msgs
  tf
  DEPENDS PCL
)

add_library(
  livox_laser_gazebo_plugins
  SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)
target_include_directories(
  livox_laser_gazebo_plugins
  PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_dependencies(livox_laser_gazebo_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  livox_laser_gazebo_plugins
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${PCL_LIBRARIES}
  protobuf::libprotobuf
  ${GAZEBO_RAY_PLUGIN}
)

install(
  TARGETS livox_laser_gazebo_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/livox_laser_simulation/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livox_laser_simulation
)
