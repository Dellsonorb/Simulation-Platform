cmake_minimum_required(VERSION 3.0.2)
project(p450_flight_facade)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  prometheus_msgs
  robot_runtime_interfaces
  rospy
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs nav_msgs prometheus_msgs
    robot_runtime_interfaces rospy tf2_geometry_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/p450_flight_facade.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
