cmake_minimum_required(VERSION 2.8.3)
project(sim_platform_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(
  PROGRAMS scripts/px4_sitl_node.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
