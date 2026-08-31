cmake_minimum_required(VERSION 3.0.2)
project(sim_platform_assets)

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
