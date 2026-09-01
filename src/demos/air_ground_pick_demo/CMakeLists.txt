cmake_minimum_required(VERSION 3.0.2)
project(air_ground_pick_demo)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
