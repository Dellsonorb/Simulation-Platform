cmake_minimum_required(VERSION 3.0.2)
project(air_ground_pick_demo)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/air_ground_pick_demo.py
  scripts/red_target_observer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
