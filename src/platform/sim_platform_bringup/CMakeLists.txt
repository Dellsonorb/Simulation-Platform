cmake_minimum_required(VERSION 3.0.2)
project(sim_platform_bringup)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_p450_launch_contract.py)
  catkin_add_nosetests(test/test_px4_sitl_node.py)
endif()

install(
  PROGRAMS scripts/px4_sitl_node.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
