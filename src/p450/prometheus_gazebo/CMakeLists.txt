cmake_minimum_required(VERSION 2.8.3)
project(prometheus_gazebo)

find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_p450_sensor_profiles.py)
endif()

catkin_install_python(
  PROGRAMS scripts/jinja_gen.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  config
  gazebo_models
  gazebo_worlds
  launch_basic
  launch_fmt
  launch_test
  launch_uav_with_sensor
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
