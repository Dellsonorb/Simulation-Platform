cmake_minimum_required(VERSION 3.0.2)
project(ground_manipulator_runtime)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/gazebo_grasp_confirmation.py
    scripts/spawn_ground_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/render_ground_robot.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_grasp_confirmation.py)
endif()
