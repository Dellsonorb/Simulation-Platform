cmake_minimum_required(VERSION 3.0.2)
project(rm4d_sim_integration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  std_msgs
  tf2_ros
  visualization_msgs
)

add_service_files(
  FILES
  PlanBasePlacement.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime rospy std_msgs tf2_ros visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/rm4d_adapter_node.py
  scripts/replay_rm4d_observation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch python_compat rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_geometry.py)
  catkin_add_nosetests(test/test_core.py)
  catkin_add_nosetests(test/test_contract.py)
endif()
