cmake_minimum_required(VERSION 3.0.2)
project(ground_pick_orchestrator)

find_package(catkin REQUIRED COMPONENTS rospy)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/ground_pick_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_mission_state.py)
  catkin_add_nosetests(test/test_safety_guards.py)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_nosetests(test/test_observation_planning.py)
endif()
