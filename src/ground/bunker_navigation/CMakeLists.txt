cmake_minimum_required(VERSION 3.0.2)
project(bunker_navigation)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs geometry_msgs
  move_base_msgs nav_msgs rospy std_msgs std_srvs tf visualization_msgs)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs
  move_base_msgs nav_msgs rospy std_msgs std_srvs tf visualization_msgs)

catkin_install_python(PROGRAMS
  scripts/approach_pose_node.py
  scripts/ground_stop_server.py
  scripts/heading_gate_node.py
  scripts/send_approach_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_approach_pose.py)
  catkin_add_nosetests(test/test_common_navigation.py)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_nosetests(test/test_costmap_sweep.py)
  catkin_add_nosetests(test/test_heading_gate.py)
  catkin_add_nosetests(test/test_mobile_manipulation_handoff.py)
  add_rostest(test/heading_gate_lifecycle.test)
endif()
