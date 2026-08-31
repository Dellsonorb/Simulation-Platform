cmake_minimum_required(VERSION 3.0.2)
project(brick_aerial_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  message_filters
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  visualization_msgs
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/aerial_brick_pose_node.py
  scripts/aerial_viewpoint_mission.py
  scripts/aerial_viewpoint_mission_multiview.py
  scripts/m1_validation_monitor.py
  scripts/world_tf_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/run_m1_demo.bash
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
