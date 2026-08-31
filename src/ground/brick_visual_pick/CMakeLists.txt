cmake_minimum_required(VERSION 3.0.2)
project(brick_visual_pick)

find_package(catkin REQUIRED COMPONENTS geometry_msgs rospy sensor_msgs std_msgs)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/brick_pose_gate.py
  scripts/visual_pick_trial_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_quality_gate.py)
  catkin_add_nosetests(test/test_pose_metrics.py)
  catkin_add_nosetests(test/test_trial_results.py)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_nosetests(test/test_matrix_report.py)
endif()
