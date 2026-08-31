cmake_minimum_required(VERSION 3.0.2)
project(bunker_navigation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs gazebo_msgs
  geometry_msgs move_base_msgs nav_msgs roscpp rospy std_msgs tf)
find_package(gazebo REQUIRED)
catkin_python_setup()
catkin_package(LIBRARIES bunker_planar_drive_plugin)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(bunker_planar_drive_plugin src/bunker_planar_drive_plugin.cpp)
target_link_libraries(bunker_planar_drive_plugin ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES})
install(TARGETS bunker_planar_drive_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/approach_pose_node.py
  scripts/approach_trial_monitor.py
  scripts/heading_gate_node.py
  scripts/navigation_trial_monitor.py
  scripts/probe_rear_obstacle_safety.py
  scripts/run_navigation_matrix.py
  scripts/run_approach_matrix.py
  scripts/send_approach_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models rviz worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_metrics.py)
  catkin_add_nosetests(test/test_approach_pose.py)
  catkin_add_nosetests(test/test_approach_metrics.py)
  catkin_add_nosetests(test/test_configuration.py)
  catkin_add_nosetests(test/test_costmap_sweep.py)
  catkin_add_nosetests(test/test_heading_gate.py)
  catkin_add_nosetests(test/test_mobile_manipulation_handoff.py)
  add_rostest(test/heading_gate_lifecycle.test)
endif()
