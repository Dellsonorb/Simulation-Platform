cmake_minimum_required(VERSION 3.0.2)
project(brick_rgbd_perception)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/brick_rgbd_node.py
  scripts/brick_pose_estimator.py
  scripts/validate_pose_robustness.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_core.py)
  catkin_add_nosetests(test/test_brick_geometry.py)
  catkin_add_nosetests(test/test_validation.py)
  add_rostest(test/perception_runtime.test)
  add_rostest(test/pose_estimation_runtime.test)
endif()
