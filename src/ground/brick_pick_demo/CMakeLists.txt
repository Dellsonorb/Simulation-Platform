cmake_minimum_required(VERSION 3.0.2)
project(brick_pick_demo)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS actionlib control_msgs gazebo_msgs geometry_msgs
  moveit_ros_planning_interface roscpp sensor_msgs std_srvs tf2
  tf2_geometry_msgs tf2_ros trajectory_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES brick_geometry grasp_geometry
  gripper_feasibility pose_frames trajectory_timing
  CATKIN_DEPENDS gazebo_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros
  trajectory_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(brick_geometry src/brick_geometry.cpp)
target_link_libraries(brick_geometry ${catkin_LIBRARIES})
add_library(grasp_geometry src/grasp_geometry.cpp)
target_link_libraries(grasp_geometry brick_geometry ${catkin_LIBRARIES})
add_library(gripper_feasibility src/gripper_feasibility.cpp)
target_link_libraries(gripper_feasibility grasp_geometry ${catkin_LIBRARIES})
add_library(trajectory_timing src/trajectory_timing.cpp)
target_link_libraries(trajectory_timing ${catkin_LIBRARIES})
add_library(pose_frames src/pose_frames.cpp)
target_link_libraries(pose_frames grasp_geometry ${catkin_LIBRARIES})
add_executable(brick_pick_node src/brick_pick_node.cpp)
target_link_libraries(brick_pick_node grasp_geometry gripper_feasibility
  pose_frames trajectory_timing
  ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_grasp_geometry test/test_grasp_geometry.cpp)
  if(TARGET test_grasp_geometry)
    target_link_libraries(test_grasp_geometry grasp_geometry ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_trajectory_timing test/test_trajectory_timing.cpp)
  if(TARGET test_trajectory_timing)
    target_link_libraries(test_trajectory_timing trajectory_timing
      ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_gripper_feasibility test/test_gripper_feasibility.cpp)
  if(TARGET test_gripper_feasibility)
    target_link_libraries(test_gripper_feasibility gripper_feasibility
      grasp_geometry ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_pose_frames test/test_pose_frames.cpp)
  if(TARGET test_pose_frames)
    target_link_libraries(test_pose_frames pose_frames grasp_geometry
      ${catkin_LIBRARIES})
  endif()
  add_rostest(test/full_pick.test)
endif()
catkin_install_python(PROGRAMS scripts/manual_brick_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS brick_geometry grasp_geometry gripper_feasibility pose_frames
  trajectory_timing brick_pick_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
