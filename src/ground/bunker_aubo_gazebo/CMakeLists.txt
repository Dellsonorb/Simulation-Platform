cmake_minimum_required(VERSION 3.0.2)
project(bunker_aubo_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp std_srvs)
find_package(gazebo REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES ag95_effort_coupling ag95_effort_coupling_plugin attachment_guard guarded_brick_attachment CATKIN_DEPENDS gazebo_ros roscpp std_srvs)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(attachment_guard src/attachment_guard.cpp)
target_link_libraries(attachment_guard ${catkin_LIBRARIES})
add_library(ag95_effort_coupling src/ag95_effort_coupling.cpp)
target_link_libraries(ag95_effort_coupling ${catkin_LIBRARIES})
add_library(ag95_effort_coupling_plugin src/ag95_effort_coupling_plugin.cpp)
target_link_libraries(ag95_effort_coupling_plugin ag95_effort_coupling
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_library(guarded_brick_attachment src/guarded_brick_attachment.cpp)
target_link_libraries(guarded_brick_attachment attachment_guard ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_attachment_guard test/test_attachment_guard.cpp)
  if(TARGET test_attachment_guard)
    target_link_libraries(test_attachment_guard attachment_guard ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_ag95_effort_coupling
    test/test_ag95_effort_coupling.cpp)
  if(TARGET test_ag95_effort_coupling)
    target_link_libraries(test_ag95_effort_coupling ag95_effort_coupling
      ${catkin_LIBRARIES})
  endif()
  add_rostest(test/bunker_only.test)
  add_rostest(test/aubo_only.test)
  add_rostest(test/ag95_only.test)
  add_rostest(test/ag95_effort_runtime.test)
  add_rostest(test/combined_robot.test)
  add_rostest(test/brick_world.test)
  add_rostest(test/camera_demo_config.test)
endif()

install(DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/move_to_camera_observation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ag95_effort_coupling ag95_effort_coupling_plugin
  attachment_guard guarded_brick_attachment
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
