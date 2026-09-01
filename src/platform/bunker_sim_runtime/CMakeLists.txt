cmake_minimum_required(VERSION 3.0.2)
project(bunker_sim_runtime)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  geometry_msgs
  nav_msgs
  roscpp
  tf2_ros
)
find_package(gazebo REQUIRED)
catkin_python_setup()
catkin_package(
  LIBRARIES bunker_planar_move_plugin
  CATKIN_DEPENDS gazebo_dev geometry_msgs nav_msgs roscpp tf2_ros
)

add_library(bunker_planar_move_plugin SHARED
  src/bunker_planar_move_plugin.cpp
)
target_include_directories(bunker_planar_move_plugin PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(bunker_planar_move_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_velocity_guard.py)
  catkin_add_nosetests(test/test_renderer.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  add_rostest(test/tf_prefix_contract.test)
endif()

catkin_install_python(
  PROGRAMS scripts/velocity_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS bunker_planar_move_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/render_bunker_runtime.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
