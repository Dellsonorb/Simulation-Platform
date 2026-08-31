cmake_minimum_required(VERSION 3.0.2)
project(prometheus_uav_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  prometheus_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(mavlink REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

catkin_package(
  INCLUDE_DIRS
  include
  vendor_upstream/common/include
  vendor_upstream/communication/include
  LIBRARIES
  uav_controller
  uav_estimator
  CATKIN_DEPENDS
  geometry_msgs
  mavros
  mavros_msgs
  nav_msgs
  prometheus_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  DEPENDS
  EIGEN3
  GEOGRAPHICLIB
  mavlink
)

include_directories(
  include
  include/Position_Controller
  vendor_upstream/common/include
  vendor_upstream/communication/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_library(
  uav_controller
  src/uav_controller.cpp
  vendor_upstream/communication/src/param_manager.cpp
)
add_dependencies(uav_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_controller ${catkin_LIBRARIES})

add_library(uav_estimator src/uav_estimator.cpp)
add_dependencies(uav_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  uav_estimator
  ${catkin_LIBRARIES}
  ${GEOGRAPHICLIB_LIBRARIES}
)

add_executable(uav_control_main src/uav_control_node.cpp)
add_dependencies(uav_control_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  uav_control_main
  uav_controller
  uav_estimator
  ${catkin_LIBRARIES}
)

add_executable(uav_command_pub utils/uav_command_pub.cpp)
add_dependencies(uav_command_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_command_pub ${catkin_LIBRARIES})

add_executable(rc_test utils/rc_test.cpp)
add_dependencies(rc_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_test ${catkin_LIBRARIES})

add_executable(joy_node utils/joy_node.cpp)
add_dependencies(joy_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_node ${catkin_LIBRARIES})

install(
  TARGETS
  uav_controller
  uav_estimator
  uav_control_main
  uav_command_pub
  rc_test
  joy_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
  vendor_upstream/common/include/
  vendor_upstream/communication/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
  launch
  launch_controller_test
  meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
