cmake_minimum_required(VERSION 3.0.2)
project(ground_manipulator_runtime)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS gazebo_dev gazebo_ros_control pluginlib roscpp)
find_package(gazebo REQUIRED)
catkin_python_setup()
catkin_package(
  LIBRARIES ground_integrated_velocity_robot_hw_sim
  CATKIN_DEPENDS gazebo_dev gazebo_ros_control pluginlib roscpp
)

add_library(ground_integrated_velocity_robot_hw_sim SHARED
  src/integrated_velocity_robot_hw_sim.cpp
)
target_include_directories(ground_integrated_velocity_robot_hw_sim PRIVATE
  ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(ground_integrated_velocity_robot_hw_sim
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)

install(TARGETS ground_integrated_velocity_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/gazebo_grasp_confirmation.py
    scripts/spawn_ground_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/render_ground_robot.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY config launch urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_grasp_confirmation.py)
endif()
