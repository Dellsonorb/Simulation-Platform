cmake_minimum_required(VERSION 3.0.2)
project(bunker_aubo_description)

find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/combined_description.test)
endif()

install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/sanitize_urdf_names.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
