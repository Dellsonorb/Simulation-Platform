cmake_minimum_required(VERSION 3.0.2)
project(rm4d_sim_integration)

find_package(catkin REQUIRED)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_geometry.py)
  catkin_add_nosetests(test/test_core.py)
endif()
