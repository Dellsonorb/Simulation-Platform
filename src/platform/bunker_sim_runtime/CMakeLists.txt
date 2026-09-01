cmake_minimum_required(VERSION 3.0.2)
project(bunker_sim_runtime)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_contracts.py)
  catkin_add_nosetests(test/test_sim_time.py)
  catkin_add_nosetests(test/test_velocity_guard.py)
  catkin_add_nosetests(test/test_spawn_bunker_preflight.py)
  catkin_add_nosetests(test/test_renderer.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  add_rostest(test/tf_prefix_contract.test)
endif()

catkin_install_python(
  PROGRAMS scripts/velocity_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/render_bunker_runtime.py
    scripts/spawn_bunker_preflight.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
