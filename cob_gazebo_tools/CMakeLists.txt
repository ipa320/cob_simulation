cmake_minimum_required(VERSION 3.0.2)
project(cob_gazebo_tools)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/move_object.py
  scripts/remove_object.py
  scripts/spawn_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
