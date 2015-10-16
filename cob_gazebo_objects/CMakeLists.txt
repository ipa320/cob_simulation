cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_objects)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

### INSTALL ###
install(DIRECTORY Media objects test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)