cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rostest
)

catkin_package()

set(envlist
#  automotive-assembly-line
  empty
  ipa-apartment
  ipa-factory
  ipa-kitchen
#  ipa-office
  ipa-production-plant
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
  foreach(env ${envlist})
    message("testing for env: ${env}")
    roslaunch_add_file_check(launch/world.launch robot_env:=${env})
    roslaunch_add_file_check(launch/world.launch ROBOT_ENV=${env})
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY Media launch test urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/elevator.py scripts/tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
