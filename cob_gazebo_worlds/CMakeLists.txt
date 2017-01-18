cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()

### TEST ###
if(CATKIN_ENABLE_TESTING)
  find_package(cob_default_env_config REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest(test/urdf_worlds.test)
  foreach(env ${cob_default_env_config_ENVLIST})
    message("testing for env: ${env}")
    #roslaunch_add_file_check(launch/world.launch robot_env:=${env})
    roslaunch_add_file_check(launch/world.launch ROBOT_ENV=${env})
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY Media launch test urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/elevator.py scripts/tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
