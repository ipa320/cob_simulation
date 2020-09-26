cmake_minimum_required(VERSION 3.0.2)
project(cob_bringup_sim)

find_package(catkin REQUIRED)

catkin_package()

### TEST ###
if(CATKIN_ENABLE_TESTING)
  find_package(cob_default_env_config REQUIRED)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    foreach(env ${cob_default_env_config_ENVLIST})
      message("testing for robot: ${robot} in env: ${env}")
      #roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env})
      roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})
    endforeach()
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
