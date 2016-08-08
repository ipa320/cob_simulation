cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup_sim)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

set(robotlist
  cob3-2
  cob3-6
  cob3-9
  cob4-1
  cob4-2
  cob4-3
  cob4-4
  cob4-6
  raw3-1
  raw3-2
  raw3-3
  raw3-4
  raw3-5
  raw3-6
)

set(envlist
  automotive-assembly-line
  empty
  ipa-apartment
  ipa-kitchen
  ipa-office  
  ipa-production-plant
)

### TEST ###
if(CATKIN_ENABLE_TESTING)
  foreach(robot ${robotlist})
    foreach(env ${envlist})
      message("testing for robot: ${robot} in env: ${env}")
      roslaunch_add_file_check(launch robot:=${robot} robot_env:=${env})
      roslaunch_add_file_check(launch ROBOT=${robot} ROBOT_ENV=${env})
    endforeach()
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/remove_object.py scripts/spawn_object.py scripts/move_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
