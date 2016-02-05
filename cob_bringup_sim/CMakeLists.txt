cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup_sim)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/remove_object.py scripts/spawn_object.py scripts/delay_robot_spawn.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
