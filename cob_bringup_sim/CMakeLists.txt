cmake_minimum_required(VERSION 2.8.3)
project(cob_bringup_sim)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS cob_default_env_config cob_default_robot_config cob_gazebo_worlds cob_gazebo gazebo_msgs gazebo_ros geometry_msgs rospy tf
)

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/remove_object.py scripts/spawn_object.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
