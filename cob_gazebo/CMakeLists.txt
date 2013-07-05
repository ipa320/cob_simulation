# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo)

find_package(catkin REQUIRED COMPONENTS)

find_package(OpenCV REQUIRED)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS gazebo gazebo_plugins gazebo_worlds gazebo_tools tf robot_state_publisher joint_trajectory_action single_joint_position_action actionlib actionlib_msgs move_base_msgs cob_srvs cob_description desire_description raw_description cob_gazebo_worlds cob_hardware_config cob_controller_configuration_gazebo pr2_gazebo pr2_gazebo_plugins
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

## Mark cpp header files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
