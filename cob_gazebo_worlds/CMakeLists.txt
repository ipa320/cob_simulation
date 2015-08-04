cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

catkin_package(
    CATKIN_DEPENDS controller_manager gazebo_msgs gazebo_ros_control gazebo_ros joint_state_controller joint_state_publisher kdl_parser position_controllers robot_state_publisher rospy std_msgs tf velocity_controllers xacro
)

### TESTING ###
if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch/empty.launch)

  # ipa-kitchen (urdf model)
  add_rostest(test/urdf_ipa-kitchen.test)
  #roslaunch_add_file_check(launch/world_urdf.launch robot_env:=ipa-kitchen)
  roslaunch_add_file_check(launch/ipa-kitchen.launch)

  # ipa-apartment (urdf model)
  add_rostest(test/urdf_ipa-apartment.test)
  #roslaunch_add_file_check(launch/world_urdf.launch robot_env:=ipa-apartment)
  roslaunch_add_file_check(launch/ipa-apartment.launch)

  # ipa-factory (world model)
  roslaunch_add_file_check(launch/ipa-factory.launch)
endif()

### INSTALL ###
install(DIRECTORY Media launch test urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/elevator.py scripts/tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
