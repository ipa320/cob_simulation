cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_worlds)

find_package(catkin REQUIRED COMPONENTS roslaunch rostest)

catkin_package(
    CATKIN_DEPENDS gazebo_ros kdl_parser tf
)


# rostest
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



install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS src/elevator.py src/tf_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
