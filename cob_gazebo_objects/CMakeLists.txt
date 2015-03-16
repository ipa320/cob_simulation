cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_objects)

find_package(catkin REQUIRED COMPONENTS )

catkin_package()

install(DIRECTORY Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY objects
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
