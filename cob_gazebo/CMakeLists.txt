cmake_minimum_required(VERSION 3.0.2)
project(cob_gazebo)

find_package(catkin REQUIRED)

catkin_package()

### TESTING ###
if(CATKIN_ENABLE_TESTING)
  find_package(cob_supported_robots REQUIRED)
  find_package(roslaunch REQUIRED)
  foreach(robot ${cob_supported_robots_ROBOTLIST})
    message("testing for robot: ${robot}")
    #rosbuild_add_roslaunch_check(launch robot:=${robot})
    #rosbuild_add_roslaunch_check(launch ROBOT=${robot})
  endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS ros/test/component_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ros/test/param_arm_test.yaml ros/test/param_tray_test.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/test
)
