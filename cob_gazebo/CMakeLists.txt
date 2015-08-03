cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS cob_controller_configuration_gazebo cob_gazebo_ros_control cob_hardware_config cob_script_server control_msgs gazebo_plugins gazebo_ros_control gazebo_ros rospy rostest trajectory_msgs
)

set(robotlist
    cob3-3
    cob3-6
    cob4-1
    raw3-1
    raw3-2
    raw3-3
    raw3-4
    raw3-5
    raw3-6
)

### TESTING ###
if(CATKIN_ENABLE_TESTING)
    foreach(robot ${robotlist})
        message("testing for robot: ${robot}")
        #rosbuild_add_roslaunch_check(launch robot:=${robot})
        #rosbuild_add_roslaunch_check(launch ROBOT=${robot})
    endforeach()
endif()

### INSTALL ###
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS ros/test/component_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/test
)

install(FILES ros/test/param_arm_test.yaml ros/test/param_tray_test.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/test
)
