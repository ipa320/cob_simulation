# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo)

find_package(catkin REQUIRED COMPONENTS)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS gazebo_ros gazebo_plugins cob_hardware_config cob_controller_configuration_gazebo rospy rostest cob_script_server trajectory_msgs control_msgs
    DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS ros/test/component_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/test
)

install(FILES ros/test/param_arm_test.yaml ros/test/param_tray_test.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/test
)


# rostest for robots
set(robotlist
cob3-1
cob3-2
cob3-3
cob3-4
cob3-5
cob3-6
cob3-7
cob4-1
raw3-1
raw3-2
raw3-3
raw3-4
raw3-5
raw3-6
desire
)

# loop through list of robots for testing
foreach(robot ${robotlist})
	message("testing for robot: ${robot}")
	#rosbuild_add_roslaunch_check(launch robot:=${robot})
	#rosbuild_add_roslaunch_check(launch ROBOT=${robot})
endforeach(robot)
