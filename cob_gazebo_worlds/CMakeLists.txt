# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(cob_gazebo_worlds)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED)


# rostest
#roslaunch_add_file_check(launch/empty.launch)

# ipa-kitchen (urdf model)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_ipa-kitchen.test)
#roslaunch_add_file_check(launch/world_urdf.launch robot_env:=ipa-kitchen)
#roslaunch_add_file_check(launch/ipa-kitchen.launch)

# ipa-apartment (urdf model)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_rostest(test/urdf_ipa-apartment.test)
#roslaunch_add_file_check(launch/world_urdf.launch robot_env:=ipa-apartment)
#roslaunch_add_file_check(launch/ipa-apartment.launch)

# ipa-factory (world model)
#roslaunch_add_file_check(launch/ipa-factory.launch)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS gazebo_ros kdl_parser tf
)

#foreach(dir materials models)
 #  install(DIRECTORY Media/${dir}/ 
  ##    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)


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
