cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

# rostest
rosbuild_add_roslaunch_check(launch/empty.launch)

# ipa-kitchen (urdf model)
rosbuild_add_rostest(test/urdf_ipa-kitchen.test)
rosbuild_add_roslaunch_check(launch/world_urdf.launch robot_env:=ipa-kitchen)
rosbuild_add_roslaunch_check(launch/ipa-kitchen.launch)

# ipa-apartment (urdf model)
rosbuild_add_rostest(test/urdf_ipa-apartment.test)
rosbuild_add_roslaunch_check(launch/world_urdf.launch robot_env:=ipa-apartment)
rosbuild_add_roslaunch_check(launch/ipa-apartment.launch)

# ipa-factory (world model)
rosbuild_add_roslaunch_check(launch/ipa-factory.launch)
