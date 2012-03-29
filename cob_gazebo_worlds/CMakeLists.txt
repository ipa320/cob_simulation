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
# worlds
rosbuild_add_roslaunch_check(ros/launch/empty-world.launch ROBOT_ENV=empty-world)
rosbuild_add_roslaunch_check(ros/launch/ipa-kitchen.launch ROBOT_ENV=ipa-kitchen)
rosbuild_add_roslaunch_check(ros/launch/ipa-4th-floor.launch ROBOT_ENV=ipa-4th-floor)
rosbuild_add_roslaunch_check(ros/launch/ipa-apartment.launch ROBOT_ENV=ipa-apartment)


