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

# find needed paths
rosbuild_find_ros_package(cob3_defs)
rosbuild_find_ros_package(cob3_ogre)
rosbuild_find_ros_package(ogre)

set(cob3_gen_files "")

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB_RECURSE cob3_defs_stl_files RELATIVE ${cob3_defs_PACKAGE_PATH}/common/files/meshes ${cob3_defs_PACKAGE_PATH}/common/files/meshes/*.stl ${cob3_defs_PACKAGE_PATH}/common/files/meshes/convex/*.stlb)

set(cob3_defs_out_path ${CMAKE_CURRENT_SOURCE_DIR}/common/files/models)
MAKE_DIRECTORY(${cob3_defs_out_path})

foreach(it ${cob3_defs_stl_files})
  get_filename_component(relative_path ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  # create subdirectory
  add_custom_command(
    OUTPUT ${cob3_defs_out_path}/${relative_path}
    # CMake 2.4 doesn't offer the make_directory command.
    #COMMAND ${CMAKE_COMMAND} -E make_directory
    COMMAND mkdir -p
    ARGS ${cob3_defs_out_path}/${relative_path})

  # convert to ogre files
  add_custom_command(
    OUTPUT ${cob3_defs_out_path}/${relative_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${cob3_defs_PACKAGE_PATH}/common/files/meshes/${it} ${cob3_defs_out_path}/${relative_path}/${basename}.mesh
    DEPENDS ${cob3_defs_out_path}/${relative_path} ${cob3_defs_PACKAGE_PATH}/common/files/meshes/${it})
  
  set(cob3_gen_files ${cob3_gen_files} ${cob3_defs_out_path}/${relative_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${cob3_gen_files})
