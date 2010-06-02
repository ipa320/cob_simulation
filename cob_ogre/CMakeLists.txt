cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# find needed paths
rosbuild_find_ros_package(cob_description)
rosbuild_find_ros_package(cob_ogre)
rosbuild_find_ros_package(ogre)

set(cob_gen_files "")

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB_RECURSE cob_description_stl_files RELATIVE ${cob_description_PACKAGE_PATH}/ros/meshes ${cob_description_PACKAGE_PATH}/ros/meshes/*/*.stl ${cob_description_PACKAGE_PATH}/ros/meshes/*/convex/*.stlb)

set(cob_description_out_path ${CMAKE_CURRENT_SOURCE_DIR}/common/files/models)

MAKE_DIRECTORY(${cob_description_out_path})

foreach(it ${cob_description_stl_files})
  get_filename_component(relative_path ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  # create subdirectory
  add_custom_command(
    OUTPUT ${cob_description_out_path}/${relative_path}
    # CMake 2.4 doesn't offer the make_directory command.
    #COMMAND ${CMAKE_COMMAND} -E make_directory
    COMMAND mkdir -p
    ARGS ${cob_description_out_path}/${relative_path})

  # convert to ogre files
  add_custom_command(
    OUTPUT ${cob_description_out_path}/${relative_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${cob_description_PACKAGE_PATH}/ros/meshes/${it} ${cob_description_out_path}/${relative_path}/${basename}.mesh
    DEPENDS ${cob_description_out_path}/${relative_path} ${cob_description_PACKAGE_PATH}/ros/meshes/${it})
  
  set(cob_gen_files ${cob_gen_files} ${cob_description_out_path}/${relative_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${cob_gen_files})
