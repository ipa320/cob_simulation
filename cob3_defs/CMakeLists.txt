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

# mesh file generations
rosbuild_find_ros_package(ivcon)
rosbuild_find_ros_package(convex_decomposition)

# iterate through all the stl files to:
#  1. convert to *.iv files
#  2. generate convex decomposed .stl
file(GLOB cob3_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/common/files/meshes/*.stl)
set(cob3_gen_files "")

foreach(it ${cob3_stl_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)

  IF ( ${basename} MATCHES "_convex" )

    message("ignoring stale .._convex.stl file:",${basename})

  ELSE ( ${basename} MATCHES "_convex" )

    #convex decomposition obj files
    add_custom_command(
      OUTPUT ${basepath}/convex
      # CMake 2.4 doesn't offer the make_directory command.
      #COMMAND ${CMAKE_COMMAND} -E make_directory
      COMMAND mkdir -p
      ARGS ${basepath}/convex)

    #create obj files for convex decomposition from stl files
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}.obj
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/convex/${basename}.obj
      DEPENDS ${it} ${basepath}/convex)

    set(cob3_gen_files ${cob3_gen_files} ${basepath}/convex/${basename}.obj)

    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.obj
      COMMAND ${convex_decomposition_PACKAGE_PATH}/convex_decomposition/bin/convex_decomposition
      ARGS ${basepath}/convex/${basename}.obj -v12 -p10
      DEPENDS ${basepath}/convex/${basename}.obj ${basepath}/convex)

    set(cob3_gen_files ${cob3_gen_files} ${basepath}/convex/${basename}_convex.obj)

    #convert obj files back to stlb, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stlb
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(cob3_gen_files ${cob3_gen_files} ${basepath}/convex/${basename}_convex.stlb)

    #convert obj files back to stla, put in directory named convex
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.stla
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stla
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

    set(cob3_gen_files ${cob3_gen_files} ${basepath}/convex/${basename}_convex.stla)

    #iv files
    add_custom_command(
      OUTPUT ${basepath}/${basename}.iv
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${it} ${basepath}/${basename}.iv
      DEPENDS ${it})
    add_custom_command(
      OUTPUT ${basepath}/convex/${basename}_convex.iv
      COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
      ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.iv
      DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)
     
    set(cob3_gen_files ${cob3_gen_files} ${basepath}/${basename}.iv ${basepath}/convex/${basename}_convex.iv)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${cob3_gen_files})
