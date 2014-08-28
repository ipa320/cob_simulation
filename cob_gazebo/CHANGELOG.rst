^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2014-08-28)
------------------
* cleaning up
* New maintainer
* no OpenCV required in cob_gazebo
* Contributors: ipa-fxm, ipa-nhg

0.5.1 (2014-03-21)
------------------
* merge with groovy_dev
* setup tests
* fix launch file
* New structure
* merge with groovy_dev
* Adapt to the new cob_controller_configuration_gazebo structure
* removed a lot of code related to packages not available in hydro anymore
* change dependency from gazebo to gazebo_ros
* remove duplication
* changed to new gazebo_ros bridge
* installation stuff
* add roslaunch arg
* Initial catkinization.
* moved gazebo service to cob_controller_configuration_gazebo
* adding additional launch file parameters for gazebo simulation
* removed deprecated coloured_point_cloud nodes and libs + cleaning up
* unpause gazebo after spawning robot
* remove cob_ogre - not needed any more
* Removed cob_ogre dependency
* removed dependency to deleted cob_ogre package
* add cam3d throttle, renamed some topics
* added relay for color image
* removed pointcloud1 converter
* opt env for ROBOT
* adapted gazebo_services to FollowJointTrajectoryAction
* spawn cob a little lower
* cleanup launch files and substitute env through arg
* plugin file
* workaround for image_transport bug
* merge
* forget removing a line
* fuerte migration
* adapt roslaunch tests
* fix for raw
* moved cob_controller_config_gazebo to cob_robots and changed some minor things to support new structure
* changed manifest description
* Service /base_controller/stop able in simulation
* merge with ipa320
* merge with ipa320
* merge
* add cob3-4 tests
* Merge pull request `#4 <https://github.com/ipa320/cob_simulation/issues/4>`_ from ipa-goa/master
  connect callback for point cloud converter
* added connect and disconnect cb to converter
* added connect and disconnect callback
* removed point cloud conversion for faster simulation
* merge with ipa320
* remove env test
* fixed topic name
* Merge remote-tracking branch 'origin-ipa-fmw/master' into automerge
* Merge branch 'master' of github.com:ipa-fmw/cob_simulation into review-ipa-fmw
* add new gazebo services
* fix test
* reduced dependencies
* new directory structure in cob_description
* changes for creating colored point cloud
* merge
* added node to generate colored point cloud for kinect, removed obsolete point cloud fix
* added launch tests for simulation stack
* integrate tactile sensors in gazebo
* start the simulated tactile sensors with the simulation
* icob for simulation
* corrected the swissranger topics to the unified naming scheme
* worked on base controller for simulation
* modified launch file to include pointcloud fix
* node for fixing pointclouds from gazebo block laser
* tray parameters for component_test
* added point_cloud_converter for PointCloud2 in simulation
* modified
* parameter file
* fix sdh
* beautifying
* single arm and arm with sdh simulation running
* update component test
* modified unittest for components
* gazebo services for desire
* first gazebo test
* gazebo testing
* merge
* restructure urdf files and launch files for simulation
* changed launch files for single components
* bugfix
* changed launch file structure for bringup
* preparing release
* debugged service interface for gazebo
* service timeout for base and removed cob_defs from showdeps
* cleanup in simulation and common
* changed to spawn_model
* services for gazebo simulation
* services for gazebo simulation
* moved ekf domo publisher to nav; update positions for new urdf trafos; moved controller_manager to cob_controller_configuration_gazebo
* renamed manifest description
* deactivated cartesian interface in launch files
* populate ipa kitchen
* preparing for grasp script
* update documentation
* update dashboard
* cartesian arm movement is working with script_server
* merge with aub
* dual arm cob3 simulation and modified controllers for schunk simulation
* lbr.launch file added
* improvements of lbr simulation
* added lbr to simulation
* small fixes for simulation
* updated simulation files
* cleanup in cob_simulation
* missing files for simulation
* merge
* new simulation interfaces
* small fix
* missing bringup file
* new launch file for no contollers
* big changes to simulation structure
* new launch files for simulation
* modified manifests for documentation
* merge with master
* changed cob3_defs to cob_def in xacro, launch and urdf files
* adapt launch file to new packages names
* renamed packages to cob_ convention
* Contributors: Alexander Bubeck, Felix Messmer, Frederik Hegger, Georg Arbeiter, Richard Bormann, Sven Schneider, abubeck, brics, brudder, fmw-jiehou, fmw-jk, ipa-fmw, ipa-fxm, ipa-goa, ipa-nhg, ipa-uhr-fm, nhg-ipa
