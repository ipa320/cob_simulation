^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_gazebo_worlds
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2014-09-22)
------------------
* 1 = true
* fix bumper plugin
* Contributors: ipa-fxm

0.6.0 (2014-09-18)
------------------
* Merge pull request `#65 <https://github.com/ipa320/cob_simulation/issues/65>`_ from ipa320/hydro_dev
  bringin updates from hydro_dev
* Merge pull request `#64 <https://github.com/ipa320/cob_simulation/issues/64>`_ from ipa320/hydro_release_candidate
  Hydro release candidate
* 0.5.2
* update changelog
* Contributors: Florian Weisshardt

0.5.2 (2014-08-28)
------------------
* change to latest transmission format
* removed physics and added floor collisions
* Set GAZEBO_MODEL_PATH
* small changes to get simulation running
* cleaning up
* Merge pull request `#54 <https://github.com/ipa320/cob_simulation/issues/54>`_ from abubeck/hydro_dev
  change physic setings to improve simulation performance
* New maintainer
* Merge branch 'hydro_dev' of github.com:abubeck/cob_simulation into hydro_dev
* Moved floor pose
* change physic setings to improve simulation performance
* Contributors: Alexander Bubeck, Felix Messmer, Florian Weisshardt, abubeck, ipa-fxm, ipa-nhg

0.5.1 (2014-03-21)
------------------
* install tags
* bring in groovy update
* New structure
* Define camera pose
* Define light
* change dependency from gazebo to gazebo_ros
* :q
* fix ground plane for all environments + restructuring
* update xmlns + beautifying
* enable paused mode again
* changed for simulation fixes
* merged
* changes for textures
* Merge pull request `#38 <https://github.com/ipa320/cob_simulation/issues/38>`_ from ipa-nhg/hydro_dev
  Hydro dev
* Corrected xacro warning in hydro.
* Created specific empty world for ipa environments
* Created empty world launch file
* Created specific empty world for ipa environments
* Changed name medication_prospan to medicine_prospan and moved the default camera position
* Created new objects
* remove debug tag
* New wall textures and floor for ipa-apartment environment
* changes for new gazebo in hydro
* installation stuff
* removed dependency on old gazebo_plugin.msgs
* cleanup deps
* Addapted .dae files for new gazebo version
* Initial catkinization without rostest stuff
* added prace logo texture
* adding additional launch file parameters for gazebo simulation
* fixing and cleaning up files
* fixing ipa-factory
* adding description for ipa-factory for sdf 1.3
* removing obsolete files
* removing obsolete files
* removing ipa-maze
* converted deprecated empty.world to latest sdf format 1.3 - by using 'save world as' in gazebo
* fixed copy paste error
* some cleaning up
* fix environment descriptions according to new gazebo format
* all env working except ipa-factory
* started to cleanup envs
* New empty_world version for groovy
* ogre_tools is now a system dependency - remove depend package
* New groovy-rosbuild branch
* fixed gazebo worlds launch file
* fixed path in launch file
* Renamed colors
* :q
* Renamed the materials
* Defined new colors
* Defined new colors
* merge
* update deps
* renamed /joint_state for elevator joitns
* added missed dependencies
* The elevator controller is not necessary
* Fixed some walls in ipa_office
* New simulation colors
* add map to rviz
* add state publisher
* rearagen launch files for display world in rviz
* Merge remote branch 'origin-ipa-nhg/master' into automerge
* Merge branch 'master' of github.com:ipa-nhg/cob_environments
* Merge remote-tracking branch 'origin-ipa320/master'
* Defined new colors
* Defined new colors
* Enabled display the worlds in Rviz
* Simulation model for 3rd flor and visualiced in rviz
* comment fuerte stuff out
* Fuerte migration cob_gazebo_worlds
* adapted raw-exhibition launch file to new env arg structure
* renamed and adapted file to new structure
* removed unused ENV_MODE macro and deleted related files
* new file for respawning the world
* fixed ENV_MODE
* add tests for launch arg
* fix for png world
* introduce arg robot_env instead of env ROBOT_ENV
* add ipa-4th-floor again
* reanimate ipa-4th-floor png map
* changed to new structure
* added Industriestrasse to simulation environments
* Fixed test errors
* Renamed launch files
* Added header to python code
* New structure in cob_gazebo_worlds package
* Merge remote branch 'origin-ipa-nhg/master' into automerge
* Added elevators in ipa-apartment world
* Added elevators in ipa-apartment
* Added elevators in ipa-apartment
* changed file permissions
* groud plane with mesh
* fixed ground plane
* use empty world from cob_gazebo_worlds again
* removed -s argument
* modified gazebo world to fit new navi map
* Moved load parameters for objects in simulation to cob_gazebo_objects
* add urdf tests for world and objects
* Load the parameters of the locations of the objects in the environment launch file
* added gazebo simulation for raw-exhibition
* Added ipa-apartment in CMakeLists.txt
* new ipa-apartment worldcob_gazebo_objects/config/ipa-apartment/object_locations.yaml
* new ipa-apartment environment
* minor changes befor merge
* Renamed ipa-playground, before was ipa-apartment
* Moved cob_gazebo_worlds from cob_simulation to cob_environments
* Added missing urdf models
* Fixed an error in CMakeLists.txt
* Coordinate system of milk is at the bottom and corresponds to object detection. Coordinate system of jodsalz, zwieback, krauter, tomaten_suppe are at the bottom, but not yet tested to fit to object detection. All launch files have been adapted to start the new urdf models now
* Fixed jumping milk box bug
* Moved cob_gazebo_worlds from cob_simulation to cob_environments
* New version of cob_gazebo_worlds, deleted all the objects and furnitures, and re-localization in cob_environments
* Added urdf model for milk box to fix coordinate system#
* Create urdf file for milk_box model to fix coordinate system
* gazebo models of checkerboards with sizes 9x6 and 4x3 added
* modified milk box gazebo coordinate system to fit with iv model
* empty world with wall
* empty world with one wall
* added chair model
* merge with ipa320
* table as .model file
* update manifest
* fix jodsalz model
* new position of the milk box
* rename milk_box
* maybe a performance improvement
* added launch tests for simulation stack
* added image_proc and changed furniture color
* new objects: table, cabinet and bookcase
* new objects table, cabinet and bookcase
* changed position
* switched to electric
* new world ipa-apartment
* JSF
* merge
* adapted initial position
* wimicare project: simulation test with objects
* backup
* wimicare project, added person objects and table
* inserted the original ipa logo in ipa-kitchen
* changed IFA to IPA
* added a door in the kitchen for fun
* corrected milk_box properties
* merge
* revert freezer door
* Merge branch 'review-aub'
* changed name of object_learning_platform
* small changes to ipa-kitchen
* object_learning_platform plus small changes in ipa-kitchen
* changed to optenv for ENV_MODE
* merged
* merged with freezer door
* new version of kitchen with freezer door
* introducing ENV_MODE=-simple for simple-kitchen
* introducing simple-kitchen
* speed up simulation by modifying textures in kitchen-world
* fixed texture rendering problem by changing image sizes
* merged
* reordered kitchen objects
* change camera view and floor color
* modified ipa-kitchen and created seperate table
* modifications to world
* added cabinet and couch
* added textures
* srs logo
* new objects and textures
* modified kitchen texture
* modified ipa-kitchen.urdf.xacro
* modified models and added four new objects
* added new controllers for platform verrsion 1
* merged kitchens
* changes to ipa-kitchen for freezer door
* changed milk_box
* modified ipa-kitchen.urdf.xacro
* removed outdated urdf model for ipa kitchen
* ipa-kitchen in urdf format
* using xacro for worlds
* moved comic out of wall
* new pictures for ipa kitchen
* we create your future picture at ipa kitchen
* care-o-bot picture at ipa kitchen
* cleaned up in gazebo worlds, now .world and .urdf worlds
* added urdf files for ipa-kitchen
* Milk_box model for object recognition within Gazebo
* added SIMX option to run gazebo in no_X-mode
* launch files for adding objects to gazebo
* missing files
* added missing files
* missing file
* cleanup in cob_gazebo_worlds
* update to use ROBOT and ROBOT_ENV
* missing files from backup
* was missing
* cleanup in simulation and common
* added more cameras
* fixed isssue with odometry topics
* new gazebo_world launch files
* changes for using planned motion; to be tested on real cob
* urdf model for table for use with environment server
* empty world for use with environment server
* launchfile for brics_rc_world.urdf
* brics_rc_world in urdf format
* changed color
* fixed wrong launch file
* missing brics objects
* removed floor
* uhr-messmerf: table
* uhr-messmerf: brics-rc world
* introduced ROBOT_ENV variable
* new cup on table
* new script table cup, modified time_from_start for all trajectories
* new map for ipa 4th floor
* simulated cameras working
* obstacles on floor
* grasp script optimisations
* removed objects from kitchen world to be launched separately
* removed objects from kitchen world to be launched separately
* populate ipa kitchen
* update documentation
* minor modifications to script_server
* cartesian arm movement is working with script_server
* merge with aub
* dual arm cob3 simulation and modified controllers for schunk simulation
* modifide nav package for simulation
* cleanup in cob_simulation
* cob worlds
* Contributors: Alexander Bubeck, Denis Štogl, Florian Weißhardt, Frederik Hegger, abubeck, fmw-jk, ipa-bnm, ipa-fmw, ipa-fmw-sh, ipa-fxm, ipa-goa, ipa-jsf, ipa-nhg, ipa-taj-dm, ipa-uhr-fm, ipa320, nhg-ipa
