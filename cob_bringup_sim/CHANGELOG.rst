^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_bringup_sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2014-12-15)
------------------
* introduce launchfile argument for -J option of spawn_model
* Contributors: ipa-fxm

0.6.1 (2014-09-22)
------------------

0.6.0 (2014-09-18)
------------------
* remove object script working
* Merge pull request `#65 <https://github.com/ipa320/cob_simulation/issues/65>`_ from ipa320/hydro_dev
  bringin updates from hydro_dev
* Merge pull request `#64 <https://github.com/ipa320/cob_simulation/issues/64>`_ from ipa320/hydro_release_candidate
  Hydro release candidate
* 0.5.2
* update changelog
* Contributors: Florian Weisshardt, ipa-nhg

0.5.2 (2014-08-28)
------------------
* cleaning up
* New maintainer
* Contributors: ipa-fxm, ipa-nhg

0.5.1 (2014-03-21)
------------------
* merge with groovy_dev
* setup tests
* Merge branch 'hydro_dev' of github.com:ipa-nhg/cob_simulation into hydro_dev
* change dependency from gazebo to gazebo_ros
* waiting for gazebo services
* Hydro migration
* installation stuff
* Initial catkinization.
* merge
* adding additional launch file parameters for gazebo simulation
* filename for uploading navigation goals is now taking into account update default_env_config structure in cob_environments
* adjust launch file names and add script to remove objects
* opt env for ROBOT
* removed outdated file
* move tf listener to gazebo worlds; git push origin master
* Spawn_object script also set a description parameter
* enhanced spawn_objects script for better error_handling and updating of already spawned objects
* Addapted spawn_object to spawn multiple times the same object in different positions
* cleanup launch files and substitute env through arg
* fix directory
* fix copy and paste error
* fix test
* Merge branch 'review-ipa320'
* Merge pull request `#15 <https://github.com/ipa320/cob_simulation/issues/15>`_ from ipa-nhg/master
  Moved ipa-apartment.launch file
* addapted robot.launch to the new cob_gazebo_worlds structure
* add arg for robot_config and env_config
* update manifest
* upload default parameters in bringup_sim
* moved cob_sound include to cob_controller_configuration_gazebo
* update deps
* New name space for objects
* merge
* adapt roslaunch tests
* The spawn_object.py script can be called with several arguments
* moved cob_controller_config_gazebo to cob_robots and changed some minor things to support new structure
* Test for ipa-apartment in CMakelists
* filled manifest
* Move spawn_object script to cob_bringup_sim
* Move script spawn_object.py to cob_bringup simscripts/spawn_object.py
* fix icob simulation
* add cob3-4 tests
* merge with ipa320
* update stack
* reduced dependencies
* added bringup_sim package
* Contributors: Alexander Bubeck, Florian Weißhardt, Frederik Hegger, abubeck, ipa-fmw, ipa-fxm, ipa-nhg, ipa-uhr-fm
