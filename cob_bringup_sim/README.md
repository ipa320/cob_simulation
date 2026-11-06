## Software Requirements

Make sure that you have the
[cob_simulation](https://index.ros.org/p/cob_simulation) stack installed
on your system.

### Starting a simulated Care-O-bot in gazebo using an environment provided by IPA

The launch files for starting the simulation depend on two environment
variables ROBOT and ROBOT_ENV, you can find a detailed explanation of
their usage on the [main Care-O-bot site](care-o-bot).


```
export ROBOT=cob4-2
export ROBOT_ENV=ipa-apartment
roslaunch cob_bringup_sim robot.launch
```



### Population the world with objects

After you started the gazebo simulation environment you can add objects
from [cob_gazebo_objects](https://index.ros.org/p/cob_gazebo_objects) to
the simulation. Their names and positions for a specific environment are
defined in
[cob_default_env_config](https://index.ros.org/p/cob_default_env_config).
To add objects you can use 

```
roslaunch cob_default_env_config upload_object_locations.launch
rosrun cob_bringup_sim spawn_object.py OBJECT_NAME1 OBJECT_NAME2 OBJECT_NAME3 ...
```

 or


```
rosrun cob_bringup_sim spawn_object.py all
```

 There are pre-defined positions for the objects
depending on the environment they get spawned. For more information
about specifying positions, see
[cob_default_env_config](https://index.ros.org/p/cob_default_env_config).

You can remove objects with 

```
rosrun cob_bringup_sim remove_object.py OBJECT_NAME1 OBJECT_NAME2 OBJECT_NAME3 ...
```

 or


```
rosrun cob_bringup_sim remove_object.py all
```
