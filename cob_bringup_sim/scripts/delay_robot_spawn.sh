#!/bin/bash
roslaunch cob_bringup_sim gazebo_world.launch $@ &
sleep 5
roslaunch cob_bringup_sim gazebo_robot.launch $@
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
