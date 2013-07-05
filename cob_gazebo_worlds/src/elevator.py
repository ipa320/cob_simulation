#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot
# \note
# ROS stack name: cob_environments
# \note
# ROS package name: cob_gazebo_worlds
#
# \author
# Author: Nadia Hammoudeh Garcia
# \author
# Supervised by: Nadia Hammoudeh Garcia
#
# \date Date of creation: 26.06.2012
#
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time
import sys
import roslib
roslib.load_manifest('cob_gazebo_worlds')
import rospy
import random
from math import *

#from gazebo.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *


apply_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
door_closed = True



def callback(ContactsState):

	if door_closed:
		if (ContactsState.states != []):
			rospy.loginfo("button pressed")	
			rand = (random.randint(0,1))
			if rand == 0:
				move_door("left")
			else:
				move_door("right")
		else:
			rospy.logdebug("button not pressed")
	else:
		rospy.loginfo("Door Opened")



def listener():
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/elevator_button1_bumper/state", ContactsState, callback, queue_size=1)
    rospy.spin()

def move_door(side):

	door_closed = False
	req = ApplyJointEffortRequest()
	req.joint_name = 'joint_elevator_'+side
	req.start_time.secs = 0
	req.duration.secs = -10
	req.effort = 500
	rospy.loginfo("door is opening")
	res = apply_effort_service(req)

	rospy.sleep(10)
	req.effort = -1000
	rospy.loginfo("door is closing")
	res = apply_effort_service(req)

	rospy.sleep(10)
	req.effort = 500
	res = apply_effort_service(req)
	door_closed = True


if __name__ == '__main__':
    listener()



