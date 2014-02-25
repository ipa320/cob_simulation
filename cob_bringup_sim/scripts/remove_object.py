#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2010 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS stack name: cob_environments
# \note
# ROS package name: cob_gazebo_objects
#
# \author
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Feb 2012
#
# \brief
# Implements script server functionalities.
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
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import sys
import roslib
roslib.load_manifest('cob_bringup_sim')

import rospy
import os

from gazebo.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print '[remove_object.py] Please specify the names of the objects to be removed'
		sys.exit()
	
	rospy.init_node("object_remover")

	# check for all objects on parameter server
	if not rospy.has_param("/objects"):
		rospy.logerr("No objects uploaded to /objects")
		all_object_names = []
	else:
		all_object_names = rospy.get_param("/objects").keys()

	# if keyword all is in list of object names we'll load all models uploaded to parameter server
	if "all" in sys.argv:
		object_names = all_object_names
	else:
		object_names = sys.argv
		object_names.pop(0) # remove first element of sys.argv which is file name

	rospy.loginfo("Trying to remove %s",object_names)
	
	for name in object_names:
		# check if object is already spawned
		srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
		req = DeleteModelRequest()
		req.model_name = name
		exists = True
		try:
			res = srv_delete_model(name)
		except rospy.ServiceException, e:
			exists = False
			rospy.logdebug("Model %s does not exist in gazebo.", name)

		if exists:
			rospy.loginfo("Model %s removed.", name)
		else:
			rospy.logerr("Model %s not found in gazebo.", name)
