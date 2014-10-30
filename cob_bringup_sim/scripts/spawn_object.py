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

from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print '[spawn_object.py] Please specify the names of the objects to be loaded'
		sys.exit()
	
	rospy.init_node("object_spawner")

	# check for all objects on parameter server
	if not rospy.has_param("/objects"):
		rospy.logerr("No objects uploaded to /objects")
		sys.exit()
	all_object_names = rospy.get_param("/objects").keys()

	# if keyword all is in list of object names we'll load all models uploaded to parameter server
	if "all" in sys.argv:
		object_names = all_object_names
	else:
		object_names = sys.argv
		object_names.pop(0) # remove first element of sys.argv which is file name

	rospy.loginfo("Trying to spawn %s",object_names)
	
	for name in object_names:
		# check for object on parameter server
		if not rospy.has_param("/objects/%s" % name):
			rospy.logerr("No description for " + name + " found at /objects/" + name)
			continue
		
		# check for model
		if not rospy.has_param("/objects/%s/model" % name):
			rospy.logerr("No model for " + name + " found at /objects/" + name + "/model")
			continue
		model = rospy.get_param("/objects/%s/model" % name)
		
		# check for model_type
		if not rospy.has_param("/objects/%s/model_type" % name):
			rospy.logerr("No model_type for " + name + " found at /objects/" + name + "/model_type")
			continue
		model_type = rospy.get_param("/objects/%s/model_type" % name)
		
		# check for position
		if not rospy.has_param("/objects/%s/position" % name):
			rospy.logerr("No position for " + name + " found at /objects/" + name + "/position")
			continue
		position = rospy.get_param("/objects/%s/position" % name)

		# check for orientation
		if not rospy.has_param("/objects/%s/orientation" % name):
			rospy.logerr("No orientation for " + name + " found at /objects/" + name + "/orientation")
			continue
		# convert rpy to quaternion for Pose message
		orientation = rospy.get_param("/objects/%s/orientation" % name)
		quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
		object_pose = Pose()
		object_pose.position.x = float(position[0])
		object_pose.position.y = float(position[1])
		object_pose.position.z = float(position[2])
		object_pose.orientation.x = quaternion[0]
		object_pose.orientation.y = quaternion[1]
		object_pose.orientation.z = quaternion[2]
		object_pose.orientation.w = quaternion[3]

		try:
			file_localition = roslib.packages.get_pkg_dir('cob_gazebo_objects') + '/objects/' + model + '.' + model_type
		except:
			print "File not found: cob_gazebo_objects" + "/objects/" + model + "." + model_type
			continue

		# call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
		if model_type == "urdf":
			srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
			file_xml = open(file_localition)
			xml_string=file_xml.read()

		elif model_type == "urdf.xacro":
			p = os.popen("rosrun xacro xacro.py " + file_localition)
			xml_string = p.read()
			p.close()
			srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

		elif model_type == "model":
			srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
			file_xml = open(file_localition)
			xml_string=file_xml.read()
		else:
			rospy.logerr('Model type not know. model_type = ' + model_type)
			continue


		# check if object is already spawned
		srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
		req = DeleteModelRequest()
		req.model_name = name
		exists = True
		try:
			rospy.wait_for_service('/gazebo/delete_model')
			res = srv_delete_model(name)
		except rospy.ServiceException, e:
			exists = False
			rospy.logdebug("Model %s does not exist in gazebo.", name)

		if exists:
			rospy.loginfo("Model %s already exists in gazebo. Model will be updated.", name)

		# spawn new model
		req = SpawnModelRequest()
		req.model_name = name # model name from command line input
		req.model_xml = xml_string
		req.initial_pose = object_pose

		res = srv_spawn_model(req)
	
		# evaluate response
		if res.success == True:
			rospy.loginfo(res.status_message + " " + name)
		else:
			print "Error: model %s not spawn. error message = "% name + res.status_message

