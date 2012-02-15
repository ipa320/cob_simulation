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
roslib.load_manifest('cob_gazebo_objects')

import rospy
import os

from gazebo.srv import *
from geometry_msgs.msg import *

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print '[spawn_object.py] Please specify the name of the object to be loaded'
		sys.exit()
	
        #TODO:can call with several arguments
	# read object type (urdf or model) and pose from .yaml file (or parameter server? then upload .yaml before) .yaml file has to be separated for each ROBOT_ENV
	model_type = rospy.get_param("/%s/model_type" % sys.argv[1])
	object_pose = Pose()
	object_pose.position.x = rospy.get_param("/%s/position.x" % sys.argv[1])
	object_pose.position.y = rospy.get_param("/%s/position.y" % sys.argv[1])
	object_pose.position.z = rospy.get_param("/%s/position.z" % sys.argv[1])
	object_pose.orientation.x = rospy.get_param("/%s/orientation.x" % sys.argv[1])
	object_pose.orientation.y = rospy.get_param("/%s/orientation.y" % sys.argv[1])
	object_pose.orientation.z = rospy.get_param("/%s/orientation.z" % sys.argv[1])
	object_pose.orientation.w = rospy.get_param("/%s/orientation.w" % sys.argv[1])
	file_xml = open(roslib.packages.get_pkg_dir('cob_gazebo_objects')+'/objects/'+sys.argv[1]+'.'+rospy.get_param('/%s/model_type' % sys.argv[1]))


	# call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
	if model_type == "urdf":
		srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	#TODO: model_type urdf.xacro calling xacro.py other solution is generate the xml file for all model_type urdf.xacro and save it in cob_gazebo_objects/objects/*.xml
	#elif model_type == "urdf.xacro":
		#xacro_cmd = roslib.packages.get_pkg_dir('xacro')+'/xacro.py' 
		#file_xml = os.popen2('rosrun %s' % xacro_cmd + '%s' % roslib.packages.get_pkg_dir('cob_gazebo_objects')+'/objects/'+sys.argv[1]+'.'+rospy.get_param('/%s/model_type' % sys.argv[1]))
		#file_xml = open('/home/nhg/table_ikea.xml')
		#srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	elif model_type == "model":
		srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
	else:
		print 'Error: Model type not know. model_type = ' + model_type
		sys.exit()

	req = SpawnModelRequest()
	req.model_name = sys.argv[1] # model name from command line input
	req.model_xml = file_xml.read()
	req.initial_pose = object_pose

	res = srv_spawn_model(req)
	
	# evaluate response
	if res.success == True:
		print "model spawned succesfully. status message = " + res.status_message
	else:
		print "Error: model not spawn. error message = " + res.status_message

