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
# Project name: Care-O-bot
# \note
# ROS package name: cob_bringup_sim
#
# \author
# Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
#
# \date Date of creation: Nov 2013
#
# \brief
# This script is able to spawn objects into gazebo which are uploaded
# to the parameter server under /objects
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
import os

import roslib
import rospy
import copy
import math

from spawn_model import SpawnModel
#from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose
import tf.transformations as tft

parents = {}
compound_keys={}

def get_flat_dict(objects, parent_name):
    """expands all objects to a flat dictionary"""
    flat_objects = {}
    for key, value in objects.iteritems():
        # check if we have an atomic object

        if(parent_name!=None):
            
            if('parent_name' not in parents):
                parents[key] = parent_name

                compound_keys[parent_name] = parent_name

            compound_keys[key] = key+'_'+compound_keys[parent_name]

        else:
            compound_keys[key] = key

        if "children" in value:
            # add parent object without children to flat_objects
            tmp_value = copy.deepcopy(value)
            to_process = tmp_value["children"]

            # add position and orientation of parent to all children
            for child_key, child_value in tmp_value["children"].iteritems():
                yaw = objects[key]["orientation"][2]
                x = objects[key]["position"][0] + math.cos(yaw) * child_value["position"][0] - math.sin(yaw) * child_value["position"][1]
                y = objects[key]["position"][1] + math.sin(yaw) * child_value["position"][0] + math.cos(yaw) * child_value["position"][1]
                child_value["position"][0] = x
                child_value["position"][1] = y
                child_value["position"][2] = objects[key]["position"][2] + child_value["position"][2]
                child_value["orientation"][2] = yaw + child_value["orientation"][2]
            del tmp_value["children"]
            flat_objects.update({compound_keys[key]:tmp_value})
            # add flattened children to flat_objects
            flat_objects.update(get_flat_dict(to_process, compound_keys[key]))

        else:
            # add object to flat_objects
            flat_objects.update({compound_keys[key]:value})

    return flat_objects

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print '[spawn_object.py] Please specify the names of the objects to be loaded'
        sys.exit()
    
    #rospy.init_node("object_spawner")

    # check for all objects on parameter server
    if not rospy.has_param("/objects"):
        rospy.logerr("No objects uploaded to /objects")
        sys.exit()
    objects = rospy.get_param("/objects")
    flat_objects = get_flat_dict(objects,None)
    print flat_objects.keys()

    # if keyword all is in list of object names we'll load all models uploaded to parameter server
    if "all" in sys.argv:
        objects = flat_objects
    elif sys.argv[1] not in flat_objects.keys():
        rospy.logerr("Object %s not found", sys.argv[1])
        sys.exit()
    else:
        objects = {sys.argv[1]:flat_objects[sys.argv[1]]}

    rospy.loginfo("Trying to spawn %s", objects.keys())
    
    for key, value in objects.iteritems():
        # check for model
        if not "model" in value:
            rospy.logerr("No model for " + key + " found.")
            continue
        
        model_string = value["model"]
        model_type = value["model_type"]
        newModel = SpawnModel()
        newModel.model_name = key
        
        # check for position
        if not "position" in value:
            rospy.logerr("No position for " + key + " found.")
            continue
        position = value["position"]

        # check for orientation
        if not "orientation" in value:
            rospy.logerr("No orientation for " + key + " found.")
            continue
        orientation = value["orientation"]
        
        # compose pose of object
        newModel.initial_xyz = [float(position[0]), float(position[1]), float(position[2])]
        newModel.initial_rpy = [orientation[0], orientation[1], orientation[2]] 

        # call gazebo service to spawn model (see http://ros.org/wiki/gazebo)
        if model_type == "urdf":
            newModel.urdf_format = True
            newModel.file_name = roslib.packages.get_pkg_dir('cob_gazebo_objects') + '/objects/' + model_string+ '.' + model_type

        elif model_type == "urdf.xacro":
            p = os.popen("rosrun xacro xacro.py " + file_location)

        elif model_type == "sdf":
            newModel.sdf_format = True
            newModel.file_name = roslib.packages.get_pkg_dir('cob_gazebo_objects') + '/objects/' + model_string+ '.' + model_type

        elif model_type == "database":
            newModel.database_name = ""

        else:
            rospy.logerr('Model type not know. model_type = ' + model_type)
            continue

        # spawn new model
        newModel.callSpawnService()

        print 'Object spawned!'