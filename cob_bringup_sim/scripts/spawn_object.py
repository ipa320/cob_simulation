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

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest, GetWorldProperties
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

    rospy.init_node("object_spawner")

    # check for all objects on parameter server
    if not rospy.has_param("/objects"):
        rospy.logerr("No objects uploaded to /objects")
        sys.exit()
    objects = rospy.get_param("/objects")
    flat_objects = get_flat_dict(objects,None)

    # check for all object groups on parameter server
    if rospy.has_param("/object_groups"):
        groups = rospy.get_param("/object_groups")
    else:
        groups = {}
        rospy.loginfo('No object-groups uploaded to /object_groups')

    # if keyword all is in list of object names we'll load all models uploaded to parameter server
    if "all" in sys.argv:
        objects = flat_objects
    elif not set(groups.keys()).isdisjoint(sys.argv):
        # get all key that are in both, the dictionary and argv
        found_groups = set(groups.keys()) & set(sys.argv)
        # get all object_names from keys that are in argv
        object_names = [groups[k] for k in found_groups]
        # flatten list of lists 'object_names' to a list
        object_names = [item for sublist in object_names for item in sublist]
        # save all dict-objects with names in 'object_names' in 'objects'
        objects = {}
        for object_name in object_names:
            if object_name in flat_objects.keys():
                objects.update({object_name:flat_objects[object_name]})
    elif sys.argv[1] not in flat_objects.keys():
        rospy.logerr("Object %s not found", sys.argv[1])
        sys.exit()
    else:
        objects = {sys.argv[1]:flat_objects[sys.argv[1]]}

    rospy.loginfo("Trying to spawn %s", objects.keys())

    # get all current models from gazebo
    # check if object is already spawned
    try:
        rospy.wait_for_service('/gazebo/get_world_properties', 30)
    except rospy.exceptions.ROSException:
        rospy.logerr("Service /gazebo/get_world_properties not available.")
        sys.exit()

    srv_get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)

    try:
        world_properties = srv_get_world_properties()
    except rospy.ServiceException as exc:
        rospy.logerr("Service did not process request: " + str(exc))

    if world_properties.success:
        existing_models = world_properties.model_names
    else:
        existing_models = []

    # Iterate through all objects
    for key, value in objects.iteritems():
        # check for model
        if not "model" in value:
            rospy.logerr("No model for " + key + " found.")
            continue
        model_string = value["model"]
        model_type = value["model_type"]
        
        # check for position
        if not "position" in value:
            rospy.logerr("No position for " + key + " found.")
            continue
        position = value["position"]

        # check for orientation
        if not "orientation" in value:
            rospy.logerr("No orientation for " + key + " found.")
            continue
        # convert rpy to quaternion for Pose message
        orientation = value["orientation"]
        quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        
        # compose pose of object
        object_pose = Pose()
        object_pose.position.x = float(position[0])
        object_pose.position.y = float(position[1])
        object_pose.position.z = float(position[2])
        object_pose.orientation.x = quaternion[0]
        object_pose.orientation.y = quaternion[1]
        object_pose.orientation.z = quaternion[2]
        object_pose.orientation.w = quaternion[3]

        # get file location
        try:
            file_location = roslib.packages.get_pkg_dir('cob_gazebo_objects') + '/objects/' + model_string+ '.' + model_type
        except roslib.packages.InvalidROSPkgException:
            rospy.logerr("No model package found for " + key + ": " + cob_gazebo_objects + " does not exist in ROS_PACKAGE_PATH")
            continue

        # open file for urdf.xacro or urdf/sdf/model
        if model_type == "urdf.xacro":
            try:
                f = os.popen("rosrun xacro xacro.py " + file_location)
            except:
                rospy.logerr("No xacro file found for " + key + " at " + file_location)
                continue
            model_type = "urdf"
        elif model_type in ['urdf', 'sdf', 'model']:
            try:
                f = open(file_location)
            except:
                rospy.logerr("No model file found for " + key + " at " + file_location)
                continue 
        else:
            rospy.logerr('Model type not known. model_type = ' + model_type)
            continue

        # read and close file
        xml_string = f.read()
        f.close()

        # open spawn service
        try:
            rospy.wait_for_service('/gazebo/spawn_'+model_type+'_model',30)
        except rospy.exceptions.ROSException:
            rospy.logerr("Service /gazebo/spawn_"+model_type+"_model not available.")
            sys.exit()
        srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_'+model_type+'_model', SpawnModel)

        # delete model if it already exists
        if key in existing_models:
            srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel) # TODO this service causes gazebo (current groovy version) to crash
            try:
                res = srv_delete_model(key)
            except rospy.ServiceException, e:
                rospy.logdebug("Error while trying to call Service /gazebo/get_world_properties.")
            rospy.loginfo("Model %s already exists in gazebo. Model will be deleted and added again.", key)

        # spawn new model
        req = SpawnModelRequest()
        req.model_name = key # model name from command line input
        req.model_xml = xml_string
        req.initial_pose = object_pose

        res = srv_spawn_model(req)

        # evaluate response
        if res.success == True:
            rospy.loginfo(res.status_message + " " + key)
        else:
            rospy.logerr("Error: model %s not spawn. error message = "% key + res.status_message)

