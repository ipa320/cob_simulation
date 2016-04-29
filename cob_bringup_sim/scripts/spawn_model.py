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
import rospy, sys, os, time
import string
import warnings

from gazebo_ros import gazebo_interface

from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
import tf.transformations as tft

model_database_template = """<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://MODEL_NAME</uri>
    </include>
  </world>
</sdf>"""        

class SpawnModel():
    def __init__(self):
        self.initial_xyz             = [0,0,0]
        self.initial_rpy             = [0,0,0]
        self.initial_q               = [0,0,0,1]
        self.file_name               = ""
        self.param_name              = ""
        self.database_name           = ""
        self.model_name              = ""
        self.robot_namespace         = rospy.get_namespace()
        self.gazebo_namespace        = "/gazebo"
        self.reference_frame         = ""
        self.unpause_physics         = False
        self.wait_for_model          = ""
        self.wait_for_model_exists   = False
        self.urdf_format             = False
        self.sdf_format              = False
        self.joint_names             = []
        self.joint_positions         = []

    def checkForModel(self,model):
        for n in model.name:
          if n == self.wait_for_model:
            self.wait_for_model_exists = True


    # Generate a blank SDF file with an include for the model from the model database
    def createDatabaseCode(self, database_name):
        return model_database_template.replace("MODEL_NAME", database_name);
    
    def callSpawnService(self):

        # wait for model to exist
        rospy.init_node('spawn_model')

        if not self.wait_for_model == "":
          rospy.Subscriber("%s/model_states"%(self.gazebo_namespace), ModelStates, self.checkForModel)
          r= rospy.Rate(10)
          while not rospy.is_shutdown() and not self.wait_for_model_exists:
            r.sleep()

        if rospy.is_shutdown():
          sys.exit(0)

        if self.file_name != "":
          rospy.loginfo("Loading model xml from file")
          if os.path.exists(self.file_name):
            if os.path.isdir(self.file_name):
              print "Error: file name is a path?", self.file_name
              sys.exit(0)
            if not os.path.isfile(self.file_name):
              print "Error: unable to open file", self.file_name
              sys.exit(0)
          else:
            print "Error: file does not exist", self.file_name
            sys.exit(0)
          # load file
          f = open(self.file_name,'r')
          model_xml = f.read()
          if model_xml == "":
            print "Error: file is empty", self.file_name
            sys.exit(0)

        # ROS Parameter
        elif self.param_name != "":
          rospy.loginfo( "Loading model xml from ros parameter")
          model_xml = rospy.get_param(self.param_name)
          if model_xml == "":
            print "Error: param does not exist or is empty"
            sys.exit(0)

        # Gazebo Model Database
        elif self.database_name != "":
          rospy.loginfo( "Loading model xml from Gazebo Model Database")
          model_xml = self.createDatabaseCode(self.database_name)
          if model_xml == "":
            print "Error: an error occured generating the SDF file"
            sys.exit(0)
        else:
          print "Error: user specified param or filename is an empty string"
          sys.exit(0)

        # setting initial pose
        initial_pose = Pose()
        initial_pose.position.x = self.initial_xyz[0]
        initial_pose.position.y = self.initial_xyz[1]
        initial_pose.position.z = self.initial_xyz[2]
        # convert rpy to quaternion for Pose message
        tmpq = tft.quaternion_from_euler(self.initial_rpy[0],self.initial_rpy[1],self.initial_rpy[2])
        q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
        initial_pose.orientation = q;

        # spawn model
        if self.urdf_format:
          success = gazebo_interface.spawn_urdf_model_client(self.model_name, model_xml, self.robot_namespace, 
                                                             initial_pose, self.reference_frame, self.gazebo_namespace)
        elif self.sdf_format:
          success = gazebo_interface.spawn_sdf_model_client(self.model_name, model_xml, self.robot_namespace, 
                                                            initial_pose, self.reference_frame, self.gazebo_namespace)
        else:
          print "Error: should not be here in spawner helper script, there is a bug"
          sys.exit(0)

        # set model configuration before unpause if user requested
        if len(self.joint_names) != 0:
          try:
            success = gazebo_interface.set_model_configuration_client(self.model_name, self.param_name, 
                                                                      self.joint_names, self.joint_positions, self.gazebo_namespace)
          except rospy.ServiceException, e:
            print "set model configuration service call failed: %s"%e

        # unpause physics if user requested
        if self.unpause_physics:
          rospy.wait_for_service('%s/unpause_physics'%(self.gazebo_namespace))
          try:
            unpause_physics = rospy.ServiceProxy('%s/unpause_physics'%(self.gazebo_namespace), Empty)
            unpause_physics()
          except rospy.ServiceException, e:
            print "unpause physics service call failed: %s"%e

        return

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print usage()
    else:
        print("spawn_model script started") # make this a print incase roscore has not been started
        sm = SpawnModel()
        sm.parseUserInputs()
        sm.callSpawnService()

        
    