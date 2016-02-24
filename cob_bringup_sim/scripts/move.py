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
# ROS stack name: cob_simulation
# \note
# ROS package name: cob_simulation
#
# \author
# Author: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
# \author
# Supervised by: Nadia Hammoudeh Garcia, email:nadia.hammoudeh.garcia@ipa.fhg.de
#
# \date Date of creation: Oct 2013
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


import rospy
import os
import copy
import numpy

from optparse import OptionParser

from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import tf
import math

_usage = """usage: %prog [options]
%prog: Moves a model in gazebo

%prog -h
  Further help on options
"""

class move():
    def __init__(self):
        self.vel = 0.0
        self.model = ""
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState)
        #self.srv_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.rate = 100.0 # hz

    def move_on_line(self, start, goal):
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        segment_length = math.sqrt(dx**2 + dy**2)
        segment_time = segment_length/self.vel
        yaw = math.atan2(dy, dx)

        segment_step_count = int(segment_time*self.rate)
        
        if segment_step_count == 0:
            return
        segment_time = segment_length/self.vel/segment_step_count
        segment_rate = rospy.Rate(self.rate)
    
        for step in numpy.linspace(0, segment_length, segment_step_count):
            step_x = start[0] + step * math.cos(yaw)
            step_y = start[1] + step * math.sin(yaw)

            object_new_pose = Pose()
            object_new_pose.position.x = step_x
            object_new_pose.position.y = step_y
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw + 1.5708)
            object_new_pose.orientation.x = quat[0]
            object_new_pose.orientation.y = quat[1]
            object_new_pose.orientation.z = quat[2]
            object_new_pose.orientation.w = quat[3]

            # spawn new model
            model_state = ModelState()
            model_state.model_name = self.name
            model_state.pose = object_new_pose
            model_state.reference_frame = 'world'
        
            # publish message
            self.pub.publish(model_state)
        
            # call service
            req = SetModelStateRequest()
            req.model_state = model_state
            #res = self.srv_set_model_state(req)
            #if not res.success:
            #    print "something went wrong in service call"
        
            # sleep until next step
            segment_rate.sleep()

    def move_polygon(self, polygon_in):
        # move on all parts of the polygon
        polygon = copy.deepcopy(polygon_in)
        polygon.append(polygon_in[0])
        while not rospy.is_shutdown():
            if len(polygon) <= 1:
                #rospy.loginfo("starting new round")
                polygon = copy.deepcopy(polygon_in)
                polygon.append(polygon_in[0])

            start = polygon[0]
            goal = polygon[1]
            polygon.pop(0)

            #rospy.loginfo("moving on new segment from " + str(start) + " to " + str(goal) + ".")
            self.move_on_line(start, goal)

    def move_circle(self, center, radius):
        # move on all parts of the polygon
        rate = rospy.Rate(self.rate)
        yaw_step = math.asin(1.0/self.rate*self.vel/radius)
        yaw = 0.0
        while not rospy.is_shutdown():
            if yaw >= 2*3.1415926:
                rospy.loginfo("starting new round")
                yaw = 0.0
            object_new_pose = Pose()
            object_new_pose.position.x = center[0] + radius * math.sin(yaw)
            object_new_pose.position.y = center[1] + radius * math.cos(yaw)
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -yaw+1.5708)
            object_new_pose.orientation.x = quat[0]
            object_new_pose.orientation.y = quat[1]
            object_new_pose.orientation.z = quat[2]
            object_new_pose.orientation.w = quat[3]

            # spawn new model
            model_state = ModelState()
            model_state.model_name = self.name
            model_state.pose = object_new_pose
            model_state.reference_frame = 'world'
        
            # publish message
            self.pub.publish(model_state)
        
            # call service
            req = SetModelStateRequest()
            req.model_state = model_state
            #res = self.srv_set_model_state(req)
            #if not res.success:
            #    print "something went wrong in service call"
            yaw += yaw_step
            rate.sleep()

    def run(self):

        parser = OptionParser(usage=_usage, prog=os.path.basename(sys.argv[0]))

        parser.add_option("-m", "--mode",
            dest="mode", choices=["polygon", "circle"], default=None,
            help="Name of model to be moved. Required.")

        parser.add_option("-n", "--name",
            dest="name", metavar="STRING", default=None,
            help="Name of model to be moved. Required.")

        parser.add_option("-v", "--velocity",
            dest="velocity", metavar="Float", default=0.5,
            help="Velocity for movement in [m/s]. Default: 0.5")

        parser.add_option("-p", "--polygon",
            dest="polygon", metavar="List of points [[x1,y1],[x2,y2], ...]", default=None,
            help="List of points forming a polygon in [m], only used for polygon movement. Default: None")

        parser.add_option("-c", "--center",
            dest="center", metavar="Point [x1,y1]", default=None,
            help="Center point, only used for circular movement. Default: None")
        
        parser.add_option("-r", "--radius",
            dest="radius", default=None,
            help="Radius, only used for circular movement. Default: None")
    
        (options, args) = parser.parse_args()

        if options.mode == None:
            parser.error("Please provide a valid mode, see -h option.")
        if options.name == None:
            parser.error("Please provide a valid model name, see -h option.")

        # start node
        self.vel = float(options.velocity)
        self.name = options.name
        rospy.init_node('move' + self.name)
            
        if options.mode == "polygon":
            if (options.polygon == None) or (type(eval(options.polygon)) is not list):
                parser.error("Please provide a valid polygon, see -h option. polygon = " + str(options.polygon))
            self.move_polygon(eval(options.polygon))
        if options.mode == "circle":
            if options.radius == None:
                parser.error("Please provide a valid radius. radius = " + str(options.radius))
            if options.center == None:
                parser.error("Please provide a valid center. center = " + str(options.center))
            self.move_circle(eval(options.center),float(options.radius))


if __name__ == "__main__":
    try:
        m = move()
        m.run()
    except rospy.ROSInterruptException:
        pass


