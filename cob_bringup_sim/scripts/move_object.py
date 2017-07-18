#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import sys


import rospy
import os
import copy
import numpy

from optparse import OptionParser

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
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
        self.parse_options()
        # start node
        self.vel = float(self.options.velocity)
        self.name = self.options.name
        rospy.init_node('move' + self.name)
        self.models = self.options.stop_objects.split()
        rospy.wait_for_service('/gazebo/get_model_state')
        self.srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 1)
        self.node_frequency = 100.0 # hz
        self.rate = rospy.Rate(self.node_frequency) # hz
        self.road_block = False
        self.timeout_respawn = rospy.Duration(60)

        # wait for all models to be available
        while not rospy.is_shutdown():
            rospy.sleep(1.0)    # give gazebo some time to start
            res = self.get_model_state(self.name)
            if not res.success:
                continue
            else:
                success = True
                for model in self.models:
                    res = self.get_model_state(model)
                    if not res.success:
                        success = False
                        break   # for-loop
                if success:
                    break   # while-loop
            rospy.logerr("Not all models available yet")
        rospy.loginfo("Ready to move object %s", self.name)

    def get_model_state(self, model_name):
        req = GetModelStateRequest()
        res = GetModelStateResponse()
        req.model_name = model_name
        try:
            res = self.srv_get_model_state(req)
        except rospy.service.ServiceException:
            pass
        return res

    def get_model_dist(self, x, y):
        model_dist = float('inf')
        for model in self.models:
            res = self.get_model_state(model)
            object_dist = math.sqrt(math.pow(res.pose.position.x-x,2) + math.pow(res.pose.position.y-y,2))
            model_dist = min(object_dist, model_dist)
        return model_dist

    def move_on_line(self, start, goal):
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        segment_length = math.sqrt(dx**2 + dy**2)
        segment_time = segment_length/self.vel
        yaw = math.atan2(dy, dx)

        segment_step_count = int(segment_time*self.node_frequency)

        if segment_step_count == 0:
            return
        segment_time = segment_length/self.vel/segment_step_count
        path = numpy.linspace(0, segment_length, segment_step_count)

        idx = 0
        last_time_in_motion = rospy.Time.now()
        while idx < segment_step_count:
            step = path[idx]

            step_x = start[0] + step * math.cos(yaw)
            step_y = start[1] + step * math.sin(yaw)

            # model to close to object?
            model_dist = self.get_model_dist(step_x, step_y)
            if(model_dist <= float(self.options.stop_distance)):
                rospy.logdebug("Model too close to object. Stopping!")
                if (rospy.Time.now() - last_time_in_motion) > self.timeout_respawn:
                    rospy.logdebug("Model move to last waypoint!")
                    self.road_block = True
                    return
            else:
                object_new_pose = Pose()
                object_new_pose.position.x = step_x
                object_new_pose.position.y = step_y
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
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

                idx += 1

                # update last time when the model moved, before coming to halt 
                last_time_in_motion = rospy.Time.now()

            # sleep until next step
            self.rate.sleep()

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

            #rospy.loginfo("moving on new segment from " + str(start) + " to " + str(goal) + ".")
            self.move_on_line(start, goal)
            if self.road_block:
                self.road_block = False
                continue
            polygon.pop(0)

    def move_circle(self, center, radius):
        # move on all parts of the polygon
        yaw_step = math.asin(1.0/self.node_frequency*self.vel/radius)
        yaw = 0.0
        last_time_in_motion = rospy.Time.now()
        while not rospy.is_shutdown():
            if yaw >= 2*math.pi:
                rospy.loginfo("starting new round")
                yaw = 0.0

            step_x = center[0] + radius * math.sin(yaw)
            step_y = center[1] + radius * math.cos(yaw)

            # model to close to object?
            model_dist = self.get_model_dist(step_x, step_y)
            if(model_dist <= float(self.options.stop_distance)):
                rospy.logdebug("Model too close to object. Stopping!")
                if (rospy.Time.now() - last_time_in_motion) > self.timeout_respawn:
                    rospy.logdebug("Model move to start waypont of the polygone!")
                    yaw = 2*math.pi
                    continue
            else:
                object_new_pose = Pose()
                object_new_pose.position.x = step_x
                object_new_pose.position.y = step_y
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, -yaw)
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

                yaw += yaw_step

                # update last time when the model moved, before coming to halt 
                last_time_in_motion = rospy.Time.now()

            # sleep until next step
            self.rate.sleep()

    def parse_options(self):
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
            dest="radius", metavar="Float", default=None,
            help="Radius, only used for circular movement. Default: None")

        parser.add_option("-o", "--stop-objects",
            dest="stop_objects", metavar="List of objects 'object_1 object_2 ...'", default='',
            help="List of Model-Name of objects that are to be avoided. Default: ''")

        parser.add_option("-d", "--stop-distance",
            dest="stop_distance", metavar="Float", default=2.0,
            help="Allowed distance to objects before stopping. Default: 2.0")

        (self.options, args) = parser.parse_args()

        if self.options.mode == None:
            parser.error("Please provide a valid mode, see -h option.")
        if self.options.name == None:
            parser.error("Please provide a valid model name, see -h option.")


    def run(self):            
        if self.options.mode == "polygon":
            if (self.options.polygon == None) or (type(eval(self.options.polygon)) is not list):
                parser.error("Please provide a valid polygon, see -h option. polygon = " + str(self.options.polygon))
            self.move_polygon(eval(self.options.polygon))
        if self.options.mode == "circle":
            if self.options.radius == None:
                parser.error("Please provide a valid radius. radius = " + str(self.options.radius))
            if self.options.center == None:
                parser.error("Please provide a valid center. center = " + str(self.options.center))
            self.move_circle(eval(self.options.center),float(self.options.radius))


if __name__ == "__main__":
    try:
        m = move()
        m.run()
    except rospy.ROSInterruptException:
        pass
