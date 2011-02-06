#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_gazebo')
roslib.load_manifest('cob_script_server')

import sys
import time
import unittest

import rospy
import rostest
from trajectory_msgs.msg import *
from simple_script_server import *
from pr2_controllers_msgs.msg import *

NAME = 'cobunit'
class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node(NAME)
        self.message_received = False
        self.sss=simple_script_server()
        
    def setUp(self):
        self.errors = []

    def test_unit(self):
        # fetch parameters
        try:
            # error range
            if not rospy.has_param('~error_range'):
                self.fail('parameter error_range does not exist on ROS Parameter Server')
            error_range = rospy.get_param('~error_range')

            # time to wait before
            if not rospy.has_param('~wait_time'):
                self.fail('parameter wait_time does not exist on ROS Parameter Server')
            wait_time = rospy.get_param('~wait_time')
            
            # part of robot
            if not rospy.has_param('~component'):
                self.fail('parameter component does not exist on ROS Parameter Server')
            component = rospy.get_param('~component')
            
            # command topic
            if not rospy.has_param('~command_topic'):
                self.fail('parameter command_topic does not exist on ROS Parameter Server')
            command_topic = rospy.get_param('~command_topic')
            
            # state topic
            if not rospy.has_param('~state_topic'):
                self.fail('parameter state_topic does not exist on ROS Parameter Server')
            state_topic = rospy.get_param('~state_topic')
            
            # movement command
            if not rospy.has_param('~target'):
                self.fail('parameter target does not exist on ROS Parameter Server')
            target = rospy.get_param('~target')
        except KeyError, e:
            self.fail('Parameters not set properly')
        print """
              Test: %s  
              Target: %s
              Command topic: %s
              State topic: %s
              Error Range: %s
              Wait Time: %s"""%(component, target, command_topic, state_topic, error_range, wait_time)
        
        # check parameters
        # \todo do parameter tests
        #self.assert_(test_duration > 0.0, "bad parameter (test_duration)")
        
        # init subscribers
        sub_command_topic = rospy.Subscriber(command_topic, JointTrajectory, self.cb_command) 
        sub_state_topic = rospy.Subscriber(state_topic, JointTrajectoryControllerState, self.cb_state) 
        
        # init component
        init_handle = self.sss.init(component)
        if init_handle.get_error_code() != 0:
            error_msg = 'Could not initialize ' + component
            self.fail(error_msg)
        
        # start actual test
        print "Waiting for messages"
        # give the topics some seconds to receive messages
        wallclock_timeout_t = time.time() + wait_time
        while not self.message_received and time.time() < wallclock_timeout_t:
            #print "###debug here###" 
            time.sleep(0.1)
        if not self.message_received:
            self.fail('No state message received within wait_time')
          
        # send commands to component
        #move_handle = self.sss.move(component,target)
        move_handle = self.sss.move("arm","folded")
        if move_handle.get_error_code() != 0:
            error_msg = 'Could not move ' + component
            self.fail(error_msg)
        
        # get last point out of trajectory
        traj_endpoint = self.command_traj.points[len(self.command_traj.points)-1]
             
        # Start evaluation
        timeout_t = traj_endpoint.time_from_start.to_sec()*0.5 # movement should already be finished, but let wait with an additional buffer of 50% times the desired time
        rospy.sleep(timeout_t)
        print "Done waiting, validating results"
        actual_pos = self.actual_pos # fix current position configuration for later evaluation
        
        # checking if target position is realy reached
        for i in range(len(traj_endpoint.positions)):
            self.assert_(((traj_endpoint.positions[i] - actual_pos[i]) < error_range), "Target position out of error_range")
            
    # callback functions
    def cb_state(self, msg):
        self.message_received = True
        self.actual_pos = msg.actual.positions
    def cb_command(self, msg):
        self.command_traj = msg
         
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

