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

class UnitTest(unittest.TestCase):
    def __init__(self, *args):
        super(UnitTest, self).__init__(*args)
        rospy.init_node('component_test')
        self.message_received = False
        self.sss=simple_script_server()
        
    def setUp(self):
        self.errors = []

    def test_component(self):
        # get parameters
        try:
            # component
            if not rospy.has_param('~component'):
                self.fail('Parameter component does not exist on ROS Parameter Server')
            component = rospy.get_param('~component')

            # movement command
            if not rospy.has_param('~target'):
                self.fail('Parameter target does not exist on ROS Parameter Server')
            target = rospy.get_param('~target')

            # time to wait before
            wait_time = rospy.get_param('~wait_time',5)

            # error range
            if not rospy.has_param('~error_range'):
                self.fail('Parameter error_range does not exist on ROS Parameter Server')
            error_range = rospy.get_param('~error_range')

        except KeyError, e:
            self.fail('Parameters not set properly')

        print """
              Component: %s  
              Target: %s
              Wait Time: %s
              Error Range: %s"""%(component, target, wait_time, error_range)
        
        # check parameters
        # \todo do more parameter tests
        if error_range < 0.0:
            error_msg = "Parameter error_range should be positive, but is " + error_range
            self.fail(error_msg)
        if wait_time < 0.0:
            error_msg = "Parameter wait_time should be positive, but is " + wait_time
            self.fail(error_msg)
        
        # init subscribers
        command_topic = "/" + component + "_controller/command"
        state_topic = "/" + component + "_controller/state"
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
        rostest.run('rostest', 'component_test', UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

