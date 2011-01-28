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
            # time of test
            test_duration = float(rospy.get_param('~test_duration'))
            # error range
            error_range = float(rospy.get_param('~error_range'))      
            # time to wait before
            wait_time = rospy.get_param('~wait_time')             
            # topics 
            command_topic = rospy.get_param('~command_topic')
            state_topic = rospy.get_param('~state_topic')
            # part of robot 
            component = rospy.get_param('~component')
            # movement command
            target = rospy.get_param('~target')
        except KeyError, e:
            self.fail('cobunit not initialized properly')
        print """
              Test: %s  
              Movement: %s
              Command: %s
              State: %s
              Test Duration: %s
              Error Range: %s
              Wait Time: %s"""%(component, target, command_topic, state_topic, test_duration, error_range, wait_time)
        self._test_cob(component, command_topic, state_topic, test_duration, error_range, wait_time, target)
        
    def _test_cob(self,component, command_topic, state_topic, test_duration, error_range, wait_time, target): 
        self.assert_(test_duration > 0.0, "bad parameter (test_duration)")
        self.sss.init(component)
        
        # start actual test
        sub_command_topic = rospy.Subscriber(command_topic, JointTrajectory, self.cb_command) 
        sub_state_topic = rospy.Subscriber(state_topic, JointTrajectoryControllerState, self.cb_state) 
        print "Waiting for messages"
        # give the test some seconds to start, may parameterize this in the future
        wallclock_timeout_t = time.time() + wait_time
        while not self.message_received and time.time() < wallclock_timeout_t:
            #print "###debug here###" 
            time.sleep(0.1)
          
        # send commands to cob
        self.sss.move(component,target)
             
        print "Starting measurement"
        timeout_t = rospy.get_time() + test_duration
        while rospy.get_time() < timeout_t:
            rospy.sleep(0.1)
        print "Done waiting, validating results"
        print self.actual_pos
        # checking here
        i = 0
        while i < len(self.comm_pos):  
            self.assert_(((self.comm_pos[i] - self.actual_pos[i]) < error_range), "Positive Range Error happens")
            self.assert_(((self.comm_pos[i] - self.actual_pos[i]) > -error_range), "Negative Range Error happens")
            i = i + 1
            
    # call back functions
    def cb_state(self, msg):
        self.message_received = True
        self.actual_pos = msg.actual.positions
    def cb_command(self, msg):
        self.comm_pos = msg.points[0].positions
        #print "### command callback!!!###"
        #print self.comm_pos
         
if __name__ == '__main__':
    try:
        rostest.run('rostest', NAME, UnitTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

