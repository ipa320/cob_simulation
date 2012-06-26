#!/usr/bin/python

import time
import sys
import roslib
roslib.load_manifest('cob_gazebo_worlds')
import rospy

from math import *

from gazebo.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *


apply_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
door1_closed = True


def callback():
    rand = (numpy.random.randint(0,size=1))
    if rand:
    	move_door("left")
    else:
    	move_door("right")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/elevator_button1_bumper/state", ContactsState, callback)	

    # add default effort to door
    req = ApplyJointEffortRequest()
    req.joint_name = 'joint_elevator_left'
    req.effort = -500
    req.start_time.secs = 0
    req.duration.secs = -10
    res = apply_effort_service(req)

    rospy.spin()

def move_door(side):

	req = ApplyJointEffortRequest()
	req.joint_name = 'joint_elevator_'+side
	req.start_time.secs = 0
	req.duration.secs = -10
	req.effort = 1000
	door_closed = True
	rospy.loginfo("door is opening")
	res = apply_effort_service(req)

	rospy.sleep(30)

	req.effort = -1000
	door_closed = False
	rospy.loginfo("door is closing")
	res = apply_effort_service(req)



if __name__ == '__main__':
    listener()
    #move_door("right")




