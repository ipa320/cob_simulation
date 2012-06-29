#!/usr/bin/python

import time
import sys
import roslib
roslib.load_manifest('cob_gazebo_worlds')
import rospy
import random
from math import *

from gazebo.srv import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *


apply_effort_service = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
door_closed = True



def callback(ContactsState):

	if door_closed:
		if (ContactsState.states != []):
			rospy.loginfo("button pressed")	
			rand = (random.randint(0,1))
			if rand == 0:
				move_door("left")
			else:
				move_door("right")
		else:
			rospy.logdebug("button not pressed")
	else:
		rospy.loginfo("Door Opened")



def listener():
    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/elevator_button1_bumper/state", ContactsState, callback, queue_size=1)
    rospy.spin()

def move_door(side):

	door_closed = False
	req = ApplyJointEffortRequest()
	req.joint_name = 'joint_elevator_'+side
	req.start_time.secs = 0
	req.duration.secs = -10
	req.effort = 500
	rospy.loginfo("door is opening")
	res = apply_effort_service(req)

	rospy.sleep(10)
	req.effort = -1000
	rospy.loginfo("door is closing")
	res = apply_effort_service(req)

	rospy.sleep(10)
	req.effort = 500
	res = apply_effort_service(req)
	door_closed = True


if __name__ == '__main__':
    listener()



