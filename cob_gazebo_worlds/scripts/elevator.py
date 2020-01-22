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
import random

import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64

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
	rospy.Subscriber("/elevator_button1_bumper", ContactsState, callback, queue_size=1)
	rospy.spin()

def move_door(side):

	door_closed = False
	topic_name = '/world/elevator_%s_joint_position_controller/command' %side
	pub = rospy.Publisher(topic_name,Float64,queue_size=10)
	rospy.sleep(1)
	pos = 0.87
	pub.publish(pos)

	rospy.loginfo("%s door is opening" %side)

	try:
		rospy.sleep(10)
	except rospy.exceptions.ROSInterruptException:
		return

	pos = 0
	rospy.loginfo("%s door is closing" %side)
	pub.publish(pos)

	try:
		rospy.sleep(10)
	except rospy.exceptions.ROSInterruptException:
		return

	door_closed = True

if __name__ == '__main__':
	listener()



