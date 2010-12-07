#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_gazebo')

import rospy
import tf

from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from simple_script_server import *


# care-o-bot includes
from cob_msgs.msg import *
from cob_srvs.srv import *

class gazebo_services():

	def __init__(self):
		self.sss = simple_script_server()

		#tray
		self.tray_states = JointTrajectoryControllerState()
		self.tray_init_srv = rospy.Service('/tray_controller/init', Trigger, self.tray_init_cb)
		self.tray_stop_srv = rospy.Service('/tray_controller/stop', Trigger, self.tray_stop_cb)
		self.tray_recover_srv = rospy.Service('/tray_controller/recover', Trigger, self.tray_recover_cb)
		self.tray_set_operation_mode_srv = rospy.Service('/tray_controller/set_operation_mode', SetOperationMode, self.tray_set_operation_mode_cb)
		self.tray_js_sub = rospy.Subscriber("/tray_controller/state", JointTrajectoryControllerState, self.tray_js_cb)

		#torso
		self.torso_states = JointTrajectoryControllerState()
		self.torso_init_srv = rospy.Service('/torso_controller/init', Trigger, self.torso_init_cb)
		self.torso_stop_srv = rospy.Service('/torso_controller/stop', Trigger, self.torso_stop_cb)
		self.torso_recover_srv = rospy.Service('/torso_controller/recover', Trigger, self.torso_recover_cb)
		self.torso_set_operation_mode_srv = rospy.Service('/torso_controller/set_operation_mode', SetOperationMode, self.torso_set_operation_mode_cb)
		self.torso_js_sub = rospy.Subscriber("/torso_controller/state", JointTrajectoryControllerState, self.torso_js_cb)

		#head
		self.head_states = JointTrajectoryControllerState()
		self.head_init_srv = rospy.Service('/head_controller/init', Trigger, self.head_init_cb)
		self.head_stop_srv = rospy.Service('/head_controller/stop', Trigger, self.head_stop_cb)
		self.head_recover_srv = rospy.Service('/head_controller/recover', Trigger, self.head_recover_cb)
		self.head_set_operation_mode_srv = rospy.Service('/head_controller/set_operation_mode', SetOperationMode, self.head_set_operation_mode_cb)
		self.head_js_sub = rospy.Subscriber("/head_controller/state", JointTrajectoryControllerState, self.head_js_cb)

		#sdh
		self.sdh_states = JointTrajectoryControllerState()
		self.sdh_init_srv = rospy.Service('/sdh_controller/init', Trigger, self.sdh_init_cb)
		self.sdh_stop_srv = rospy.Service('/sdh_controller/stop', Trigger, self.sdh_stop_cb)
		self.sdh_recover_srv = rospy.Service('/sdh_controller/recover', Trigger, self.sdh_recover_cb)
		self.sdh_set_operation_mode_srv = rospy.Service('/sdh_controller/set_operation_mode', SetOperationMode, self.sdh_set_operation_mode_cb)
		self.sdh_js_sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.sdh_js_cb)

		#arm
		self.arm_states = JointTrajectoryControllerState()
		self.arm_init_srv = rospy.Service('/arm_controller/init', Trigger, self.arm_init_cb)
		self.arm_stop_srv = rospy.Service('/arm_controller/stop', Trigger, self.arm_stop_cb)
		self.arm_recover_srv = rospy.Service('/arm_controller/recover', Trigger, self.arm_recover_cb)
		self.arm_set_operation_mode_srv = rospy.Service('/arm_controller/set_operation_mode', SetOperationMode, self.arm_set_operation_mode_cb)
		self.arm_js_sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.arm_js_cb)

	# tray
	def tray_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def tray_stop_cb(self, req):
		self.tray_goal = []
		for i in range(len(self.tray_states.actual.positions)):
			self.tray_goal.append(self.tray_states.actual.positions[i])
		self.sss.move("tray",[self.tray_goal])
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def tray_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	

	def tray_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def tray_js_cb(self, data):
		self.tray_states=data

	#torso
	def torso_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def torso_stop_cb(self, req):
		self.torso_goal = []
		for i in range(len(self.torso_states.actual.positions)):
			self.torso_goal.append(self.torso_states.actual.positions[i])
		self.sss.move("torso",[self.torso_goal])
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def torso_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def torso_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def torso_js_cb(self, data):
		self.torso_states=data

	# head
	def head_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def head_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def head_stop_cb(self, req):
		self.head_goal = []
		for i in range(len(self.head_states.actual.positions)):
			self.head_goal.append(self.head_states.actual.positions[i])
		self.sss.move("head",[self.head_goal])
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def head_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def head_js_cb(self, data):
		self.head_states=data

	# sdh
	def sdh_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_stop_cb(self, req):
		self.sdh_goal = []
		for i in range(len(self.sdh_states.actual.positions)):
			self.sdh_goal.append(self.sdh_states.actual.positions[i])
		self.sss.move("sdh",[self.sdh_goal])
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def sdh_js_cb(self, data):
		self.sdh_states=data

	# arm
	def arm_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_stop_cb(self, req):
		self.arm_goal = []
		for i in range(len(self.arm_states.actual.positions)):
			self.arm_goal.append(self.arm_states.actual.positions[i])
		self.sss.move("arm",[self.arm_goal],False)
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	def arm_js_cb(self, data):
		self.arm_states=data

	# base
	def base_stop_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp

if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   rospy.loginfo("gazebo_services running")
   rospy.spin()

