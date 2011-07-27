#!/usr/bin/env python
import roslib; roslib.load_manifest('cob_gazebo')

import rospy
import tf
import actionlib

from pr2_controllers_msgs.msg import JointTrajectoryAction
from move_base_msgs.msg import MoveBaseAction

# care-o-bot includes
from cob_srvs.srv import *

class gazebo_services():

	def __init__(self):
		# base
		self.base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		#self.base_init_srv = rospy.Service('/base_controller/init', Trigger, self.base_init_cb)
		#self.base_stop_srv = rospy.Service('/base_controller/stop', Trigger, self.base_stop_cb)
		#self.base_recover_srv = rospy.Service('/base_controller/recover', Trigger, self.base_recover_cb)
		#self.base_set_operation_mode_srv = rospy.Service('/base_controller/set_operation_mode', SetOperationMode, self.base_set_operation_mode_cb)

		#torso
		self.torso_client = actionlib.SimpleActionClient('/torso_controller/joint_trajectory_action', JointTrajectoryAction)
		self.torso_init_srv = rospy.Service('/torso_controller/init', Trigger, self.torso_init_cb)
		self.torso_stop_srv = rospy.Service('/torso_controller/stop', Trigger, self.torso_stop_cb)
		self.torso_recover_srv = rospy.Service('/torso_controller/recover', Trigger, self.torso_recover_cb)
		self.torso_set_operation_mode_srv = rospy.Service('/torso_controller/set_operation_mode', SetOperationMode, self.torso_set_operation_mode_cb)

		#tray
		self.tray_client = actionlib.SimpleActionClient('/tray_controller/joint_trajectory_action', JointTrajectoryAction)
		self.tray_init_srv = rospy.Service('/tray_controller/init', Trigger, self.tray_init_cb)
		self.tray_stop_srv = rospy.Service('/tray_controller/stop', Trigger, self.tray_stop_cb)
		self.tray_recover_srv = rospy.Service('/tray_controller/recover', Trigger, self.tray_recover_cb)
		self.tray_set_operation_mode_srv = rospy.Service('/tray_controller/set_operation_mode', SetOperationMode, self.tray_set_operation_mode_cb)

		#arm
		self.arm_client = actionlib.SimpleActionClient('/arm_controller/joint_trajectory_action', JointTrajectoryAction)
		self.arm_init_srv = rospy.Service('/arm_controller/init', Trigger, self.arm_init_cb)
		self.arm_stop_srv = rospy.Service('/arm_controller/stop', Trigger, self.arm_stop_cb)
		self.arm_recover_srv = rospy.Service('/arm_controller/recover', Trigger, self.arm_recover_cb)
		self.arm_set_operation_mode_srv = rospy.Service('/arm_controller/set_operation_mode', SetOperationMode, self.arm_set_operation_mode_cb)

		#arm left
		self.arm_left_client = actionlib.SimpleActionClient('/arm_left_controller/joint_trajectory_action', JointTrajectoryAction)
		self.arm_left_init_srv = rospy.Service('/arm_left_controller/init', Trigger, self.arm_left_init_cb)
		self.arm_left_stop_srv = rospy.Service('/arm_left_controller/stop', Trigger, self.arm_left_stop_cb)
		self.arm_left_recover_srv = rospy.Service('/arm_left_controller/recover', Trigger, self.arm_left_recover_cb)
		self.arm_left_set_operation_mode_srv = rospy.Service('/arm_left_controller/set_operation_mode', SetOperationMode, self.arm_left_set_operation_mode_cb)
		
		#arm right
		self.arm_right_client = actionlib.SimpleActionClient('/arm_right_controller/joint_trajectory_action', JointTrajectoryAction)
		self.arm_right_init_srv = rospy.Service('/arm_right_controller/init', Trigger, self.arm_right_init_cb)
		self.arm_right_stop_srv = rospy.Service('/arm_right_controller/stop', Trigger, self.arm_right_stop_cb)
		self.arm_right_recover_srv = rospy.Service('/arm_right_controller/recover', Trigger, self.arm_right_recover_cb)
		self.arm_right_set_operation_mode_srv = rospy.Service('/arm_right_controller/set_operation_mode', SetOperationMode, self.arm_right_set_operation_mode_cb)

		#sdh
		self.sdh_client = actionlib.SimpleActionClient('/sdh_controller/joint_trajectory_action', JointTrajectoryAction)
		self.sdh_init_srv = rospy.Service('/sdh_controller/init', Trigger, self.sdh_init_cb)
		self.sdh_stop_srv = rospy.Service('/sdh_controller/stop', Trigger, self.sdh_stop_cb)
		self.sdh_recover_srv = rospy.Service('/sdh_controller/recover', Trigger, self.sdh_recover_cb)
		self.sdh_set_operation_mode_srv = rospy.Service('/sdh_controller/set_operation_mode', SetOperationMode, self.sdh_set_operation_mode_cb)
		
		#sdh left
		self.sdh_left_client = actionlib.SimpleActionClient('/sdh_left_controller/joint_trajectory_action', JointTrajectoryAction)
		self.sdh_left_init_srv = rospy.Service('/sdh_left_controller/init', Trigger, self.sdh_left_init_cb)
		self.sdh_left_stop_srv = rospy.Service('/sdh_left_controller/stop', Trigger, self.sdh_left_stop_cb)
		self.sdh_left_recover_srv = rospy.Service('/sdh_left_controller/recover', Trigger, self.sdh_left_recover_cb)
		self.sdh_left_set_operation_mode_srv = rospy.Service('/sdh_left_controller/set_operation_mode', SetOperationMode, self.sdh_left_set_operation_mode_cb)
		
		#sdh right
		self.sdh_right_client = actionlib.SimpleActionClient('/sdh_right_controller/joint_trajectory_action', JointTrajectoryAction)
		self.sdh_right_init_srv = rospy.Service('/sdh_right_controller/init', Trigger, self.sdh_right_init_cb)
		self.sdh_right_stop_srv = rospy.Service('/sdh_right_controller/stop', Trigger, self.sdh_right_stop_cb)
		self.sdh_right_recover_srv = rospy.Service('/sdh_right_controller/recover', Trigger, self.sdh_right_recover_cb)
		self.sdh_right_set_operation_mode_srv = rospy.Service('/sdh_right_controller/set_operation_mode', SetOperationMode, self.sdh_right_set_operation_mode_cb)
		
		#head
		self.head_client = actionlib.SimpleActionClient('/head_controller/joint_trajectory_action', JointTrajectoryAction)
		self.head_init_srv = rospy.Service('/head_controller/init', Trigger, self.head_init_cb)
		self.head_stop_srv = rospy.Service('/head_controller/stop', Trigger, self.head_stop_cb)
		self.head_recover_srv = rospy.Service('/head_controller/recover', Trigger, self.head_recover_cb)
		self.head_set_operation_mode_srv = rospy.Service('/head_controller/set_operation_mode', SetOperationMode, self.head_set_operation_mode_cb)

	# base
	def base_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def base_stop_cb(self, req):
		self.base_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def base_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def base_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	# torso
	def torso_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def torso_stop_cb(self, req):
		self.torso_client.cancel_all_goals()
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

	#tray
	def tray_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def tray_stop_cb(self, req):
		self.tray_client.cancel_all_goals()
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

	# arm
	def arm_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_stop_cb(self, req):
		self.arm_client.cancel_all_goals()
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
		
	# arm left
	def arm_left_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_left_stop_cb(self, req):
		self.arm_left_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_left_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_left_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp
		
	# arm right
	def arm_right_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_right_stop_cb(self, req):
		self.arm_right_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def arm_right_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def arm_right_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	# sdh
	def sdh_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_stop_cb(self, req):
		self.sdh_client.cancel_all_goals()
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
		
	# sdh left
	def sdh_left_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_left_stop_cb(self, req):
		self.sdh_left_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_left_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_left_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp
		
	# sdh right
	def sdh_right_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_right_stop_cb(self, req):
		self.sdh_right_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def sdh_right_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def sdh_right_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

	# head
	def head_init_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def head_stop_cb(self, req):
		self.head_client.cancel_all_goals()
		resp = TriggerResponse()
		resp.success.data = True
		return resp

	def head_recover_cb(self, req):
		resp = TriggerResponse()
		resp.success.data = True
		return resp
	
	def head_set_operation_mode_cb(self, req):
		resp = SetOperationModeResponse()
		resp.success.data = True
		return resp

if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   rospy.loginfo("gazebo_services running")
   rospy.spin()

