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
      self.base_tf_listener = tf.TransformListener()
      
      #tray
      self.tray_states = JointTrajectoryControllerState()
      self.tray_stop_srv = rospy.Service('/tray_controller/stop', Trigger, self.tray_stop_cb)
      self.tray_js_sub = rospy.Subscriber("/tray_controller/state", JointTrajectoryControllerState, self.tray_js_cb)

      #torso
      self.torso_states = JointTrajectoryControllerState()
      self.torso_stop_srv = rospy.Service('/torso_controller/stop', Trigger, self.torso_stop_cb)
      self.torso_js_sub = rospy.Subscriber("/torso_controller/state", JointTrajectoryControllerState, self.torso_js_cb)

      #head
      self.head_states = JointTrajectoryControllerState()
      self.head_stop_srv = rospy.Service('/head_controller/stop', Trigger, self.head_stop_cb)
      self.head_js_sub = rospy.Subscriber("/head_controller/state", JointTrajectoryControllerState, self.head_js_cb)

      #sdh
      self.sdh_states = JointTrajectoryControllerState()
      self.sdh_stop_srv = rospy.Service('/sdh_controller/stop', Trigger, self.sdh_stop_cb)
      self.sdh_js_sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.sdh_js_cb)

      #arm
      self.arm_states = JointTrajectoryControllerState()
      self.arm_stop_srv = rospy.Service('/arm_controller/stop', Trigger, self.arm_stop_cb)
      self.arm_js_sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.arm_js_cb)

      #base
      rospy.loginfo("waiting for map...")
      rospy.wait_for_service("/static_map")
      self.base_states = ()
      self.base_stop_srv = rospy.Service('/base_controller/stop', Trigger, self.base_stop_cb)

      rospy.loginfo("gazebo_services running")
      self.run()
      



   def tray_stop_cb(self, req):
      self.tray_goal = []
      for i in range(len(self.tray_states.actual.positions)):
         self.tray_goal.append(self.tray_states.actual.positions[i])
      self.sss.move("tray",[self.tray_goal])
      resp = TriggerResponse()
      return resp

   def tray_js_cb(self, data):
      self.tray_states=data

   def torso_stop_cb(self, req):
      self.torso_goal = []
      for i in range(len(self.torso_states.actual.positions)):
         self.torso_goal.append(self.torso_states.actual.positions[i])
      self.sss.move("torso",[self.torso_goal])
      resp = TriggerResponse()
      return resp

   def torso_js_cb(self, data):
      self.torso_states=data

   def head_stop_cb(self, req):
      self.head_goal = []
      for i in range(len(self.head_states.actual.positions)):
         self.head_goal.append(self.head_states.actual.positions[i])
      self.sss.move("head",[self.head_goal])
      resp = TriggerResponse()
      return resp

   def head_js_cb(self, data):
      self.head_states=data

   def sdh_stop_cb(self, req):
      self.sdh_goal = []
      for i in range(len(self.sdh_states.actual.positions)):
         self.sdh_goal.append(self.sdh_states.actual.positions[i])
      self.sss.move("sdh",[self.sdh_goal])
      resp = TriggerResponse()
      return resp

   def sdh_js_cb(self, data):
      self.sdh_states=data

   def arm_stop_cb(self, req):
      self.arm_goal = []
      for i in range(len(self.arm_states.actual.positions)):
         self.arm_goal.append(self.arm_states.actual.positions[i])
      self.sss.move("arm",[self.arm_goal])
      resp = TriggerResponse()
      return resp

   def arm_js_cb(self, data):
      self.arm_states=data

   def base_stop_cb(self, req):
      resp = TriggerResponse()
      if len(self.base_states) != 0:
         self.base_sxyz = (self.base_states[1][0],self.base_states[1][1],self.base_states[1][2],self.base_states[1][3])
         self.base_euler = euler_from_quaternion(self.base_sxyz)
         print self.base_euler
         self.base_goal = []   # <- 3 double/int
         self.base_goal.append(self.base_states[0][0])
         self.base_goal.append(self.base_states[0][1])
         self.base_goal.append(self.base_euler[2])
         self.sss.move("base",self.base_goal)
      else:
         resp.success=1
         resp.errorMessage="No transformation recieved yet"
      return resp


   def run(self):
      while not rospy.is_shutdown():
         try:
            self.base_states = self.base_tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
         except (tf.LookupException, tf.ConnectivityException):
            continue
      

if __name__ == "__main__":
   rospy.init_node('gazebo_services')
   gazebo_services()
   #rospy.loginfo("gazebo_services running")
   #rospy.spin()

