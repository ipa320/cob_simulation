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


import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('my_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0,0,0.01),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "map",
                         "world")
        
        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            rospy.logwarn("ROSTimeMovedBackwardsException during sleep(). Continue anyway...")
        except rospy.exceptions.ROSInterruptException:
            pass
