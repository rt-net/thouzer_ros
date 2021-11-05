#!/usr/bin/env python
# -*- coding: utf-8 -*-

# SPDX-License-Identifier: Apache-2.0

# Copyright 2021 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import rospy
from geometry_msgs.msg import Twist
from thouzer_msgs.msg import Vel
from std_srvs.srv import Trigger, TriggerResponse

class CmdVelTwist(object):
    def __init__(self):
        rospy.loginfo("cmd_vel remapper start")
        self._twist_sub = rospy.Subscriber('/cmd_vel', Twist, self.joy_callback, queue_size=1)
        self._vel_pub = rospy.Publisher('/thouzer/vel', Vel, queue_size=1)

    def joy_callback(self, msg):
        vel = Vel()
        vel.v_mps = msg.linear.x
        vel.w_degps = math.degrees(msg.angular.z)
        print(vel)
        self._vel_pub.publish(vel)


if __name__ == '__main__':
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    rospy.init_node('thouzer_cmd_vel')
    logicool_cmd_vel = CmdVelTwist()
    rospy.spin()
