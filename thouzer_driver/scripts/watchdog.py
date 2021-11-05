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

import rospy
from thouzer_msgs.msg import Battery, StringStamped


class ThouzerWatchdog(object):
    def __init__(self):
        self.battery_voltage = 0.0
        self._battery_sub = rospy.Subscriber('/thouzer/battery', Battery, self._callback_battery, queue_size=1)
        self._stats_pub = rospy.Publisher('/thouzer/status/watchdog', StringStamped, queue_size=1)
        self.timestamp = None

    def _callback_battery(self, msg):
        self.battery_voltage = msg.voltage_v
        self.timestamp = rospy.Time.now()

    def loop(self):
        now = rospy.Time.now()
        msg = StringStamped()
        msg.header.stamp = now
        if self.timestamp == None or now.to_sec() - self.timestamp.to_sec() > 3:
            rospy.logerr("thouzer_driver/watchdog: Cannot receive MQTT topic from Thouzer")
            msg.data = "dead"
        else:
            msg.data = "alive"
        self._stats_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('odom_trans')
    watchdog = ThouzerWatchdog()
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        watchdog.loop()
        r.sleep()
