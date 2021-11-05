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

import sys, rospy, math
from std_srvs.srv import Trigger, TriggerResponse
from thouzer_msgs.msg import Battery, Commander
from thouzer_msgs.msg import App as StatusApp
from enum import Enum


class ThouzerStatus(Enum):
    UNKNOWN = 0
    STARTING = 1
    EMERGENCY = 2
    USER_INUPUT = 3
    COMMAND = 4


class ThouzerCommander():
    def __init__(self):
        # if not self.set_power(False): sys.exit(1)   #モータの電源を切る（TrueをFalseに）
        rospy.on_shutdown(self.set_power)

        self.thouzer_status = ThouzerStatus.UNKNOWN
        self.battery_voltage = 0.0
        self._is_on = False

        self._commander_pub = rospy.Publisher('/thouzer/commander', Commander, queue_size=1)
        self._battery_sub = rospy.Subscriber('/thouzer/battery', Battery, self._callback_battery, queue_size=1)
        self._status_app_sub = rospy.Subscriber('/thouzer/status/app', StatusApp, self._callback_status_app, queue_size=1)

        # Wait for MQTT server
        self._wait_for_mqtt_server()

        self._srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
        self._srv_off = rospy.Service('motor_off', Trigger, self.callback_off)

    def _wait_for_mqtt_server(self):
        i = 0
        while self.thouzer_status == ThouzerStatus.UNKNOWN and self.battery_voltage == 0.0:
            rospy.sleep(0.01)
            i += 1
            if i == 499:
                rospy.loginfo("thouzer_driver/commander.py: Waiting for MQTT server")
                rospy.logerr("thouzer_driver/commander.py: Error connectiong to MQTT server")
                i = 0

    def _callback_status_app(self, msg):
        self.thouzer_status = self._parse_status_message(msg.app)

    def _parse_status_message(self, status_message):
        if status_message == "#start":
            # 起動
            self._is_on = False
            return ThouzerStatus.STARTING
        elif status_message == "#alert":
            # 非常停止
            return ThouzerStatus.EMERGENCY
        elif status_message == "#check":
            # 不明
            return ThouzerStatus.UNKNOWN
        elif "app-userInput" in status_message:
            # ユーザ操作受付
            self._is_on = False
            return ThouzerStatus.USER_INUPUT
        elif "app-whisperer" in status_message:
            # コマンド受付
            self._is_on = True
            return ThouzerStatus.COMMAND

    def _callback_battery(self, msg):
        self.battery_voltage = msg.voltage_v

    def set_power(self, onoff=False):
        try:
            power_command = Commander()
            if self._is_on != onoff:
                if onoff:
                    power_command.app = 'app-whisperer'
                else:
                    power_command.app = ''
                self._is_on = onoff
                self._commander_pub.publish(power_command)
            return True
        except Exception as e:
            print(e)
            rospy.logerr("cannot send MQTT topic to thouzer")
        return False

    def onoff_response(self, onoff):
        d = TriggerResponse()
        d.success = self.set_power(onoff)
        d.message = "ON" if self._is_on else "OFF"
        return d

    def callback_on(self, message):
        return self.onoff_response(True)

    def callback_off(self, message):
        return self.onoff_response(False)

if __name__ == '__main__':
    rospy.init_node('thouzer_commander')
    cmd = ThouzerCommander()
    rospy.spin()
