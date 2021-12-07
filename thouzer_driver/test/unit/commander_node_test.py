#!/usr/bin/env python
# coding: utf8

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

# This script is adapted from
# https://github.com/groove-x/mqtt_bridge/blob/db4ee39da436e60fb3d4557cc1122ec822b65668/scripts/mqtt_bridge_node_test.py
# which is released under the MIT License.
# Copyright (c) 2016 GROOVE X, Inc.
# https://github.com/groove-x/mqtt_bridge/blob/db4ee39da436e60fb3d4557cc1122ec822b65668/LICENSE.txt


import logging
import sys
import time
import unittest
from logging import getLogger

import msgpack
import json
from mock import MagicMock

import rosgraph
import rospy
import paho.mqtt.client as mqtt
from std_srvs.srv import Trigger, TriggerResponse
from thouzer_msgs.msg import Commander

logging.basicConfig(stream=sys.stderr)
logger = getLogger(__name__)
logger.setLevel(logging.DEBUG)

class TestThouzerDriverCommander(unittest.TestCase):

    def setUp(self):
        # mqtt
        self.mqtt_callback_exec_cmd = MagicMock()
        self.mqttc = mqtt.Client()
        self.mqttc.username_pw_set("mqtt", "write_password_here")
        self.mqttc.connect("localhost", 1883, 60)
        self.mqttc.message_callback_add("0/THOUZER_HW/RMS-9999-ZZZZZ/exec/cmd", self.mqtt_callback_exec_cmd)
        self.mqttc.subscribe("0/THOUZER_HW/RMS-9999-ZZZZZ/exec/cmd")
        self.mqttc.loop_start()
        # ros
        rospy.init_node('thouzer_driver_commander_test_node')

    def tearDown(self):
        self.mqttc.loop_stop()
        self.mqttc.disconnect()

    def get_publisher(self, topic_path, msg_type, **kwargs):
        # wait until the number of connections would be same as ros master
        pub = rospy.Publisher(topic_path, msg_type, **kwargs)
        num_subs = len(self._get_subscribers(topic_path))
        for i in range(20):
            num_cons = pub.get_num_connections()
            if num_cons == num_subs:
                return pub
            time.sleep(0.1)
        self.fail("failed to get publisher")

    def _get_subscribers(self, topic_path):
        ros_master = rosgraph.Master('/rostopic')
        topic_path = rosgraph.names.script_resolve_name('rostopic', topic_path)
        state = ros_master.getSystemState()
        subs = []
        for sub in state[1]:
            if sub[0] == topic_path:
                subs.extend(sub[1])
        return subs

    def _wait_callback(self, callback_func):
        for i in range(10):
            if callback_func.called:
                return
            time.sleep(0.1)
        self.fail("callback have not been triggered")
        # callback_func.assert_called_once()

    def _wait_no_callback(self, callback_func):
        for i in range(10):
            if callback_func.called:
                self.fail("callback have been triggered, which should not")
            time.sleep(0.1)
        if not callback_func.called:
            return True
        else:
            self.fail("callback have been triggered, which should not")

    def _publish_thouzer_status(self, status):
        self.mqttc.publish("0/THOUZER_HW/RMS-9999-ZZZZZ/status/app",
                           '{"app": "' + status + '", "running": "OK", "timestamp": "1970-02-14T03:07:52.288" }'
                           )

    def test_exec_cmd_publisher(self):
        publisher = self.get_publisher("/thouzer/commander", Commander, queue_size=1)
        publisher.publish(Commander('app-whisperer'))
        self._wait_callback(self.mqtt_callback_exec_cmd)
        self.mqtt_callback_exec_cmd.assert_called_once()
        msg = self.mqtt_callback_exec_cmd.call_args[0][2]
        self.assertEqual(msg.topic, "0/THOUZER_HW/RMS-9999-ZZZZZ/exec/cmd")
        self.assertEqual(msg.payload, json.dumps({"app": "app-whisperer"}))

    def test_cmd_on_from_service(self):
        self._publish_thouzer_status("#start")
        rospy.wait_for_service('/motor_on')
        rospy.ServiceProxy('/motor_on', Trigger).call()
        self._wait_callback(self.mqtt_callback_exec_cmd)
        self.mqtt_callback_exec_cmd.assert_called_once()
        msg = self.mqtt_callback_exec_cmd.call_args[0][2]
        self.assertEqual(msg.topic, "0/THOUZER_HW/RMS-9999-ZZZZZ/exec/cmd")
        self.assertEqual(msg.payload, json.dumps({"app": "app-whisperer"}))

    def test_cmd_off_from_service(self):
        self._publish_thouzer_status("./bin/app-whisperer")
        rospy.wait_for_service('/motor_off')
        rospy.ServiceProxy('/motor_off', Trigger).call()
        self._wait_callback(self.mqtt_callback_exec_cmd)
        self.mqtt_callback_exec_cmd.assert_called_once()
        msg = self.mqtt_callback_exec_cmd.call_args[0][2]
        self.assertEqual(msg.topic, "0/THOUZER_HW/RMS-9999-ZZZZZ/exec/cmd")
        self.assertEqual(msg.payload, json.dumps({"app": ""}))

    def test_cmd_off_from_service_while_off(self):
        self._publish_thouzer_status("./bin/app-userInputInhibit")
        rospy.wait_for_service('/motor_off')
        rospy.ServiceProxy('/motor_off', Trigger).call()
        self._wait_no_callback(self.mqtt_callback_exec_cmd)

    def test_cmd_on_from_service_while_on(self):
        self._publish_thouzer_status("./bin/app-whisperer")
        rospy.wait_for_service('/motor_on')
        rospy.ServiceProxy('/motor_on', Trigger).call()
        self._wait_no_callback(self.mqtt_callback_exec_cmd)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('thouzer_driver', 'commander_node_test', TestThouzerDriverCommander)
