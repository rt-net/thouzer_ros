#!/usr/bin/python
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
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from thouzer_msgs.msg import Odom as ThouzerVirtualOdom
from thouzer_msgs.msg import Vel

class OdomPublisher():
    def __init__(self):
        self.sub_odom = rospy.Subscriber('/thouzer/odom', ThouzerVirtualOdom, self._callback_odom)
        self.sub_cmd = rospy.Subscriber('/thouzer/formatted_vel', Vel, self._callback_cmd_vel)
        self.sub_imu = rospy.Subscriber('/imu/data_raw', Imu, self._callback_imu)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.bc_odom = tf2_ros.TransformBroadcaster()

        self.odom_frame_id = rospy.get_param("odom_frame_id", "odom")
        self.robot_frame_id = rospy.get_param("robot_frame_id", "base_footprint")

        self.thouzer_virtual_odom = ThouzerVirtualOdom()

        self.imu_msg = Imu()
        self.imu_msg.angular_velocity.z = 0

        self.vx = 0.0
        self.vth = 0.0
        self.odom_theta = 0.0
        self.x_m = 0.0
        self.y_m = 0.0

        self.send_time = rospy.Time.now()

    def _callback_odom(self, msg):
        """Callback function
        Get Odometry calculated in Thouzer

        Args:
            msg (thouzer_msgs/Odom): ROS message
        """
        self.thouzer_virtual_odom = msg

    def _callback_cmd_vel(self, msg):
        """Callback function
        Get Thouzer Velocity

        Args:
            msg (thouzer_msgs/Vel): ROS message
        """
        self.vx = msg.v_mps
        self.vth = msg.w_degps

    def _callback_imu(self, msg):
        """Callback function
        Get Thouzer Velocity

        Args:
            msg (sensor_msgs/Imu): ROS message
        """
        self.imu_msg = msg

    def send_odom(self, time, pos_x_m, pos_y_m, pos_yaw_rad, vel_x, vel_th):
        odom = Odometry()
        odom.header.stamp = time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.robot_frame_id

        # set the position
        odom.pose.pose.position.x = pos_x_m
        odom.pose.pose.position.y = pos_y_m
        odom.pose.pose.position.z = 0.0

        # set the orientation
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, pos_yaw_rad)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # set the velocity
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vel_th

        self.pub_odom.publish(odom)

    def broadcast_tf(self, time, pos_x_m, pos_y_m, pos_yaw_rad):
        # calculate tf
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp    = time
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id  = self.robot_frame_id

        transform.transform.translation.x = pos_x_m
        transform.transform.translation.y = pos_y_m
        transform.transform.translation.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, pos_yaw_rad)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # broadcast tf
        self.bc_odom.sendTransform(transform)

    def trans(self):
        """
        Translate Odometry(thouzer_msgs/Odom) to Odometry(nav_msgs.msg/Odometry)
        Sends Odometry and TF
        """
        cur_time = rospy.Time.now()

        delta_t = cur_time.to_sec() - self.send_time.to_sec()

        # estimate posture
        if (cur_time.to_sec() - self.imu_msg.header.stamp.to_sec() < 1.0):
            # if not (self.vth + self.vx == 0):
            if True:
                self.odom_theta += round(self.imu_msg.angular_velocity.z * 20)/20 * delta_t  # cut less than 0.05[rad/s]
                # self.odom_theta += self.imu_msg.angular_velocity.z * delta_t
            yaw_rad = self.odom_theta
            self.x_m += self.vx * math.cos(yaw_rad) * delta_t
            self.y_m += self.vx * math.sin(yaw_rad) * delta_t
            rospy.loginfo("Imu")
        else:
            yaw_rad = math.radians(self.thouzer_virtual_odom.yaw_deg)
            self.x_m = self.thouzer_virtual_odom.x_m
            self.y_m = self.thouzer_virtual_odom.y_m
            rospy.loginfo("LocalOdom")

        # send odometry
        self.send_odom(cur_time,
                       self.x_m, self.y_m, yaw_rad,
                       self.vx, self.vth)
        # broadcast tf
        self.broadcast_tf(cur_time,
                         self.x_m, self.y_m, yaw_rad)

        self.send_time = cur_time
        rospy.loginfo("Odomery: x=%s y=%s theta=%s", self.thouzer_virtual_odom.x_m, self.thouzer_virtual_odom.y_m, yaw_rad)


def main():
    rospy.init_node('odom_trans')
    odom_trans = OdomPublisher()
    r = rospy.Rate(50)

    while not rospy.is_shutdown():
        odom_trans.trans()
        r.sleep()

if __name__ == '__main__':
    main()
