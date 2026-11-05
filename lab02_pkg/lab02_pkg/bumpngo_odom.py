# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations as tf
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data


class Bumpngo_odom(Node):

    def __init__(self):
        super().__init__('bumpngo_odom')
        self.declare_parameter('max_vel', 0.18)
        self.max_vel = self.get_parameter('max_vel').get_parameter_value().double_value
        self.declare_parameter('max_w', 1.0)
        self.max_w = self.get_parameter('max_w').get_parameter_value().double_value
        self.declare_parameter('frequency', 80.0)
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer = self.create_timer(1.0/frequency, self.timer_callback)
        self.subscription_timer = self.create_subscription(LaserScan, '/scan', self.listener_callback_scan, qos_profile_sensor_data)
        self.subscription_timer  # prevent unused variable warning
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile_sensor_data)
        self.subscription_odom  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_scan = None
        self.last_pose = None
        self.last_orientation_yaw = None
        self.state = 'FORWARD'
        self.turn_dir_relative = 0.0
        self.turn_dir_abs = 0.0
        self.d_enter = 0.30   # start turning below this front distance (m)
        self.d_exit = 0.40    # stop turning only above this (m), must be > d_enter
        self.K_w = 5

    def listener_callback_scan(self, scan):
        self.last_scan = scan

    def listener_callback_odom(self, msg):
        self.last_pose = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        quat=[orient.x,orient.y,orient.z,orient.w]
        _,_,self.last_orientation_yaw= tf.euler_from_quaternion(quat)

    def clean_ranges(self, scan):
        clean_scan = np.asarray(scan.ranges, dtype=float)
        # inf,nan --> range_max
        clean_scan = np.nan_to_num(clean_scan, nan=scan.range_max, posinf=scan.range_max, neginf=scan.range_max)
        return np.clip(clean_scan, 0.0, scan.range_max)
    
    def angle_to_index(self, scan, angle):
        # wrap the angle into [angle_min, angle_min + 2π)
        angle = (angle - scan.angle_min) % (2 * np.pi) + scan.angle_min
        return int(round((angle - scan.angle_min) / scan.angle_increment))

    def window(self, scan, r, center_deg, size):
        c = self.angle_to_index(scan, np.deg2rad(center_deg))
        n = int(round(np.deg2rad(size/2) / scan.angle_increment))
        idx = np.arange(c - n, c + n + 1) % len(r)    # wraps around the array ends
        return r[idx]

    def front_distance(self, scan, r):
        front = self.window(scan, r, 0, 60)
        return np.percentile(front, 20)

    def choose_dir(self, scan, clean_scan):
        step_size = 5
        window_size = 60
        for i in range(15,180,step_size):
            candidate_range_left=self.window(scan, clean_scan, i, window_size)
            candidate_range_right=self.window(scan, clean_scan, -i, window_size)
        
            percentile_left=np.percentile(candidate_range_left,20)
            percentile_right=np.percentile(candidate_range_right,20)
        
            if percentile_left > percentile_right and percentile_left > self.d_exit:
                self.get_logger().info(f'turning to {i:d}')
                return np.deg2rad(i)
            elif percentile_right > self.d_exit:
                self.get_logger().info(f'turning to {-i:d}')
                return -np.deg2rad(i)
        self.get_logger().info(f'turning to 180')
        return np.deg2rad(180)

    def timer_callback(self):
        if self.last_scan is None or self.last_orientation_yaw is None:
            return
        scan = self.last_scan
        clean_scan = self.clean_ranges(scan)
        front = self.front_distance(scan, clean_scan)
        msg = Twist()

        if self.state == 'FORWARD':
            if front < self.d_enter:
                self.state = 'TURNING'
                self.turn_dir_relative = self.choose_dir(self.last_scan, clean_scan)   # chosen once per turn
                err = self.turn_dir_relative + self.last_orientation_yaw
                self.turn_dir_abs = np.arctan2(np.sin(err), np.cos(err))
                self.get_logger().info(f'Obstacle at {front:.2f} m, turning')
                msg.linear.x = 0.0
            else:
                msg.linear.x = self.max_vel

        elif self.state == 'TURNING':
            if abs(self.last_orientation_yaw-self.turn_dir_abs)<5e-2:
                self.state = 'FORWARD'
                self.get_logger().info('Path clear, going forward')
                msg.angular.z = 0.0
            else:
                theta_err = self.turn_dir_abs-self.last_orientation_yaw
                theta_err = np.arctan2(np.sin(theta_err), np.cos(theta_err))
                w=self.K_w*theta_err # Control law for angular position
                w = np.clip(w, -self.max_w, self.max_w)
                msg.angular.z = w
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bumpngo_odom = Bumpngo_odom()

    rclpy.spin(bumpngo_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bumpngo_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
