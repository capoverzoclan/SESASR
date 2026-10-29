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

class Bumpngo(Node):

    def __init__(self):
        super().__init__('bumpngo')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription_reset = self.create_subscription(LaserScan, '/scan', self.listener_callback_scan, 10)
        self.subscription_reset  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_scan = None
        self.state = 'FORWARD'
        self.turn_dir = 1.0
        self.d_enter = 0.40   # start turning below this front distance (m)
        self.d_exit = 0.60    # stop turning only above this (m), must be > d_enter

    def listener_callback_scan(self, scan):
        self.last_scan = scan

    def clean_ranges(self, scan):
        clean_scan = np.asarray(scan.ranges, dtype=float)
        # inf,nan --> range_max
        clean_scan = np.nan_to_num(clean_scan, nan=scan.range_max, posinf=scan.range_max, neginf=scan.range_max)
        return np.clip(clean_scan, 0.0, scan.range_max)

    def front_distance(self, scan, r):
        n = int(np.round(np.deg2rad(30) / scan.angle_increment))
        front = np.concatenate((r[:n], r[-n:]))
        return np.percentile(front, 10)

    def choose_dir(self, scan, clean_scan):
        n = int(np.round(np.deg2rad(30) / scan.angle_increment))
        left = clean_scan[n:3*n].mean()       # +30° to +90°
        right = clean_scan[-3*n:-n].mean()    # -90° to -30°
        return 1.0 if left >= right else -1.0   # +1 = turn left, -1 = turn right


    def timer_callback(self):
        if self.last_scan is None:
            return
        scan = self.last_scan
        clean_scan = self.clean_ranges(scan)
        front = self.front_distance(scan, clean_scan)

        if self.state == 'FORWARD' and front < self.d_enter:
            self.state = 'TURNING'
            self.turn_dir = self.choose_dir(self.last_scan, clean_scan)   # chosen once per turn
            self.get_logger().info(f'Obstacle at {front:.2f} m, turning')
        elif self.state == 'TURNING' and front > self.d_exit:
            self.state = 'FORWARD'
            self.get_logger().info('Path clear, going forward')

        msg = Twist()
        if self.state == 'FORWARD':
            msg.linear.x = 0.20
        else:
            msg.angular.z = self.turn_dir * 1.5
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bumpngo = Bumpngo()

    rclpy.spin(bumpngo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bumpngo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
