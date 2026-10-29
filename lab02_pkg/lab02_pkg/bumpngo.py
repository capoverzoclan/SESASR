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
        self.subscription_reset = self.create_subscription(LaserScan, '/scan', self.listener_callback_control, 10)
        self.subscription_reset  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def fwclear(self, scan):
        ranges = np.asarray(scan.ranges)
        n = int(np.round(np.deg2rad(30)/scan.angle_increment))
        r = ranges[:n]
        l = ranges[-n:]
        front = np.concatenate((r,l))
        valid = front[np.isfinite(front)]
        if valid.size == 0:
            return True
        avg = valid.mean()
        if avg <= 0.40:
            return False
        else:
            return True

    def rotate(self):
        return True

    def listener_callback_control(self, scan):
        if self.fwclear(scan):
            msg = Twist()
            msg.linear.x = 0.20
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Forward clear: proceeding at 0.20 m/s')
        else:
            self.rotate()
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
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
