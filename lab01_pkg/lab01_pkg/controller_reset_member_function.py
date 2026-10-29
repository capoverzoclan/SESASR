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
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Controller_reset(Node):

    def __init__(self):
        super().__init__('controller_reset')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription_reset = self.create_subscription(Bool, '/reset', self.listener_callback_reset, 10)
        self.subscription_reset  # prevent unused variable warning
        self.N = 1
        self.n = 0
        self.dir = 0

    def timer_callback(self):
        msg = Twist()
        if self.n == self.N:
            self.n = 1
            if self.dir == 3:
                self.dir = 0
                self.N += 1
            else:
                self.dir += 1
        else:
            self.n += 1
        match self.dir:
            case 0:
                msg.linear.x = 1.0
            case 1:
                msg.linear.y = 1.0
            case 2:
                msg.linear.x = -1.0
            case 3:
                msg.linear.y = -1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing:\n  %.3f %.3f %.3f\n  %.3f %.3f %.3f' % (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z))

    def listener_callback_reset(self, msg):
        self.N = 1
        self.n = 0
        self.dir = 0
        self.get_logger().info('Controller reset')

def main(args=None):
    rclpy.init(args=args)

    controller_reset = Controller_reset()

    rclpy.spin(controller_reset)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
