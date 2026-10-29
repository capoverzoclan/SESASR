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

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class Localization_reset(Node):

    def __init__(self):
        super().__init__('localization_reset')
        self.subscription_control = self.create_subscription(Twist, '/cmd_vel', self.listener_callback_control, 10)
        self.subscription_control  # prevent unused variable warning
        self.subscription_reset = self.create_subscription(Bool, '/reset', self.listener_callback_reset, 10)
        self.subscription_reset  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        self.pos = Pose()

    def listener_callback_control(self, msg):
        self.pos.position.x += msg.linear.x
        self.pos.position.y += msg.linear.y
        self.publisher_.publish(self.pos)
        self.get_logger().info('New pos:\n  %.3f %.3f %.3f\n  %.3f %.3f %.3f %.3f' % (self.pos.position.x, self.pos.position.y, self.pos.position.z, self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w))

    def listener_callback_reset(self, msg):
        self.pos.position.x = 0.0
        self.pos.position.y = 0.0
        self.publisher_.publish(self.pos)
        self.get_logger().info('Reset msg received:\nNew pos:\n  %.3f %.3f %.3f\n  %.3f %.3f %.3f %.3f' % (self.pos.position.x, self.pos.position.y, self.pos.position.z, self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w))


def main(args=None):
    rclpy.init(args=args)

    localization_reset = Localization_reset()

    rclpy.spin(localization_reset)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
