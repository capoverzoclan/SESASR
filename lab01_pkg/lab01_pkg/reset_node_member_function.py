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
from math import sqrt

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class Reset_node(Node):

    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(Pose, '/pose', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Bool, '/reset', 10)

    def listener_callback(self, msg):
        msg_out = Bool()
        msg_out.data = True
        dist = sqrt(msg.position.x**2+msg.position.y**2)
        if dist>6.0:
            self.publisher_.publish(msg_out)
            self.get_logger().info('!!! Distance > 6.0 : dist = %.3f' % dist)

def main(args=None):
    rclpy.init(args=args)

    reset_node = Reset_node()

    rclpy.spin(reset_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()