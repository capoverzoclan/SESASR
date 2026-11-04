# pyrefly: ignore [missing-import]
from math import inf
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import qos_profile_sensor_data 

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # dont put the slash / in front of topic name (because of namespace, if we put / the topic name becomes absolute)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.sub_callback, qos_profile_sensor_data)
        self.min_front_dist=1000
        self.avg_right_dist=1000
        self.avg_left_dist=1000
        self.thres=1
        self.state=1
        self.cont=0
        self.timer_period = 0.5  # seconds 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.dist_front=0.0 # minimum distance bet. 20 and -20 degrees
        
    
    def sub_callback(self, msg: LaserScan):
        ranges=msg.ranges
        increment=msg.angle_increment
        front_degrees=10.0*math.pi/180.0
        rl_degrees=90.0*math.pi/180.0
        front_index=int(front_degrees/increment)
        rl_index=int(rl_degrees/increment)
        front_range=[]
        right_range=[]
        left_range=[]
        for i in range(-front_index,front_index):
            front_range.append(ranges[i])
        for i in range(rl_index-front_index,rl_index+front_index):
            left_range.append(ranges[i])
        for i in range(-rl_index-front_index,-rl_index+front_index):
            right_range.append(ranges[i])

        self.min_front_dist=min(front_range)
        # self.avg_right_dist=sum(right_range)/len(right_range)
        # self.avg_left_dist=sum(left_range)/len(left_range)
        self.avg_right_dist=min(right_range)
        self.avg_left_dist=min(left_range)
        #print(type(ranges))
        #print(front_range)
        #print(ranges)
        #print(msg.angle_increment)

        #self.get_logger().info(f"Received message: {msg.linear.x}, {msg.linear.y}")
    
    def timer_callback(self):
        twist=Twist()

        # if self.min_front_dist < self.thres:
        #     if self.avg_left_dist > self.avg_right_dist:
        #         self.state = 2
        #     else: 
        #         self.state = 3
        # else:
        #     self.state=1


        if self.state == 1:
            self.get_logger().info("Moving forward")
            twist.linear.x = 0.22
            twist.angular.z = 0.0
            if self.min_front_dist < self.thres:
                if self.avg_left_dist > self.avg_right_dist:
                    self.state = 2
                else: 
                    self.state = 3
        elif self.state == 2:
            self.get_logger().info("Turning left")
            twist.linear.x = 0.0
            twist.angular.z = math.pi/4
            self.cont+=1
            if self.cont == 2/self.timer_period:
                self.state=1
                self.cont=0

        elif self.state == 3:
            self.get_logger().info("Turning right")
            twist.linear.x = 0.0
            twist.angular.z = -math.pi/4
            self.cont+=1
            if self.cont == 2/self.timer_period:
                self.state=1
                self.cont=0

        self.get_logger().info(f"Avg. right: {self.avg_right_dist}, Avg. left: {self.avg_left_dist}")

        self.publisher_.publish(twist)

        # msg = String()
        # msg.data = f'Hello ROS 2: {self.i}'
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()