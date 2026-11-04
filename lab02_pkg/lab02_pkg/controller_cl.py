# pyrefly: ignore [missing-import]
from math import inf
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import tf_transformations

class ControllerCLNode(Node):

    def __init__(self):
        super().__init__('controller_cl')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # dont put the slash / in front of topic name (because of namespace, if we put / the topic name becomes absolute)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.sub_callback, 10)
        self.subscriber_odom_=self.create_subscription(Odometry, 'odom', self.sub_callback_odom,10)
        self.front_window=20
        self.turn_thres=1000
        self.thres_min=1
        self.turn_angle=90
        self.min_front_dist=1000

        self.th_ref=0.0
        self.timer_period = 0.1  # seconds 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        # self.dist_front=0.0 # minimum distance bet. 20 and -20 degrees
        self.robot_pose=Pose()
        self.yaw=0.0

        self.max_vel=0.22
        self.max_omega=1.4
        self.K_omega=5
        self.omega_err=1000

        self.state=1 # 1: forward, 2: turns
        
    
    def sub_callback(self, msg: LaserScan):
        ranges=msg.ranges
        increment=msg.angle_increment
        front_degrees=self.front_window*math.pi/(2*180.0)
        front_index=int(front_degrees/increment)
        front_range=[]
        for i in range(-front_index,front_index):
            front_range.append(ranges[i])

        self.min_front_dist=np.percentile(front_range,20)

        step_size = 1
        window_size = 5
        for i in range(round(step_size+window_size/2),180,step_size):
            angle_min=(i-window_size/2)*math.pi/180.0
            angle_max=(i+window_size/2)*math.pi/180.0
            index_min=round(angle_min/increment)
            index_max=round(angle_max/increment)
            candidate_range_left=[]
            candidate_range_right=[]
            for j in range(index_min, index_max):
                candidate_range_left.append(ranges[j])
            for j in range(-index_max,-index_min):
                candidate_range_right.append(ranges[j])

            percentile_left=np.percentile(candidate_range_left,10)
            percentile_right=np.percentile(candidate_range_right,10)

            percentile=[]

            if percentile_left > percentile_right:
                percentile=percentile_left
                self.turn_angle=np.deg2rad(i)
            else:
                percentile=percentile_right
                self.turn_angle=-np.deg2rad(i)

            if percentile > self.turn_thres:
                break

        #print(type(ranges))
        print("Front range",self.min_front_dist)
        #print(ranges)
        #print(msg.angle_increment)

        #self.get_logger().info(f"Received message: {msg.linear.x}, {msg.linear.y}")

    def sub_callback_odom(self, msg:Odometry):
        pose_wc=PoseWithCovariance()
        pose_wc=msg.pose # pose with covariance
        self.robot_pose=pose_wc.pose
        orient=self.robot_pose.orientation
        quat=[orient.x,orient.y,orient.z,orient.w]
        _,_,self.yaw= tf_transformations.euler_from_quaternion(quat) 
        #self.get_logger().info(f"Yaw: {self.yaw}")
    
    def timer_callback(self):
        twist=Twist()
        
        # State control
        if self.state==1 and self.min_front_dist <= self.thres_min:
            self.state=2

            self.th_ref=self.yaw+self.turn_angle
        elif self.state==2 and abs(self.omega_err) <= 1e-5:
            self.state=1

        if self.state==1:
            # Velocity control
            twist.linear.x=self.max_vel
            twist.angular.z=0.0
        elif self.state==2:
            twist.linear.x=0.0
            # Angular control
            self.omega_err=self.th_ref-self.yaw
            omega=self.K_omega*self.omega_err # Control law for angular position
            omega=min(omega,self.max_omega)
            omega=max(omega,-self.max_omega)
            twist.angular.z=omega
        
        
        self.get_logger().info(f"State: {self.state}, Theta ref: {self.th_ref}, Theta Odom: {self.yaw}, Vel: {twist.linear.x}, Ang: {twist.angular.z}, Omega error: {self.omega_err}")
        #self.get_logger().info(f"Avg. right: {self.avg_right_dist}, Avg. left: {self.avg_left_dist}")

        self.publisher_.publish(twist)

        # msg = String()
        # msg.data = f'Hello ROS 2: {self.i}'
        # self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg.data}"')
        # self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerCLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()