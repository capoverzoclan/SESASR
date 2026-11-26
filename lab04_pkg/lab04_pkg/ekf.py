import math
import numpy as np
from numpy.linalg import inv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from landmark_msgs.msg import LandmarkArray, Landmark

from lab04_pkg.velocity_motion_model import sample_velocity_motion_model
from lab04_pkg.jacobian_models import velocity_mm_simpy, landmark_sm_simpy


def normalize_angle(theta):
    """Normalize angle between [-pi, pi)"""
    return (theta + np.pi) % (2 * np.pi) - np.pi


def residual(a, b, **kwargs):
    """Compute residual between measurement vectors, normalizing angles if requested."""
    y = a - b
    if 'angle_idx' in kwargs:
        idx = kwargs['angle_idx']
        y[idx] = normalize_angle(y[idx])
    return y


class RobotEKF(Node):
    def __init__(self):
        """Initializes the extended Kalman filter creating the necessary matrices."""
        super().__init__('ekf')
        dim_x = 3
        dim_u = 2

        self.mu = np.zeros(dim_x)  # mean state estimate [x, y, theta]
        self.Sigma = np.eye(dim_x) * 0.1  # covariance state estimate
        self.Mt = np.diag([0.05**2, np.deg2rad(2.0)**2])  # process noise covariance

        self.eval_gux, self.eval_Gt, self.eval_Vt = velocity_mm_simpy()
        self.eval_hx_landm, self.eval_Ht_landm = landmark_sm_simpy()

        self._I = np.eye(dim_x)  # identity matrix used for computations

        # Known landmarks coordinates (x, y) in map/world frame
        self.landmarks_map = {
            11: (-1.1, -1.1),
            12: (-1.1,  0.0),
            13: (-1.1,  1.1),
            21: ( 0.0, -1.1),
            22: ( 0.0,  0.0),
            23: ( 0.0,  1.1),
            31: ( 1.1, -1.1),
            32: ( 1.1,  0.0),
            33: ( 1.1,  1.1),
        }

        # Landmark measurement noise covariance Qt = diag(sigma_r^2, sigma_phi^2)
        sigma_r = 0.1
        sigma_phi = np.deg2rad(5.0)
        self.Qt_landmark = np.diag([sigma_r**2, sigma_phi**2])

        # Subscribers and Publishers
        self.subscriber_odom_ = self.create_subscription(
            Odometry, 'odom', self.sub_callback_odom, 10
        )
        self.subscriber_landmark_ = self.create_subscription(
            LandmarkArray, 'landmarks', self.update_step, 10
        )
        self.pub_ekf = self.create_publisher(Odometry, 'ekf', 10)

        self.timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.predict_step)

        self.v = 0.0
        self.omega = 0.0

        # Velocity model noise parameters (a1,a2,a3,a4,a5,a6)
        self.vel_model_noise_parameters=np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]) 

    def predict(self, u, sigma_u, g_extra_args=()):
        """Update the state prediction using control input u and propagate covariance."""
        # Update state prediction evaluating motion model
        args = (*self.mu, *u)
        Gt = self.eval_Gt(*args, *g_extra_args)
        Vt = self.eval_Vt(*args, *g_extra_args)
        self.mu = self.eval_gux(self.mu, u, sigma_u, *g_extra_args)
        self.mu[2] = normalize_angle(self.mu[2])
        self.Sigma = Gt @ self.Sigma @ Gt.T + Vt @ self.Mt @ Vt.T

    def update(self, z, eval_hx, eval_Ht, Qt, Ht_args=(), hx_args=(), residual=residual, **kwargs):
        """Performs the update / correction step of the extended Kalman filter."""
        if np.isscalar(z):
            z = np.asarray([z], float)
            Qt = np.atleast_2d(Qt).astype(float)

        # Kalman gain computation
        Ht = eval_Ht(*Ht_args)
        SigmaHT = self.Sigma @ Ht.T
        self.S = Ht @ SigmaHT + Qt
        self.K = SigmaHT @ inv(self.S)

        # Expected measurement and residual
        z_hat = eval_hx(*hx_args)
        if np.isscalar(z_hat):
            z_hat = np.asarray([z_hat], float)

        y = residual(z, z_hat, **kwargs)
        self.mu = self.mu + self.K @ y
        self.mu[2] = normalize_angle(self.mu[2])

        # Joseph form covariance update
        I_KH = self._I - self.K @ Ht
        self.Sigma = I_KH @ self.Sigma @ I_KH.T + self.K @ Qt @ self.K.T

    def sub_callback_odom(self, msg: Odometry):
        """Store latest linear and angular velocity from /odom topic."""
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z

    def predict_step(self):
        """Timer callback performing the EKF prediction at 20 Hz."""
        sigma_u = self.vel_model_noise_parameters
        v = self.v
        w = self.omega
        # Guard against division by zero in velocity motion model when omega == 0
        if abs(w) < 1e-6:
            w = 1e-6 if w >= 0.0 else -1e-6

        cmd_vel = np.array([v, w])
        ekf_dt = self.timer_period
        self.predict(u=cmd_vel, sigma_u=sigma_u, g_extra_args=(ekf_dt,))
        self.publish_ekf()

    def update_step(self, msg: LandmarkArray):
        """Callback performing measurement updates for each observed landmark."""
        if not msg.landmarks:
            return

        for lm in msg.landmarks:
            lm_id = lm.id
            if lm_id not in self.landmarks_map:
                continue

            z = np.array([lm.range, lm.bearing])
            mx, my = self.landmarks_map[lm_id]

            self.update(
                z,
                eval_hx=self.eval_hx_landm,
                eval_Ht=self.eval_Ht_landm,
                Qt=self.Qt_landmark,
                Ht_args=(*self.mu, mx, my),
                hx_args=(*self.mu, mx, my),
                residual=residual,
                angle_idx=-1,
            )

        self.publish_ekf()

    def publish_ekf(self):
        """Publish estimated pose in nav_msgs/Odometry on /ekf topic."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        msg.pose.pose.position.x = float(self.mu[0])
        msg.pose.pose.position.y = float(self.mu[1])
        msg.pose.pose.position.z = 0.0

        # Planar quaternion from yaw
        theta = float(self.mu[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # 6x6 covariance matrix
        cov = np.zeros((6, 6))
        cov[0:2, 0:2] = self.Sigma[0:2, 0:2]
        cov[5, 5] = self.Sigma[2, 2]
        msg.pose.covariance = cov.flatten().tolist()

        self.pub_ekf.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
