import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from  matplotlib.patches import Arc
from scipy.stats import norm
arrow = u'$\u2191$'

def landmark_range_bearing_model(robot_pose, landmark, sigma):
    """""
    Sampling z from landmark model for range and bearing
    """ ""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]

    r_ = math.dist([x, y], [m_x, m_y]) + np.random.normal(0.0, sigma[0])
    phi_ = math.atan2(m_y - y, m_x - x) - theta + np.random.normal(0.0, sigma[1])
    return np.array([r_, phi_])


def landmark_range_bearing_sensor(robot_pose, landmark, sigma, max_range=6.0, fov=math.pi / 2):
    """""
    Simulate the detection of a landmark with a virtual sensor able to estimate range and bearing
    """ ""
    z = landmark_range_bearing_model(robot_pose, landmark, sigma)

    # filter z for a more realistic sensor model (add a max range distance and a FOV)
    if z[0] > max_range or abs(z[1]) > fov / 2:
        return None

    return z

def landmark_model_sample_range_bearing(robot_pose,landmark,sigma):
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]
    sigma_r, sigma_phi = sigma[:]

    d_hat = np.sqrt((m_x-x)**2+(m_y-y)**2)
    phi_hat = np.arctan2(m_y-y,m_x-x)-theta

    r = d_hat + np.random.normal(0.0, sigma_r)
    phi = phi_hat + np.random.normal(0.0, sigma_phi)

    phi = np.arctan2(np.sin(phi), np.cos(phi))
    return np.array([r, phi])


def landmark_model_prob(z, landmark, robot_pose, max_range, fov, sigma):
    """""
    Landmark sensor model algorithm:
    Inputs:
      - z: the measurements features (range and bearing of the landmark from the sensor) [r, phi]
      - landmark: the landmark position in the map [m_x, m_y]
      - x: the robot pose [x,y,theta]
    Outputs:
     - p: the probability p(z|x,m) to obtain the measurement z from the state x
        according to the estimated range and bearing
    """""
    m_x, m_y = landmark[:]
    x, y, theta = robot_pose[:]
    sigma_r, sigma_phi = sigma[:]

    r_hat = math.dist([x, y], [m_x, m_y])
    phi_hat = math.atan2(m_y - y, m_x - x) - theta
    p = norm.pdf(z[0] - r_hat, loc=0, scale=sigma_r) * norm.pdf(z[1] - phi_hat, loc=0, scale=sigma_phi)

    return p

def landmark_model_sample_pose(z, landmark, sigma):
    """""
    Sample a robot pose from the landmark model
    Inputs:
        - z: the measurements features (range and bearing of the landmark from the sensor) [r, phi]
        - landmark: the landmark position in the map [m_x, m_y]
        - sigma: the standard deviation of the measurement noise [sigma_r, sigma_phi]
    Outputs:
        - x': the sampled robot pose [x', y', theta']
    """""
    m_x, m_y = landmark[:]
    sigma_r, sigma_phi = sigma[:]

    gamma_hat = np.random.uniform(0, 2*math.pi) # random angle for the sampled pose
    r_hat = z[0] + np.random.normal(0, sigma_r) # add noise to the range measurement
    phi_hat = z[1] + np.random.normal(0, sigma_phi) # add noise to the bearing measurement

    x_ = m_x + r_hat * math.cos(gamma_hat)
    y_ = m_y + r_hat * math.sin(gamma_hat)
    theta_ = gamma_hat - math.pi - phi_hat

    return np.array([x_, y_, theta_])


def plot_sampled_poses(robot_pose, z, landmark, sigma):
    """""
    Plot sampled poses from the landmark model
    """""
    # plot samples poses
    for i in range(500):
        x_prime = landmark_model_sample_pose(z, landmark, sigma)
        # plot robot pose
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(x_prime[2])-90)
        plt.scatter(x_prime[0], x_prime[1], marker=rotated_marker, s=80, facecolors='none', edgecolors='b')
    
    # plot real pose
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose[2])-90)
    plt.scatter(robot_pose[0], robot_pose[1], marker=rotated_marker, s=140, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("Landmark Model Pose Sampling")
    # plt.savefig("landmark_model_sampling.pdf")
    plt.show()


def plot_landmarks(landmarks, robot_pose, z, p_z, max_range=6.0, fov=math.pi/4):
    """""
    Plot landmarks, robot pose with sensor FOV, and detected landmarks with associated probability
    """""
    x, y, theta = robot_pose[:]

    start_angle = theta + fov/2
    end_angle = theta - fov/2

    plt.figure()
    ax = plt.gca()
    # plot robot pose
    # find the virtual end point for orientation
    endx = x + 0.5 * math.cos(theta)
    endy = y + 0.5 * math.sin(theta)
    plt.plot(x, y, 'or', ms=10)
    plt.plot([x, endx], [y, endy], linewidth = '2', color='r')

    # plot FOV
    # get ray target coordinates
    fov_x_left = x + math.cos(start_angle) * max_range
    fov_y_left = y + math.sin(start_angle) * max_range
    fov_x_right = x + math.cos(end_angle) * max_range
    fov_y_right = y + math.sin(end_angle) * max_range

    plt.plot([x, fov_x_left], [y, fov_y_left], linewidth = '1', color='b')
    plt.plot([x, fov_x_right], [y, fov_y_right], linewidth = '1', color='b')

    R = max_range
    a, b = 2*R, 2*R
    arc = Arc((x, y), a, b,
                 theta1=math.degrees(end_angle), theta2=math.degrees(start_angle), color='b', lw=1.2)
    ax.add_patch(arc)

    # plot landmarks
    for i, lm in enumerate(landmarks):
        plt.plot(lm[0], lm[1], "sk", ms=10, alpha=0.7)

    # plot perceived landmarks position and associated probability (color scale)
    lm_z = np.zeros((len(z), 2))
    for i in range(len(z)):
        # draw endpoint with probability from Likelihood Fields
        lx = x + z[i][0] * math.cos(z[i][1]+theta)
        ly = y + z[i][0] * math.sin(z[i][1]+theta)
        lm_z[i, :] = lx, ly
    
    col = np.array(p_z)
    plt.scatter(lm_z[:,0], lm_z[:,1], s=60, c=col, cmap='viridis')
    # plot colorbar with label
    cbar = plt.colorbar()
    cbar.set_label('Probability density value', rotation=270, labelpad=15)

    plt.show()
    plt.close('all')


def main():
    ##############################
    ### Landmark model example ###
    ##############################

    # robot pose
    robot_pose = np.array([3., 4., -math.pi/2])  # [x, y, theta] in map coordinates
    # landmarks position in the map
    landmark_real = np.array([5,2])
    n_samples=1000

    sigma=(0.1,0.001)

    robot_pose_estimated=np.zeros((n_samples,3))

    z=landmark_model_sample_range_bearing(robot_pose,landmark_real,sigma)

    for i in range(n_samples):
        robot_pose_estimated[i,:]=landmark_model_sample_pose(z,landmark_real,sigma)
        # plot robot pose
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose_estimated[i,2])-90)
        plt.scatter(robot_pose_estimated[i,0], robot_pose_estimated[i,1], marker=rotated_marker, s=80, facecolors='none', edgecolors='b')
    
    # plot real pose
    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(math.degrees(robot_pose[2])-90)
    plt.scatter(robot_pose[0], robot_pose[1], marker=rotated_marker, s=140, facecolors='none', edgecolors='r')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("Landmark Model Pose Sampling")
    # plt.savefig("landmark_model_sampling.pdf")
    plt.show()
    

    
    plt.close('all')

if __name__ == "__main__":
    main()