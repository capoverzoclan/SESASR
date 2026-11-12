
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, degrees
import matplotlib as mpl

arrow = u'$\u2191$'

def evaluate_sampling_dist(mu, sigma, n_samples, sample_function, n_bins=100):
    """ 
    Evaluate the sampling distribution by generating samples, 
    computing mean and std dev, and plotting a histogram. 
    """
    samples = []

    for i in range(n_samples):
        samples.append(sample_function(mu, sigma))

    print("%30s : mean = %.3f, std_dev = %.3f" % ("Normal", np.mean(samples), np.std(samples)))

    count, bins, ignored = plt.hist(samples, n_bins)
    # plt.plot(bins, norm(mu, sigma).pdf(bins), linewidth=2, color='r')
    plt.xlim([mu - 5*sigma, mu + 5*sigma])
    plt.title("Normal distribution of samples")
    plt.grid()
    # plt.savefig("velocity_model_gaussian_dist.pdf")
    plt.show()

def sample_velocity_motion_model(x, u, a, dt):
    """ 
    Sample velocity motion model.
    Arguments:
    x -- pose of the robot before moving [x, y, theta]
    u -- velocity reading obtained from the robot [v, w]
    a -- noise parameters of the motion model [a1, a2, a3, a4, a5, a6]
    dt -- time interval of prediction
    """
    v_hat = u[0] + np.random.normal(0, a[0]*u[0]**2 + a[1]*u[1]**2)
    w_hat = u[1] + np.random.normal(0, a[2]*u[0]**2 + a[3]*u[1]**2)
    gamma_hat = np.random.normal(0, a[4]*u[0]**2 + a[5]*u[1]**2)

    r = v_hat/w_hat
    x_prime = x[0] - r*sin(x[2]) + r*sin(x[2]+w_hat*dt)
    y_prime = x[1] + r*cos(x[2]) - r*cos(x[2]+w_hat*dt)
    theta_prime = x[2] + w_hat*dt + gamma_hat*dt

    return np.array([x_prime, y_prime, theta_prime])


def main():
    plt.close('all')

    ###### Exercise 1 ########
    n_samples = 500
    dt = 0.5

    x = [2, 4, 0] # initial pose
    u = [1.8, 1] # velocity command (linear and angular)
    a1 = [0.1, 0.1, 0.00001, 0.00001, 0.00001, 0.00001] # noise variance 1
    a11 = [0.00001,  0.00001, 0.1, 0.1,  0.00001,  0.00001]
    a2 = [0.01, 0.01, 0.1, 0.1, 0.1, 0.1] # noise variance 2

    x_prime1 = np.zeros([n_samples, 3])
    x_prime11 = np.zeros([n_samples, 3])
    x_prime2 = np.zeros([n_samples, 3])

    for i in range(n_samples):
        x_prime1[i,:] = sample_velocity_motion_model(x, u, a1, dt)
        x_prime11[i,:] = sample_velocity_motion_model(x, u, a11, dt)
        x_prime2[i,:] = sample_velocity_motion_model(x, u, a2, dt)

    rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
    rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x[2])-90)
    plt.scatter(x[0], x[1], marker=rotated_marker, s=100, facecolors='none', edgecolors='b')

    for x_ in x_prime1[:500]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='r')

    for x_ in x_prime11[:500]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='b')

    for x_ in x_prime2[:500]:
        rotated_marker = mpl.markers.MarkerStyle(marker=arrow)
        rotated_marker._transform = rotated_marker.get_transform().rotate_deg(degrees(x_[2])-90)
        plt.scatter(x_[0], x_[1], marker=rotated_marker, s=40, facecolors='none', edgecolors='g')

    plt.xlabel("x-position [m]")
    plt.ylabel("y-position [m]")
    plt.title("velocity motion model sampling")
    # plt.savefig("velocity_samples.pdf")
    plt.show()



if __name__ == "__main__":
    main()
