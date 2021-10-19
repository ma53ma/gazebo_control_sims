#!/usr/bin/env python
#import matplotlib
#matplotlib.use('Agg')
import rospy
import numpy as np
import copy

import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import Point
import tf_conversions
import tf2_ros
from tf.transformations import euler_from_quaternion
from sklearn.metrics.pairwise import rbf_kernel


dist_thresh = np.inf
x=0
y=0
theta=0
endpt_x_s = []
endpt_y_s = []
free_x_s = []
free_y_s = []
ranges = []

laser_point = Point()
#fig = plt.gcf()
#fig.show()
#fig.canvas.draw()

first = True

def odom_callback(odom_msg):
    global x
    global y
    global theta
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y

    # sean's script

    rot_q = odom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # rospy.loginfo('robot theta: {0}'.format(theta))

def gen_lidar_data(scan_msg):
    global x, y, endpt_x_s, endpt_y_s, free_x_s, free_y_s, theta, ranges, laser_point
    current_scan_msg = copy.deepcopy(scan_msg)
    current_theta = copy.deepcopy(theta)
    current_x = copy.deepcopy(x)
    current_y = copy.deepcopy(y)
    new_endpt_x_s = []
    new_endpt_y_s = []
    new_free_x_s = []
    new_free_y_s = []
    points = []
    labels = []

    unoccupied_points_per_meter = 1
    margin = 0.03
    n = 0

    for deg in range(0,len(current_scan_msg.ranges)):
        dist = current_scan_msg.ranges[deg]
        if dist < dist_thresh:
            robot_to_world_T = [[np.cos(current_theta),  -np.sin(current_theta), current_x],
                                [np.sin(current_theta), np.cos(current_theta), current_y],
                                [0, 0, 1]]
            lidar_rad = deg*np.pi / 180

            para = np.sort(np.random.random(np.int16(dist * unoccupied_points_per_meter)) * (1 - 2 * margin) + margin)[
                   :, np.newaxis]

            lidar_pt_x = dist*np.cos(lidar_rad)  # IN LIDAR FRAME
            lidar_pt_y = dist*np.sin(lidar_rad)  # IN LIDAR FRAME

            lidar_pt = [[lidar_pt_x],[lidar_pt_y], [1]]
            world_pt = np.matmul(robot_to_world_T, lidar_pt)

            robot_pos = np.array((x, y))
            lidar_endpoint = np.array([world_pt[0][0], world_pt[1][0]])
            #rospy.loginfo('robot_pos: {0}'.format(robot_pos))
            #rospy.loginfo('lidar_endpoint: {0}'.format(lidar_endpoint))
            #rospy.loginfo('para: {0}'.format(para))
            points_scan_i = robot_pos + para * (lidar_endpoint - robot_pos)
            #rospy.loginfo('points scan i: {0}'.format(points_scan_i))
            #rospy.loginfo('points scan i x_s: {0}'.format((points_scan_i[:, 0])))
            #points = np.vstack((points_scan_i, lidar_endpoint))
            #labels = np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))

            new_endpt_x_s = np.append(new_endpt_x_s, world_pt[0], axis=0)
            new_endpt_y_s = np.append(new_endpt_y_s, world_pt[1], axis=0)
            new_free_x_s = np.append(new_free_x_s, points_scan_i[:, 0], axis=0)
            new_free_y_s = np.append(new_free_y_s, points_scan_i[:, 1], axis=0)
            # rospy.loginfo('deg: {0}'.format(deg))
            if n == 0:  # first data point
                points = np.vstack((points_scan_i, lidar_endpoint))
                labels = np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))
                n += 1
            else:
                points = np.vstack((points, np.vstack((points_scan_i, lidar_endpoint))))
                labels = np.vstack((labels, np.vstack((np.zeros((points_scan_i.shape[0], 1)), np.array([1])[:, np.newaxis]))))
    #endpt_x_s = new_endpt_x_s
    #endpt_y_s = new_endpt_y_s
    #free_x_s = new_free_x_s
    #free_y_s = new_free_y_s
    # plot_lidar_data(new_endpt_x_s, new_endpt_y_s, new_free_x_s, new_free_y_s)
    trainingData = np.hstack((points, labels))
    generate_HM(trainingData)


def calcPosterior(Phi, y, xi, mu0, sig0):
    logit_inv = sigmoid(xi)
    lam = 0.5 / xi * (logit_inv - 0.5)

    sig = 1. /(1./sig0 + 2*np.sum( (Phi.T**2)*lam, axis=1)) # note the numerical trick for the dot product

    mu = sig*(mu0/sig0 + np.dot(Phi.T, y - 0.5).ravel())

    return mu, sig

def sigmoid(x):
    return 1. / (1 + np.exp(-x))


def generate_HM(trainingData):
    global first
    xlims = [-3, 3]
    ylims = [-3, 3]
    # Step 0 - training data
    X3, y3 = trainingData[:, :2], trainingData[:, 2]

    # Step 1 - define hinde points
    xx, yy = np.meshgrid(np.linspace(xlims[0], xlims[1], 30), np.linspace(ylims[0], ylims[1], 30))
    grid = np.hstack((xx.ravel().reshape(-1, 1), yy.ravel().reshape(-1, 1)))

    # Step 2 - compute features
    gamma = 0.7
    Phi = rbf_kernel(X3, grid, gamma=gamma)

    # Step 3 - estimate the parameters
    # Let's define the prior
    N, D = Phi.shape[0], Phi.shape[1]
    epsilon = np.ones(N)
    mu = np.zeros(D)
    sig = 10000 * np.ones(D)

    for i in range(3):
        # E-step
        mu, sig = calcPosterior(Phi, y3, epsilon, mu, sig)

        # M-step
        epsilon = np.sqrt(np.sum((Phi ** 2) * sig, axis=1) + (Phi.dot(mu.reshape(-1, 1)) ** 2).ravel())

    # Step 4 - predict
    qxx, qyy = np.meshgrid(np.linspace(-5, 5, 120), np.linspace(-5, 5, 120))
    qX = np.hstack((qxx.ravel().reshape(-1, 1), qyy.ravel().reshape(-1, 1)))
    qPhi = rbf_kernel(qX, grid, gamma=gamma)
    qw = np.random.multivariate_normal(mu, np.diag(sig), 1000)
    occ = sigmoid(qw.dot(qPhi.T))
    occMean = np.mean(occ, axis=0)
    occStdev = np.std(occ, axis=0)


    # Plot
    if first:
        plt.figure(figsize=(15, 4))
    plt.subplot(131)
    plt.scatter(grid[:,0], grid[:,1], c='k', marker='o')
    plt.scatter(X3[:, 0], X3[:, 1], c=y3, marker='x', cmap='jet')
    if first:
        plt.colorbar()
    plt.title('Hinge points and dataset')
    plt.xlim(xlims)
    plt.ylim(ylims)
    plt.subplot(132)
    plt.scatter(qX[:, 0], qX[:, 1], c=occMean, s=4, cmap='jet', vmin=0, vmax=1)
    if first:
        plt.colorbar()
    plt.title('Occupancy probability - mean')
    plt.xlim(xlims)
    plt.ylim(ylims)
    plt.subplot(133)
    plt.scatter(qX[:, 0], qX[:, 1], c=occStdev, s=4, cmap='jet', vmin=0)
    if first:
        plt.colorbar()
        first = False
    plt.title('Occupancy probability - stdev')
    plt.xlim(xlims)
    plt.ylim(ylims)
    plt.draw()
    plt.pause(0.0000001)


def plot_lidar_data(new_endpt_x_s, new_endpt_y_s, new_freept_x_s, new_freept_y_s):
    #rospy.loginfo('new_endpt_x_s: {0}'.format(new_endpt_x_s))
    #rospy.loginfo('new_endpt_y_s: {0}'.format(new_endpt_y_s))
    #rospy.loginfo('new_freept_x_s: {0}'.format(new_freept_x_s))
    #rospy.loginfo('new_freept_y_s: {0}'.format(new_freept_y_s))

    plt.plot(new_endpt_x_s, new_endpt_y_s, 'ro', markersize=1)
    plt.plot(new_freept_x_s, new_freept_y_s, 'bo', markersize=1)
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    plt.draw()
    plt.pause(0.0000001)


def plot_lidar():
    rospy.init_node('plot_lidar', anonymous=True)
    # plt.figure(figsize=(15, 5))
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=3, buff_size=2**14)
    scan_sub = rospy.Subscriber('/scan', LaserScan, gen_lidar_data, queue_size=3, buff_size=2**14)
    plt.ion()
    plt.show()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    plot_lidar()