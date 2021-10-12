#!/usr/bin/env python
#import matplotlib
#matplotlib.use('Agg')
import rospy
import numpy as np
import copy

import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

dist_thresh = np.inf
x=0
y=0
theta=0
x_s = []
y_s = []
ranges = []

#fig = plt.gcf()
#fig.show()
#fig.canvas.draw()

def odom_callback(odom_msg):
    global x
    global y
    global theta
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y

    rot_q = odom_msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # rospy.loginfo('robot theta: {0}'.format(theta))

def gen_lidar_data(scan_msg):
    global x_s
    global y_s
    global theta
    global ranges
    current_theta = copy.deepcopy(theta)
    new_x_s = []
    new_y_s = []
    # rospy.loginfo('ranges size: {0}'.format(np.shape(ranges)))
    # rospy.loginfo('distance at 0: {0}'.format(scan_msg.ranges[0]))
    for deg in range(0,len(scan_msg.ranges)):
        dist_val = scan_msg.ranges[deg]
        if dist_val < dist_thresh:
            # theta already in radians
            # lidar converted to radians
            lidar_rad = deg*np.pi / 180
            sum_angle = lidar_rad + current_theta
            while sum_angle > 2*np.pi:
                sum_angle = sum_angle - 2*np.pi
            while sum_angle < -2*np.pi:
                sum_angle = sum_angle + 2*np.pi
            if deg == 180:
                rospy.loginfo('summed angle at 180: {0}'.format(sum_angle))
            obst_x = x + dist_val*np.cos(sum_angle) #  + theta
            obst_y = y + dist_val*np.sin(sum_angle) # + theta
            new_x_s.append(obst_x)
            new_y_s.append(obst_y)
    x_s = new_x_s
    y_s = new_y_s
    plot_lidar_data()
    #plt.pause(0.01)
    #fig.canvas.draw()
    #fig.show()

def plot_lidar_data():
    plt.plot(x_s, y_s, 'ko')
    plt.xlim([-5, 5])
    plt.ylim([-5, 5])
    plt.draw()
    plt.pause(0.0000001)



def plot_lidar():
    rospy.init_node('plot_lidar', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=3, buff_size=2**14)
    scan_sub = rospy.Subscriber('/scan', LaserScan, gen_lidar_data, queue_size=3, buff_size=2**14)
    plt.ion()
    plt.show()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    plot_lidar()