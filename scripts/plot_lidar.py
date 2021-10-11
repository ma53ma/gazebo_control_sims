#!/usr/bin/env python
#import matplotlib
#matplotlib.use('Agg')
import rospy
import numpy as np

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
    rospy.loginfo('robot theta: {0}'.format(theta))

def gen_lidar_data(scan_msg):
    global x_s
    global y_s
    ranges = scan_msg.ranges
    for deg in range(0,len(ranges)):
        dist_val = ranges[deg]
        if dist_val < dist_thresh:
            obst_x = x + dist_val*np.cos(deg + theta)
            obst_y = y + dist_val*np.sin(deg + theta)
            x_s.append(obst_x)
            y_s.append(obst_y)
    plt.plot(x_s, y_s, 'ko')
    plt.xlim([-2, 0])
    plt.ylim([-2, 0])
    plt.draw()
    plt.pause(0.0000001)
    #plt.pause(0.01)
    #fig.canvas.draw()
    #fig.show()



def plot_lidar():
    rospy.init_node('plot_lidar', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
    scan_sub = rospy.Subscriber('/scan', LaserScan, gen_lidar_data)
    plt.ion()
    plt.show()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    plot_lidar()