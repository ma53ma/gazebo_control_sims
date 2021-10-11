#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist


x = 0
y = 0
theta = 0
twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


def odom_callback(odom_msg):
    global x
    global y
    global theta
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y


    rot_q = odom_msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #rospy.loginfo('x is ${0}'.format(x))
    #rospy.loginfo('y is ${0}'.format(y))
    #rospy.loginfo('theta is ${0}'.format(theta))
    goal = [2, 1]
    obsts = [[0, 0],
             [1.15, 0],
             [-1.15, 0],
             [1.15, 1.15],
             [0, 1.15],
             [-1.15, 1.15],
             [1.15, -1.15],
             [0, -1.15],
             [-1.15, -1.15]]


    K_att = 0.5
    K_rep = 2
    vel_x_att = K_att*(x - goal[0])
    vel_y_att = K_att*(y - goal[1])
    #rospy.loginfo('vel_x_att is ${0}'.format(vel_x_att))
    #rospy.loginfo('vel_y_att is ${0}'.format(vel_y_att))
    D_obs = 0.15
    rho_0 = 0.2
    vel_att = np.array([[vel_x_att],[vel_y_att]])
    vel_rep = np.array([0.0, 0.0])
    #rospy.loginfo('vel_att is ${0}'.format(vel_att))
    obst_cnt = 0
    for i in range(0, len(obsts)):
        obst = np.array(obsts[i])
        state = np.array([x,y])
        rho_x = np.linalg.norm(state - obst) - D_obs
        #rospy.loginfo('rho_x is ${0}'.format(rho_x))
        if rho_x < rho_0:
            obst_cnt += 1
            obst_vel_rep = (K_rep/(rho_x*rho_x))*((1/rho_x) - 1/rho_0)*(obst - state)/rho_x
            #rospy.loginfo('obst_vel_rep is ${0}'.format(obst_vel_rep))
            vel_rep += obst_vel_rep
    vel_rep = np.reshape(vel_rep, (2, 1))
    vel_total = -vel_att - vel_rep

    rospy.loginfo('obst_cnt is ${0}'.format(obst_cnt))
    # transformation to v,w
    l = 0.1
    l_mat = np.array([[1, 0],
             [0, 1/l]])
    rot_mat = np.array([[np.cos(-theta), -np.sin(-theta)],
               [np.sin(-theta), np.cos(theta)]])
    #rospy.loginfo('l_mat is ${0}'.format(l_mat))
    #rospy.loginfo('rot_mat is ${0}'.format(rot_mat))

    vw_vector = np.matmul(np.matmul(l_mat, rot_mat), vel_total)
    if np.abs(np.amax(vw_vector)) > 1:
        vw_vector = vw_vector / np.linalg.norm(vw_vector)
    twist = Twist()
    twist.linear.x = 0.5*vw_vector[0]
    twist.angular.z = 0.5*vw_vector[1]
    # twist_pub.publish(twist)


#def xy_vels_to_wv_vels(xdot, ydot):

def APF_controller():
    rospy.init_node('APF_controller', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    APF_controller()