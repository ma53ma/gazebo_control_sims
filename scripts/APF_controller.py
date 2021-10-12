#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist


# To reset position, Use: $rosservice call /gazebo/reset_simulation "{}"

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
    # rospy.loginfo('x is ${0}'.format(x))
    # rospy.loginfo('y is ${0}'.format(y))
    # rospy.loginfo('theta is ${0}'.format(theta))
    # distance between the "point infront of the robot" (used to calculate v,w) and the robot's center
    # Potential CONFUSION/FIX: Verify weather this length and the formulated equation/matrices actually corresponds to the distance between the center axle (between the two wheels) and point 
    #                      or between the robot's center and the point and if the matrices are correctly adjusted to match this equation
    l = 0.15
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

    # Using x_transformed and y_transformed as the actual coordinates of the robot instead of x,y to accomodate transformation to v,w
    x_transformed = x + l*np.cos(theta)
    y_transformed = y + l*np.sin(theta)

    K_att = 3
    K_rep = 2

    goal_direction = np.subtract([x_transformed,y_transformed], goal)
    # Unit vector in the direction of the goal
    goal_direction_hat = goal_direction/np.linalg.norm(goal_direction)
    # Vel_att will have an uniform magnitude on all points of the map 
    vel_att = np.reshape(K_att*goal_direction_hat, (2,1))

    D_obs = 0.15
    rho_0 = 0.2
    vel_rep = np.array([0.0, 0.0])
    rospy.loginfo('vel_att is ${0}'.format(vel_att))
    obst_cnt = 0
    for i in range(0, len(obsts)):
        obst = np.array(obsts[i])
        state = np.array([x_transformed,y_transformed])
        rho_x = np.linalg.norm(state - obst) - D_obs
        #rospy.loginfo('rho_x is ${0}'.format(rho_x))
        if rho_x < rho_0:
            obst_cnt += 1
            obst_vel_rep = (1/(rho_x*rho_x))*((1/rho_x) - 1/rho_0)*(obst - state)/rho_x
            # rospy.loginfo('obst_vel_rep is ${0}'.format(obst_vel_rep))
            vel_rep += obst_vel_rep
    vel_rep = np.reshape(vel_rep, (2, 1))
    vel_total = -vel_att - K_rep*vel_rep
    rospy.loginfo('vel_rep is ${0}'.format(vel_rep))
    rospy.loginfo('vel_total is ${0}'.format(vel_total))

    # rospy.loginfo('obst_cnt is ${0}'.format(obst_cnt))
    # transformation to v,w
    l_mat = np.array([[1, 0],
             [0, 1/l]])
    rot_mat = np.array([[np.cos(-theta), -np.sin(-theta)],
               [np.sin(-theta), np.cos(theta)]])
    #rospy.loginfo('l_mat is ${0}'.format(l_mat))
    #rospy.loginfo('rot_mat is ${0}'.format(rot_mat))

    

    vw_vector = np.matmul(np.matmul(l_mat, rot_mat), vel_total)
    rospy.loginfo('vw_vector is ${0}'.format(vw_vector))

    #robot maximum velocity limits
    robot_max_linearx = 0.22
    robot_max_rotz = 2.84

    #If the calculated velocities exceed the maximum velocity limits, normalize and scale the vw_vector to the limit.
    if np.abs(vw_vector[0]) > robot_max_linearx:
        temp = vw_vector / np.linalg.norm(vw_vector)
        vw_vector = (robot_max_linearx/np.abs(temp[0])) * temp

    if np.abs(vw_vector[1]) > robot_max_rotz:
        temp = vw_vector / np.linalg.norm(vw_vector)
        vw_vector = (robot_max_rotz/np.abs(temp[1])) * temp

    twist = Twist()
    twist.linear.x = vw_vector[0]
    twist.angular.z = vw_vector[1]

    # Stop if the robot is within 0.25m to the goal
    if np.linalg.norm(goal_direction) < 0.25:
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    twist_pub.publish(twist)


#def xy_vels_to_wv_vels(xdot, ydot):

def APF_controller():
    rospy.init_node('APF_controller', anonymous=True)
    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=3, buff_size=2**14)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    APF_controller()
