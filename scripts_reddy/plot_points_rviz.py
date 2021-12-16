#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Twist
from matplotlib.collections import LineCollection
from matplotlib import cm

import sched, time

try:
    from cubic_spline_trajectory import Spline2D, cubic_trajectory
except ImportError:
    raise

class MarkerBasics(object):
    def __init__(self, points_array, scale=0.1,pub_topic='/points', rgba_array = None, is_marker_array = True):
        self.rate = rospy.Rate(1)
        self.scale = scale
        self.is_marker_array = is_marker_array
        if is_marker_array == True:
            self.marker_object_pub = rospy.Publisher(pub_topic, MarkerArray, queue_size=1)
            self.init_marker_Arr(points_array, z_val=0)
            
        else:
            self.init_lineStrip(points_array, rgba_array, z_val = 0)
            self.marker_object_pub = rospy.Publisher(pub_topic, Marker, queue_size=1)
    
    def init_lineStrip(self, points_array, rgba_array, z_val = 0):
        self.make_marker()

        self.marker_object.type = Marker.LINE_STRIP

        point_arr_data = []
        for coordinate in points_array:
            my_point = Point()
            my_point.x = coordinate[0]
            my_point.y = coordinate[1]
            my_point.z = z_val
            point_arr_data.append(my_point)
        self.marker_object.points = point_arr_data

        my_colors = []
        for color in rgba_array:
            my_color = ColorRGBA()
            my_color.r = color[0]
            my_color.g = color[1]
            my_color.b = color[2]
            my_color.a = color[3]
            my_colors.append(my_color)
        self.marker_object.colors = my_colors
        
        # for publishing
        self.marker_arr_object = self.marker_object


    def init_marker_Arr(self,points_array, z_val=0):
        self.marker_arr_object = MarkerArray()

        self.array_length = len(points_array)
        for index, coordinate in enumerate(points_array):
            self.marker_arr_object.markers.append(self.make_marker(index, z_val, coordinate))

    def make_marker(self, index=0, z_val=0, coordinate=[0.0, 0.0]):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "/odom"
        self.marker_object.header.stamp = rospy.Time.now()
        self.marker_object.ns = "robot_objective"
        self.marker_object.id = index
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        my_point.x = coordinate[0]
        my_point.y = coordinate[1]
        my_point.z = z_val
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = self.scale
        self.marker_object.scale.y = self.scale
        self.marker_object.scale.z = self.scale

        if self.is_marker_array == True:
            self.marker_object.color.r = 0.0
            self.marker_object.color.g = 0.0
            self.marker_object.color.b = 1.0
            self.marker_object.color.a = 1.0

            if index == 0:
                self.marker_object.color.g = 0.6
                self.marker_object.color.b = 0.1
            elif index == self.array_length - 1:
                self.marker_object.color.r = 0.9
                self.marker_object.color.b = 0.1
                self.marker_object.color.b = 0.1


        self.marker_object.lifetime = rospy.Duration(0)

        return self.marker_object
    
    def start(self):
        while not rospy.is_shutdown():
            self.marker_object_pub.publish(self.marker_arr_object)
            self.rate.sleep()

def send_command(vel_msg):
    print(time.time())
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    rospy.init_node('points_vis_node', anonymous=True)
    with open('way_points.txt') as file:
        points_array = file.readlines()
        points_array = [[float(str_val) for str_val in line.rstrip().split()] for line in points_array]
    markerbasics_object = MarkerBasics(points_array)

    # Cubic Spline fit and display
    # make it a numpy array and extend second dimensions with zeros for theta
    points_array = np.pad(np.array(points_array),[(0,0),(0,1)],mode='constant')
    x = points_array[:,0]
    y = points_array[:,1]

    # x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    # y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.01  # [m] distance of each interpolated points

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)
    rx, ry = [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
    rxy = np.append([rx],[ry],axis=0).transpose()
    path_object = MarkerBasics(rxy,scale=0.05,pub_topic='/points1')

    cub_traj = cubic_trajectory(sp, max_vel=0.15, max_accel=0.05, max_jerk=0.1)

    # interpolate at several points along the path
    print("total time for execution is {}s".format(cub_traj.total_time))
    times = np.linspace(0, cub_traj.total_time, 1001)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = cub_traj.calc_traj_point(t)

    x_T, y_T = state[0, :], state[1, :]

    some_map = cm.get_cmap('plasma')
    # normalized -1 to 1
    data = state[3,:]
    norm_data = 2.*(data - np.min(data))/np.ptp(data)-1
    my_colors = some_map(norm_data)

    xy_append = np.append([x_T],[y_T],axis=0).transpose()
    linestrip_object = MarkerBasics(xy_append,scale=0.1, pub_topic='/line_strip',rgba_array=my_colors, is_marker_array=False)

        # Lets try some navigation
    global vel_pub
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    linear_val = state[3, :]
    angular_val = state[4, :]
    print('OK')

    my_sch = sched.scheduler(time.time, time.sleep)
    for i, my_time in enumerate(times):
        vel_msg = Twist()
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.linear.x = linear_val[i]
        vel_msg.angular.z = angular_val[i]
        # vel_msg.linear.x = 1.0
        # vel_msg.angular.z = 1.0
        print(i, my_time)
        my_sch.enter(my_time, 1, send_command, argument=(vel_msg,))
    
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    my_sch.enter(times[-1] + 0.1, 1, send_command,argument=(vel_msg,))
    print("done")
    my_sch.run()

    try:
        # markerbasics_object.start()
        # path_object.start()
        linestrip_object.start()
        pass
    except rospy.ROSInterruptException:
        pass


    # rospy.spin()