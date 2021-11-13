#!/usr/bin/env python

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    from cubic_spline_planner import Spline2D
except ImportError:
    raise

class MarkerBasics(object):
    def __init__(self, lines, scale=0.1,pub_topic='/points'):
        self.marker_object_pub = rospy.Publisher(pub_topic, MarkerArray, queue_size=1)
        self.rate = rospy.Rate(1)
        self.scale = scale
        self.init_marker_Arr(lines, z_val=0)

    def init_marker_Arr(self,lines, z_val=0):
        self.marker_arr_object = MarkerArray()

        self.array_length = len(lines)
        for index, coordinate in enumerate(lines):
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

if __name__ == '__main__':
    rospy.init_node('points_vis_node', anonymous=True)
    with open('way_points.txt') as file:
        lines = file.readlines()
        lines = [[float(str_val) for str_val in line.rstrip().split()] for line in lines]
    markerbasics_object = MarkerBasics(lines)

    # Cubic Spline fit and display
    # make it a numpy array and extend second dimensions with zeros for theta
    lines = np.pad(np.array(lines),[(0,0),(0,1)],mode='constant')
    x = lines[:,0]
    y = lines[:,1]

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


    try:
        markerbasics_object.start()
        path_object.start()
    except rospy.ROSInterruptException:
        pass