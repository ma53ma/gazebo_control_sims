#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class MarkerBasics(object):
    def __init__(self):
        self.marker_object_pub = rospy.Publisher('/points', MarkerArray, queue_size=1)
        self.rate = rospy.Rate(1)
        self.init_marker_Arr(z_val=0)

    def init_marker_Arr(self, z_val=0):
        self.marker_arr_object = MarkerArray()
        with open('way_points.txt') as file:
            lines = file.readlines()
            lines = [[float(str_val) for str_val in line.rstrip().split()] for line in lines]
        
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
        self.marker_object.scale.x = 0.1
        self.marker_object.scale.y = 0.1
        self.marker_object.scale.z = 0.1

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
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass