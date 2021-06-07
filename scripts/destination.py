#!/usr/bin/env python3

import rospy as rp
from visualization_msgs.msg import Marker

class Point:
    def __init__(self, x, y, color):
        self.pub = rp.Publisher('/destination', Marker, queue_size=10)
        self.marker = Marker()

        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rp.Time.now()
        self.marker.ns = 'destination'
        self.marker.id = 0
        self.marker.type = Marker.ARROW
        # self.marker.action = Marker.ADD
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = 0.05

        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.01

        self.marker.color.r = color[0]
        self.marker.color.g = color[1]
        self.marker.color.b = color[2]
        self.marker.color.a = 1.0

    def publish(self):
        self.pub.publish(self.marker)


if __name__ == '__main__':
    rp.init_node('points', log_level=rp.DEBUG)
    destination = Point(10, 6, (1.0, 0.0, 1.0))
    while not rp.is_shutdown():
        destination.publish()
        rp.sleep(0.5)
