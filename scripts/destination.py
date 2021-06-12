#!/usr/bin/env python3

import rospy as rp
from visualization_msgs.msg import Marker

class Point:
    def __init__(self, x, y, qx, qy, qz, qw, color):
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

        self.marker.pose.orientation.x = qx
        self.marker.pose.orientation.y = qy
        self.marker.pose.orientation.z = qz
        self.marker.pose.orientation.w = qw
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
    rp.init_node('destination', log_level=rp.DEBUG)
    x = rp.get_param('~x')
    y = rp.get_param('~y')
    qx = rp.get_param('~qx')
    qy = rp.get_param('~qy')
    qz = rp.get_param('~qz')
    qw = rp.get_param('~qw')
    destination = Point(x, y, qx, qy, qz, qw, (1.0, 0.0, 1.0))
    while not rp.is_shutdown():
        destination.publish()
        rp.sleep(0.5)
