from copy import copy
import numpy as np
import rospy as rp
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
import tf

class Map(object):
    def __init__(self):
        self.map = None
        self.width = 0
        self.height = 0
        self.resolution = 0

        self.destination = None

        rp.Subscriber('/map', OccupancyGrid, self.map_callback)
        rp.Subscriber('/destination', Marker, self.set_destination)
        self.search_pub = rp.Publisher('/search', Marker, queue_size=10)
        self.route_pub = rp.Publisher('/route', Marker, queue_size=10)
        self.nodes_pub = rp.Publisher('/nodes', MarkerArray, queue_size=10)

    def map_callback(self, data):
        self.map = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.resolution = data.info.resolution

    def set_destination(self, data):
        quaternion = (data.pose.orientation.x,
                      data.pose.orientation.y,
                      data.pose.orientation.z,
                      data.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.destination = np.array([[0], [euler[2]], [data.pose.position.x], [data.pose.position.y]])

    def publish_search(self, E, all_points=False):
        marker = Marker()
        def add_point(p):
            pt = Point()
            pt.x = p[2]
            pt.y = p[3]
            pt.z = 0.
            marker.points.append(pt)
        marker.header.frame_id = "map"
        marker.header.stamp = rp.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.color.r = 0.
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 0.5
        marker.scale.x = 0.05
        for e in E:
            if all_points:
                for i, a in enumerate(e):
                    if i < len(e) - 1:
                        add_point(e[i])
                        add_point(e[i + 1])
            else:
                add_point(e[0])
                add_point(e[-1])
        self.search_pub.publish(marker)

    def publish_nodes(self, V_active, V_inactive):
        marker_array = MarkerArray()
        for i, node in enumerate(V_active):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = i + 1
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = node[2]
            marker.pose.position.y = node[3]
            marker.pose.position.z = 0
            marker_array.markers.append(marker)
        for i, node in enumerate(V_inactive):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = len(marker_array.markers) - 1 + i
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = node[2]
            marker.pose.position.y = node[3]
            marker.pose.position.z = 0
            marker_array.markers.append(marker)
        self.nodes_pub.publish(marker_array)

    def publish_route(self, route, all_points=False):
        marker = Marker()
        def add_point(p):
            pt = Point()
            pt.x = p[2]
            pt.y = p[3]
            pt.z = 0.
            marker.points.append(pt)
        marker.header.frame_id = "map"
        marker.header.stamp = rp.Time.now()
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.color.r = 1.
        marker.color.g = 0.
        marker.color.b = 0.
        marker.color.a = 1.
        marker.scale.x = 0.2
        for e in route:
            if all_points:
                for i, a in enumerate(e):
                    if i < len(e) - 1:
                        add_point(e[i])
                        add_point(e[i + 1])
            else:
                add_point(e[0])
                add_point(e[-1])
        self.route_pub.publish(marker)
