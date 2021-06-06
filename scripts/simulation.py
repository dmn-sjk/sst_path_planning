#!/usr/bin/env python3

import tf.transformations
from sst.msg import Steering, State
import rospy as rp
from geometry_msgs.msg import Twist, PoseStamped, Point
import numpy as np
from tf import TransformBroadcaster
from sst_planning import SST
from visualization_msgs.msg import Marker, MarkerArray
from car_model import CarModel


class Simulation:
    def __init__(self):
        self.sst = SST()
        self.car = CarModel()
        destination_pub = rp.Publisher('/destination', Marker, queue_size=100)
        destination = Marker()
        destination.header.frame_id = 'map'
        destination.type = Marker.CUBE
        destination.id = 0
        destination.pose.position.x = 5
        destination.pose.position.y = 5
        destination.pose.position.z = 0
        destination.color.a = 1.0
        destination.color.r = 0.0
        destination.color.g = 0.0
        destination.color.b = 1.0
        destination_pub.publish()

    def steer(self, ):
        pass

if __name__ == '__main__':
    try:
        rp.init_node('simulation')
        rp.Rate(50)
        sim = Simulation()
        G = sim.sst.sst_planning()
        while not rp.is_shutdown():
            continue

        # path = []
        # curr = G.E[-1][-1]
        # while not path[-1] == 0:
        #     for e in G.E:
        #         path.append()
        #
        # prev_time = rospy.Time.now().to_sec()
        # while not rospy.is_shutdown():
        #     for steer, time in G.trajectory:
        #         sim.car.steer_vector = steer
        #         time = rospy.Time.now().to_sec()
        #         delta_t = time - prev_time
        #
        #         car.update_state(delta_t)
        #         car.send_tf_car()
        #         car.publish_front_wheels()
        #         car.publish_state()
        #
        #         prev_time = time


    except rp.ROSInterruptException:
        pass
