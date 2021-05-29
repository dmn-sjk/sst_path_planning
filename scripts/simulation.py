#!/usr/bin/env python3

import tf.transformations
from sst.msg import Steering, State
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
import numpy as np
import math
from tf import TransformBroadcaster
from sst_planning import SST
from visualization_msgs.msg import Marker, MarkerArray
from car_model import CarModel


class Simulation:
    def __init__(self):
        self.sst = SST()
        self.car = CarModel()

    def steer(self, ):


if __name__ == '__main__':
    try:
        rospy.init_node('simulation')
        rospy.Rate(50)
        sim = Simulation()
        G = sim.sst.sst_planning()

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


    except rospy.ROSInterruptException:
        pass
