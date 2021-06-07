#!/usr/bin/env python3

from sst.msg import Steering, State
import rospy as rp
import numpy as np
from sst_planning import SST
from car_model import CarModel
from sst.msg import Steering, State


class Simulation(SST):
    def __init__(self, N=2000, delta_bn=15, delta_s=0.5, T_prop=0.5):
        super(Simulation, self).__init__(N, delta_bn, delta_s, T_prop)
        self.car = CarModel()
        self.steer_pub = rp.Publisher('/steering', Steering, queue_size=10)

    def get_route(self, G):
        print('Choosing route...')
        controls = []
        route = []
        closest = self.get_closest_to_dest(G)
        curr = closest
        i = 0
        while not np.array_equal(np.zeros((4, 1), dtype=int), curr):
            if np.array_equal(curr, G.E[i][-1]):
                route.append(G.E[i])
                controls.append(G.trajectory[i])
                curr = G.E[i][0]
                i = 0
            else:
                i += 1
        print('Found route!')

        controls.reverse()
        route.reverse()
        return controls,  route

    def get_closest_to_dest(self, G):
        closest, ind = self.nearest(self.destination, G.V_active)
        return closest

    def publish_control(self, ips):
        control = Steering()
        control.car_lin_vel = ips[1]
        control.steer_ang_vel = ips[0]
        self.steer_pub.publish(control)

if __name__ == '__main__':
    try:
        rp.init_node('simulation')
        rp.Rate(50)
        sim = Simulation(1000, 15, 0.5, 0.5)
        G = sim.sst_planning()
        controls, route = sim.get_route(G)

        i = 0
        prev_time = rp.Time.now().to_sec()
        while not rp.is_shutdown():
            sim.publish_route(route, all_points=True)
            time = rp.Time.now().to_sec()
            delta_t = time - prev_time
            print(delta_t, controls[i][1])

            if delta_t >= controls[i][1]:
                sim.publish_control(controls[i][0])
                i += 1
                prev_time = time
                if i == len(controls):
                    control = [0.0, 0.0]
                    sim.publish_control(control)
                    break
    except rp.ROSInterruptException:
        pass
