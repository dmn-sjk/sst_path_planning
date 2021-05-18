#!/usr/bin/env python3

import numpy as np
from car_model import CarModel


class SST:
    def __init__(self):
        self.car = CarModel()
        # dobrać
        self.detla_bn = 0.1
        self.detla_s = 0.1
        self.T_prop = 0.5

        self.V_active = []
        self.V_inactive = []

    def euclidan_dist(self, a, b):
        sum = 0
        for i in range(len(a)):
            sum += pow(a[i] - b[i], 2)
        return np.sqrt(sum)

    def monte_carlo_prop(self, x_prop):
        t = np.random.uniform(0, self.T_prop)
        ipsylon = np.array([[np.random.uniform(-self.car.max_ang_vel_wheel, self.car.max_ang_vel_wheel)],
                            [np.random.uniform(-self.car.max_lin_vel, self.car.max_lin_vel)]])

        return self.car.get_new_state(x_prop, ipsylon, t)

    def sample_state(self):
        return np.array([[np.random.uniform(-self.car.max_steering_angle, self.car.max_steering_angle)],
                         [np.random.uniform(-self.car.max_theta, self.car.max_theta)],
                         [np.random.uniform(-self.car.max_x, self.car.max_y)],
                         [np.random.uniform(-self.car.max_y, self.car.max_y)]])

    def near(self, x_rand):
        near = []
        for x in V_active:
            if self.euclidan_dist(x, x_rand) <= self.detla_bn:
                near.append(x)
        return near

    # TODO
    def nearest(self, x_rand):
        nearest = None
        return nearest

    # TODO
    def cost(self, x):
        cost = 0
        return cost

    # TODO
    def best_first_selection(self, X, V_active, delta_bn):
        x_rand = self.sample_state()
        x_near = self.near(x_rand)
        if len(x_near) == 0:
            return self.nearest(x_rand)
        else:
            best = x_near[0]
            for x in x_near:
                if self.cost(x) < self.cost(best):
                    best = x
            return best


if __name__ == '__main__':
    sst = SST()
