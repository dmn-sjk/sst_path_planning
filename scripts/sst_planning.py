#!/usr/bin/env python3

import numpy as np
from car_model import CarModel


class SST:
    def __init__(self):
        self.car = CarModel()

    def monte_carlo_prop(self, x_prop, T_prop):
        t = np.random.uniform(0, T_prop)
        ipsylon = np.array([[np.random.uniform(-self.car.max_ang_vel_wheel, self.car.max_ang_vel_wheel)],
                            [np.random.uniform(-self.car.max_lin_vel, self.car.max_lin_vel)]])

        return self.car.get_new_state(x_prop, ipsylon, t)

    def best_first_selection(self, X, V_active, delta_bn):
        pass


if __name__ == '__main__':
    sst = SST()
