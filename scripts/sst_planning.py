#!/usr/bin/env python3

import numpy as np
from car_model import CarModel


class S:
    def __init__(self, state=None, rep=None):
        self.rep = rep
        self.state = state


class SST:
    def __init__(self):
        self.car = CarModel()
        # dobrać
        self.detla_bn = 0.1
        self.detla_s = 0.1
        self.T_prop = 0.5
        self.N = 1000

        self.V_active = [[[0], [0], [0], [0]]]
        self.V_inactive = []
        self.V = [self.V_active, self.V_inactive]

        # E i trajectories mają zgodne indeksy
        self.E = ['a', 'b', 'c']
        self.trajectories = []
        self.parents = [None, 0, 1]

        self.S = [S([[0], [0], [0], [0]], [[0], [0], [0], [0]])]

        # self.G = [self.V, self.E]
        self.G = {}

    def euclidan_dist_norm(self, a, b):
        a_norm = self.normalize_state(a)
        b_norm = self.normalize_state(b)
        sum = 0
        for i in range(len(a)):
            sum += pow(a_norm[i] - b_norm[i], 2)
        return np.sqrt(sum)

    # TODO ew. dodać pętle
    def monte_carlo_prop(self, x_prop):
        t = np.random.uniform(0, self.T_prop)
        ipsylon = np.array([[np.random.uniform(-self.car.max_ang_vel_wheel, self.car.max_ang_vel_wheel)],
                            [np.random.uniform(-self.car.max_lin_vel, self.car.max_lin_vel)]])

        return self.car.get_new_state(x_prop, ipsylon, t), [ipsylon, t]

    def sample_state(self):

        return np.array([[np.random.uniform(-self.car.max_steering_angle, self.car.max_steering_angle)],
                         [np.random.uniform(-self.car.max_theta, self.car.max_theta)],
                         [np.random.uniform(-self.car.max_x, self.car.max_y)],
                         [np.random.uniform(-self.car.max_y, self.car.max_y)]])

    def normalize_state(self, state):
        max_vals = np.array([[self.car.max_steering_angle],
                             [self.car.max_theta],
                             [self.car.max_x],
                             [self.car.max_y]])

        norm_state = (state + max_vals) / (2 * max_vals)
        return norm_state   # [0 - 1]

    def near(self, x_rand):
        near = []
        for x in self.V_active:
            if self.euclidan_dist_norm(x, x_rand) <= self.detla_bn:
                near.append(x)
        return near

    def nearest(self, x_rand):
        nearest = self.V_active[-1]

        for node in self.V_active:
            if self.euclidan_dist_norm(nearest, x_rand) > self.euclidan_dist_norm(node, x_rand):
                nearest = node
        return nearest

    # TODO !!!!!!!!!!!!!!!
    def cost(self, x):
        curr = self.E.index(x)
        cost = 0
        if not curr == 0:
            curr = self.E.index(x)
            while curr is not None:
                cost += 1
                curr = self.parents[curr]
        return cost - 1

    def best_first_selection(self):
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

    # TODO (mapa potrzebna)
    def collisision_free(self, traj):
        pass

    # TODO s_new_rep ogarnąć
    def is_node_locally_the_best(self, x_new):
        s_new = self.nearest(x_new)
        if self.euclidan_dist_norm(s_new, x_new) > self.detla_s:
            self.S.append(x_new)
            s_new = x_new
            s_new_rep = None
        x_peer = s_new_rep

        if x_peer is None or self.cost(x_new) < self.cost(x_peer):
            return True
        return False

    # TODO
    def prune_dominated_nodes(self, x_new):
        s_new = self.nearest(x_new)
        # x_peer = s_new_rep
        x_peer = 0
        if x_peer is not None:
            self.V_active.remove(x_peer)
            self.V_inactive.append(x_peer)
        s_new_rep = x_new
        while x_peer is not None and self.is_leaf(x_peer) and x_peer in self.V_inactive:
            x_parent = self.parent(x_peer)
            self.E.remove(self.euclidan_dist_norm(x_parent, x_peer))
            self.V_inactive.remove(x_peer)
            x_peer = x_parent

    def is_leaf(self, x_peer):
        parents_list = list(self.V.keys())
        if x_peer in parents_list:
            return True
        return False

    def parent(self, x_peer):
        key_list = list(self.V.keys())
        position = list(self.V.values()).index(x_peer)
        return key_list[position]

    def sst_planning(self):
        for i in range(self.N):
            x_selected = self.best_first_selection()
            x_new, ips_t = self.monte_carlo_prop(x_selected)
            if self.collisision_free(x_selected, x_new, ips_t):
                if self.is_node_locally_the_best(x_new):
                    self.V_active.append(x_new)
                    self.E[x_new] = x_selected
                    self.trajectories.append(ips_t)
                    self.prune_dominated_nodes(x_new, self.V_active, self.V_inactive)
        return self.G


if __name__ == '__main__':
    sst = SST()
    print(sst.cost('c'))
    # for i in range(10000):
    #     for j, num in enumerate(sst.normalize_state(sst.sample_state())):
    #         print(num)
