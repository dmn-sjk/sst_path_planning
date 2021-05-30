#!/usr/bin/env python3

from warnings import catch_warnings
import numpy as np
from car_model import CarModel
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy as rp
import kdtree
from copy import copy


class S:
    def __init__(self, state=None, rep=None):
        self.rep = rep
        self.state = state


class G:
    def __init__(self):
        self.V_active = []
        self.V_inactive = []
        self.E = []
        self.trajectory = []
        self.parents = []


class SST:
    def __init__(self, N=1000, delta_bn=0.01, delta_s=0.1, T_prop=0.5):
        self.car = CarModel()
        # dobrać
        self.detla_bn = delta_bn
        self.detla_s = delta_s
        self.T_prop = T_prop
        self.N = N

        self.G = G()

        self.G.V_active = [np.array([[0], [0], [0], [0]])]
        self.G.V_inactive = []

        self.G.E = []

        self.S = [S(np.array([[0], [0], [0], [0]]), np.array([[0], [0], [0], [0]]))]

        self.search_pub = rp.Publisher('/search', Marker, queue_size=10)

    def euclidan_dist_norm(self, a, b):
        a_norm = self.normalize_state(a)
        b_norm = self.normalize_state(b)
        # a_norm = a
        # b_norm = b
        sum = 0
        for i in range(len(a)):
            sum += pow(a_norm[i] - b_norm[i], 2)

        return np.sqrt(sum)

    def monte_carlo_prop(self, x_prop):
        t = np.random.uniform(0, self.T_prop)
        ipsylon = np.array([[np.random.uniform(-self.car.max_ang_vel_wheel, self.car.max_ang_vel_wheel)],
                            [np.random.uniform(-self.car.max_lin_vel, self.car.max_lin_vel)]])

        x = [x_prop]
        traj = [[ipsylon, t]]

        for i in range(10):
            x.append(self.car.get_new_state(x[-1], ipsylon, t/10))

        return x, traj

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
        for x in self.G.V_active:
            if self.euclidan_dist_norm(x, x_rand) <= self.detla_bn:
                near.append(x)
        return near

    def nearest(self, x_rand, var):
        if isinstance(var[0], np.ndarray):
            nearest = var[-1]

            for node in var:
                if self.euclidan_dist_norm(nearest, x_rand) > self.euclidan_dist_norm(node, x_rand):
                    nearest = node
            return nearest
        elif isinstance(var[0], S):
            nearest = var[-1].state
            entire_s = var[-1]
            ind = len(var) - 1
            for i, node in enumerate(var):
                if self.euclidan_dist_norm(nearest, x_rand) > self.euclidan_dist_norm(node.state, x_rand):
                    nearest = node.state
                    entire_s = node
                    ind = i
            return entire_s, ind
        else:
            raise Exception ('var argument in nearest function not ndarray or S class object') 

    def cost(self, x):
        cost = 0
        parent = x
        k = 0
        if len(self.G.E) == 0:
            return 0
        else:
            while not np.array_equal(parent, self.G.E[0][0]):
                for i, e in enumerate(self.G.E):
                    k += 1
                    if k > 200:
                        # exit()
                        pass
                    print(f'i: {i}')
                    print(f'cost: {cost}')
                    #print('\n\n')
                    # if np.array_equal(e[-1], parent):
                    if (np.abs(e[-1] - parent) <= 1e-6).all():
                        parent = e[0]
                        cost += 1
                        k = 0
                        if np.array_equal(parent, self.G.E[0][0]):
                            break
        return cost

    def best_first_selection(self):
        x_rand = self.sample_state()
        x_near = self.near(x_rand)
        if len(x_near) == 0:
            return self.nearest(x_rand, self.G.V_active)
        else:
            best = x_near[0]
            for x in x_near:
                if self.cost(x) < self.cost(best):
                    best = x
            return best

    # TODO (mapa potrzebna)
    def collision_free(self, traj, x_new, ips_t):
        return True

    def is_node_locally_the_best(self, x_new, x_selected):
        s, ind = self.nearest(x_new, self.S)
        s_new = S()
        s_new.state = np.copy(s.state)
        s_new.rep = copy(s.rep)
        # for s in self.S:
        #     comp = s.state == s_new.state
        #     if comp.all():
        #         s_new.rep = s.rep

        if self.euclidan_dist_norm(s_new.state, x_new) > self.detla_s:
            s_new.state = x_new
            s_new.rep = None
            self.S.append(s_new)

        x_peer = s_new.rep

        if x_peer is None or self.cost(x_selected) + 1 < self.cost(x_peer):
            return True
        return False

    def prune_dominated_nodes(self, x_new):
        s, ind = self.nearest(x_new, self.S)
        s_new = S()
        s_new.state = np.copy(s.state)
        s_new.rep = copy(s.rep)
        # for s in self.S:
        #     comp = s.state == s_new.state
        #     if comp.all():
        x_peer = s_new.rep
        # print(x_peer)
        if x_peer is not None:
            print('should delete')
            for i, act in enumerate(self.G.V_active):
                if np.array_equal(x_peer, act):
                    self.G.V_active.pop(i)
                    print('deleted from active')
            self.G.V_inactive.append(x_peer)

        self.S[ind].rep = x_new

        while x_peer is not None and self.is_leaf(x_peer) and x_peer in self.G.V_inactive:
            x_parent = self.parent(x_peer)
            print('should prune')
            for i, e in enumerate(self.G.E):
                if np.array_equal(e[0], x_parent) and np.array_equal(e[-1], x_peer):
                    print('pruned\n')
                    self.G.E.pop(i)
                    self.G.trajectory.pop(i)
            self.G.V_inactive.remove(x_peer)
            x_peer = x_parent

    def is_leaf(self, x_peer):
        leaf_list = []
        for e in self.G.E:
            leaf_list.append(e[-1])
        for leaf in leaf_list:
            if np.array_equal(x_peer, leaf):
                return True
        return False

    def parent(self, x_peer):
        for e in self.G.E:
            if np.array_equal(e[-1], x_peer):
                return e[0]

    def sst_planning(self):
        for i in range(self.N):
            # print(i)
            x_selected = self.best_first_selection()
            x, ips_t = self.monte_carlo_prop(x_selected)
            x_new = x[-1]
            if self.collision_free(x_selected, x_new, ips_t):
                if self.is_node_locally_the_best(x_new, x_selected):
                    self.G.V_active.append(x_new)
                    self.G.E.append(x)
                    self.G.trajectory.append(ips_t)
                    self.prune_dominated_nodes(x_new)
                    self.publish_search(self.G.E)
                    print(f'Edges: {len(self.G.E)}')
        return self.G

    def publish_search(self, E):
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
        marker.scale.x = 0.1 * 0.5
        for e in E:
            # for i, a in enumerate(e):
            #     if i < len(e) - 1:
            #         add_point(e[i])
            #         add_point(e[i + 1])
            add_point(e[0])
            add_point(e[-1])
        self.search_pub.publish(marker)


if __name__ == '__main__':
    sst = SST()
    try:
        rp.init_node('search')
        rp.Rate(50)
        G = sst.sst_planning()
        print('end')
        while not rp.is_shutdown():
            continue
    except rp.ROSInterruptException:
        pass
