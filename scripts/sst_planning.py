#!/usr/bin/env python3

from warnings import catch_warnings
import numpy as np
from car_model import CarModel
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import rospy as rp
# import kdtree
from copy import copy
from time import time
from sst.msg import Steering


def remove_array(L, arr):
    ind = 0
    size = len(L)
    while ind != size and not np.array_equal(L[ind], arr):
        ind += 1
    if ind != size:
        L.pop(ind)
        return L
    else:
        raise ValueError('array not found in list.')


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
    def __init__(self, N=1000, delta_bn=15, delta_s=0.5, T_prop=0.5, map=None): # delta_bn=0.5, delta_s=0.5, T_prop=0.2
        self.car = CarModel()
        # dobrać
        self.detla_bn = delta_bn
        self.detla_s = delta_s
        self.T_prop = T_prop
        self.N = N

        self.G = G()

        self.G.V_active = [np.array([[0], [0], [0], [0]])]
        self.cost_active = [0]
        self.G.V_inactive = []

        self.G.E = []

        self.S = [S(np.array([[0], [0], [0], [0]]), np.array([[0], [0], [0], [0]]))]
        self.cost_s = [0]
        if map is None:
            self.map_sub = rp.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.search_pub = rp.Publisher('/search', Marker, queue_size=10)
        self.nodes_pub = rp.Publisher('/nodes', MarkerArray, queue_size=100)
        self.destination_pub = rp.Publisher('/destination', Marker, queue_size=1)

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map = None

    def map_callback(self, data):
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map = data.data
        self.map_resolution = data.info.resolution

    def el_in_array(self, el, array):
        for node in array:
            if np.array_equal(el, node):
                return True
        return False

    def euclidan_dist_norm_old(self, a, b, norm=False):
        if norm:
            a = self.normalize_state(a)
            b = self.normalize_state(b)

        sum = 0
        for i in range(len(a)):
            sum += pow(a[i] - b[i], 2)
        return np.sqrt(sum)


    def euclidan_dist_norm(self, a, b, norm=False):
        if norm:
            a = self.normalize_state(a)
            b = self.normalize_state(b)

        # par = np.array([[np.sin(a[1] - b[1])],
        #                 [np.cos(a[1] - b[1])],
        #                 [1]])
        par = np.ones((3, 1))
        return np.sqrt(np.sum((par * (a[1:] - b[1:])) ** 2))

    def monte_carlo_prop(self, x_prop):
        t = np.random.uniform(0, self.T_prop)
        ipsylon = np.array([[np.random.uniform(-self.car.max_ang_vel_wheel, self.car.max_ang_vel_wheel)],
                            [np.random.uniform(0, self.car.max_lin_vel)]])

        x = [x_prop]
        traj = [ipsylon, t/10*10]

        for i in range(10):
            x.append(self.car.get_new_state(x[-1], ipsylon, t/10))

        return x, traj

    def sample_state(self):

        return np.array([[np.random.uniform(-self.car.max_steering_angle, self.car.max_steering_angle)],
                         [np.random.uniform(0, self.car.max_theta)],
                         [np.random.uniform(-self.car.max_x, self.car.max_y)],
                         [np.random.uniform(-self.car.max_y, self.car.max_y)]])

    def normalize_state(self, state):
        max_vals = np.array([[self.car.max_steering_angle],
                             [self.car.max_theta],
                             [self.car.max_x],
                             [self.car.max_y]])

        norm_state = (state + max_vals[4-state.shape[0]:]) / (2 * max_vals[4-state.shape[0]:])
        return norm_state   # [0 - 1]

    def near(self, x_rand):
        near = []
        ind = []
        for i, x in enumerate(self.G.V_active):
            if self.euclidan_dist_norm_old(x[1:], x_rand[1:]) <= self.detla_bn:
                near.append(x)
                ind.append(i)
        return near, ind

    def nearest(self, x_rand, var):
        if isinstance(var[0], np.ndarray):
            nearest = var[-1]
            nearest_i = len(var) - 1
            for i, node in enumerate(var):
                if self.euclidan_dist_norm_old(nearest[2:], x_rand[2:]) > self.euclidan_dist_norm_old(node[2:], x_rand[2:]):
                    nearest = node
                    nearest_i = i

            return nearest, nearest_i

        elif isinstance(var[0], S):
            nearest = var[-1].state
            entire_s = var[-1]
            ind = len(var) - 1
            for i, node in enumerate(var):
                if self.euclidan_dist_norm_old(nearest[1:], x_rand[1:]) > self.euclidan_dist_norm_old(node.state[1:], x_rand[1:]):
                    nearest = node.state
                    entire_s = node
                    ind = i

            return entire_s, ind
        else:
            raise Exception ('var argument in nearest function not ndarray or S class object') 

    def best_first_selection(self):
        x_rand = self.sample_state()
        x_near, inds = self.near(x_rand)
        if len(x_near) == 0:
            return self.nearest(x_rand, self.G.V_active)
        else:
            best_i = inds[0]
            for i in inds:
                if self.cost_active[i] < self.cost_active[best_i]:
                    best_i = i
            return self.G.V_active[best_i], best_i

    def collision_free(self, traj):
        for node in traj:
            x = int(node[2] / self.map_resolution) + self.map_width // 2
            y = int(node[3] / self.map_resolution) + self.map_height // 2
            if self.map[x + y * self.map_width] == 100:
                return False
        return True

    def is_node_locally_the_best(self, x_new, ind_selected, dist):
        start = time()
        s, ind = self.nearest(x_new, self.S)
        s_new = S()
        s_new.state = np.copy(s.state)
        s_new.rep = copy(s.rep)

        if self.euclidan_dist_norm(s_new.state, x_new) > self.detla_s:
            s_new.state = x_new
            s_new.rep = None
            self.S.append(s_new)
            self.cost_s.append(None)

        x_peer = s_new.rep
        if x_peer is not None:
            x_peer_cost = self.cost_s[ind]

        if x_peer is None or self.cost_active[ind_selected] + dist < x_peer_cost:
            # print(f'is_node_locally_the_best: {time() - start} s')
            return True
        # print(f'is_node_locally_the_best: {time() - start} s')
        return False

    def prune_dominated_nodes(self, x_new, x_new_cost):
        start = time()
        s, ind = self.nearest(x_new, self.S)
        s_new = S()
        s_new.state = np.copy(s.state)
        s_new.rep = copy(s.rep)
        # for s in self.S:
        #     comp = s.state == s_new.state
        #     if comp.all():
        x_peer = s_new.rep
        if x_peer is not None:
            for i, act in enumerate(self.G.V_active):
                if np.array_equal(x_peer, act):
                    self.G.V_active.pop(i)
                    self.cost_active.pop(i)
                    break
            self.G.V_inactive.append(x_peer)

        self.S[ind].rep = x_new
        self.cost_s[ind] = x_new_cost

        while x_peer is not None and self.is_leaf(x_peer) and self.el_in_array(x_peer, self.G.V_inactive):
            x_parent = self.parent(x_peer)
            for i, e in enumerate(self.G.E):
                if np.array_equal(e[0], x_parent) and np.array_equal(e[-1], x_peer):
                    self.G.E.pop(i)
                    self.G.trajectory.pop(i)
                    break
            self.G.V_inactive = remove_array(self.G.V_inactive, x_peer)
            x_peer = x_parent

    def is_leaf(self, x_peer):
        start = time()
        leaf_list = []
        to_remove = []
        for e in self.G.E:
            leaf_list.append(e[-1])
            to_remove.append(e[0])
        for el in to_remove:
            try:
                leaf_list = remove_array(leaf_list, el)
            except:
                pass
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
            print(i)
            start = time()
            x_selected, ind_selected = self.best_first_selection()
            # print(f'best_first_selection: {time() - start} s')

            start = time()
            x, ips_t = self.monte_carlo_prop(x_selected)
            # print(f'monte_carlo_prop: {time() - start} s')

            x_new = x[-1]
            dist = self.euclidan_dist_norm(x[0], x_new)
            if self.collision_free(x):
                if self.is_node_locally_the_best(x_new, ind_selected, dist):


                    self.G.V_active.append(x_new)
                    self.cost_active.append(self.cost_active[ind_selected] + dist)
                    self.G.E.append(x)
                    self.G.trajectory.append(ips_t)

                    start = time()
                    self.prune_dominated_nodes(x_new, self.cost_active[ind_selected] + dist)
                    # print(f'prune_dominated_nodes: {time() - start} s')

    
                    self.publish_search(self.G.E)
                    self.publish_nodes(self.G.V_active, self.G.V_inactive)
                    self.publish_destination()
                    # print(len(self.G.V_inactive))
                    # print(f'Edges: {len(self.G.E)}')
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
        marker.scale.x = 0.05
        for e in E:
            for i, a in enumerate(e):
                if i < len(e) - 1:
                    add_point(e[i])
                    add_point(e[i + 1])
            # add_point(e[0])
            # add_point(e[-1])
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

    def publish_destination(self):
        destination = Marker()
        destination.header.frame_id = 'map'
        destination.action = destination.ADD
        destination.type = destination.ARROW
        destination.id = 0
        destination.pose.position.x = 10.0
        destination.pose.position.y = 6.0
        destination.pose.position.z = 0
        destination.color.a = 1.0
        destination.color.r = 1.0
        destination.color.g = 0.0
        destination.color.b = 1.0
        destination.scale.x = 0.5
        destination.scale.y = 0.5
        destination.scale.z = 0.1
        destination.pose.orientation.w = 1.0
        destination.pose.orientation.x = 0.0
        destination.pose.orientation.y = 0.0
        destination.pose.orientation.z = 0.0
        self.destination_pub.publish(destination)


if __name__ == '__main__':
    sst = SST()
    steer_pub = rp.Publisher('/steering', Steering)
    route_pub = rp.Publisher('/route', Marker, queue_size=1)
    try:
        rp.init_node('search')
        rp.Rate(50)
        G = sst.sst_planning()
        print('end')
        closest, ind = sst.nearest(np.array([[0],
                                        [0],
                                        [10],
                                        [6]]), G.V_active)
        curr = closest
        i = 0
        route = []

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = marker.ADD
        marker.type = marker.ARROW
        marker.id = 2
        marker.pose.position.x = closest[2]
        marker.pose.position.y = closest[3]
        marker.pose.position.z = 0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        route_pub.publish(marker)

        while not np.array_equal(np.zeros((4, 1), dtype=int), curr):
            if np.array_equal(curr, G.E[i][-1]):


                route.append(G.trajectory[i])
                curr = G.E[i][0]
                i = 0
            else:
                i += 1
        print('found route')

        route.reverse()
        i = 0
        prev_time = rp.Time.now().to_sec()
        while not rp.is_shutdown():
            route_pub.publish(marker)
            sst.publish_destination()

            delta_t = rp.Time.now().to_sec() - prev_time
            if delta_t >= route[i][1]:
                steer = Steering()
                steer.steer_ang_vel = route[i][0][0]
                steer.car_lin_vel = route[i][0][1]
                steer_pub.publish(steer)
                i += 1
                prev_time = rp.Time.now().to_sec()
                if i == len(route):
                    steer.steer_ang_vel = 0.0
                    steer.car_lin_vel = 0.0
                    steer_pub.publish(steer)
                    break
    except rp.ROSInterruptException:
        pass
