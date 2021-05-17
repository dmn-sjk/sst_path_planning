#!/usr/bin/env python3

import tf.transformations
from sst.msg import Steering, State
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math
from tf import TransformBroadcaster
from copy import copy


class CarModel:
    def __init__(self):
        self.L = 2.2
        self.max_steering_angle = math.pi / 3
        self.max_lin_vel = 5.0
        self.max_ang_vel_wheel = 5.0

        # rospy.Subscriber('/steering', Steering, self.call_steering, queue_size=1)

        # teleop keyboard
        rospy.Subscriber('/cmd_vel', Twist, self.call_steering, queue_size=1)

        self.state_pub = rospy.Publisher('/state', State, queue_size=1)
        self.front_wheels_pub = rospy.Publisher('/front_wheels', PoseStamped, queue_size=1)
        self.tf_br = TransformBroadcaster()

        self.steer_vector = np.zeros((2, 1))
        self.state = np.zeros((4, 1))

    def call_steering(self, data):
        # self.steer_vector[0] = data.steer_ang_vel
        # self.steer_vector[1] = data.car_lin_vel

        # teleop keyboard
        self.steer_vector[0] = data.angular.z
        self.steer_vector[1] = data.linear.x

    def publish_state(self):
        msg_state = State()
        msg_state.beta = self.state[0]
        msg_state.theta = self.state[1]
        msg_state.x = self.state[2]
        msg_state.y = self.state[3]

        self.state_pub.publish(msg_state)

    def publish_front_wheels(self):
        msg_front_wheels = PoseStamped()
        msg_front_wheels.header.frame_id = 'car'
        msg_front_wheels.header.stamp = rospy.Time.now()
        msg_front_wheels.header.seq = rospy.Time.now()
        msg_front_wheels.pose.position.x = self.L
        msg_front_wheels.pose.position.y = 0
        msg_front_wheels.pose.position.z = 0
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0, 0, copy(self.state[0]))
        msg_front_wheels.pose.orientation.x = qx
        msg_front_wheels.pose.orientation.y = qy
        msg_front_wheels.pose.orientation.z = qz
        msg_front_wheels.pose.orientation.w = qw

        self.front_wheels_pub.publish(msg_front_wheels)

    def send_tf_wheels(self):
        self.tf_br.sendTransform((self.L, 0, 0), tf.transformations.quaternion_from_euler(0, 0, copy(self.state[0])),
                                 rospy.Time.now(), 'front_wheels', 'car')

    def send_tf_car(self):
        translation = (self.state[2], self.state[3], 0)
        self.tf_br.sendTransform(translation, tf.transformations.quaternion_from_euler(0, 0, copy(self.state[1])),
                                 rospy.Time.now(), 'car', 'map')

    def update_state(self, delta_t):
        state_dot = np.array([[1, 0],
                              [0, math.tan(self.state[0]) / self.L],
                              [0, math.cos(self.state[1])],
                              [0, math.sin(self.state[1])]]).dot(self.steer_vector)

        self.state = self.state + state_dot * delta_t

        if self.state[0] > self.max_steering_angle:
            self.state[0] = self.max_steering_angle
        elif self.state[0] < -self.max_steering_angle:
            self.state[0] = -self.max_steering_angle

        rospy.loginfo("State: %f, %f, %f, %f", *self.state)

    def get_new_state(self, state, steer_vector, delta_t):
        state_dot = np.array([[1, 0],
                              [0, math.tan(state[0]) / self.L],
                              [0, math.cos(state[1])],
                              [0, math.sin(state[1])]]).dot(steer_vector)

        new_state = state + state_dot * delta_t

        if new_state[0] > self.max_steering_angle:
            new_state[0] = self.max_steering_angle
        elif new_state[0] < -self.max_steering_angle:
            new_state[0] = -self.max_steering_angle

        return new_state


if __name__ == '__main__':
    try:
        rospy.init_node('car_model')
        rospy.Rate(50)
        car = CarModel()

        prev_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            time = rospy.Time.now().to_sec()
            delta_t = time - prev_time

            car.update_state(delta_t)
            car.send_tf_car()
            car.publish_front_wheels()
            car.publish_state()

            prev_time = time

    except rospy.ROSInterruptException:
        pass
