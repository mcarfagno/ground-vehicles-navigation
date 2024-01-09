#!/usr/bin/env python3

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from vision_msgs import Detection3D, Detection3DArray

import os
import csv
import math
import numpy as np
from numpy import linalg as la

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def dist(p1, p2):
    return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)


def find_angle(v1, v2):
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)


class MpcLogger(object):
    def __init__(self):
        self.cte_log = []
        self.rate = rospy.Rate(10)
        self.waypoints = self.read_waypoints()
        self.obstacles = self.read_obstacles()

        self.odometry_sub = rospy.Subscriber("/gem/odometry", Odometry, queue_size=1)
        self.trajectory_pub = rospy.Publisher("/mpc/trajectory", Path, queue_size=1)
        self.obstacles_pub = rospy.Publisher(
            "/mpc/obstacles", Detection3DArray, queue_size=1
        )
        self.obstacles_viz_pub = rospy.Publisher(
            "/mpc/obstacles/markers", MarkerArray, queue_size=1
        )

    def read_waypoints(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../data/waypoints.csv")

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        return path_points

    def read_obstacles(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../data/obstacles.csv")

        with open(filename) as f:
            obs = [tuple(line) for line in csv.reader(f)]

        return obs

    def compute_cte(self, curr_x, curr_y, curr_yaw):
        dx = [front_x - x for x in self.waypoints[:, 0]]
        dy = [front_y - y for y in self.waypoints[:, 1]]
        target_index = int(np.argmin(np.hypot(dx, dy)))

        front_axle_vec_rot_90 = np.array(
            [[math.cos(curr_yaw - math.pi / 2.0)], [math.sin(curr_yaw - math.pi / 2.0)]]
        )

        vec_target_2_front = np.array([[dx[target_index]], [dy[target_index]]])

        return np.dot(vec_target_2_front.T, front_axle_vec_rot_90)

    # TODO: plot MPC results
    def plot(self):
        return

    def run(self):
        while not rospy.is_shutdown():
            # TODO: check for goal and plot results

            # TODO: publish Path and Obstacles

            self.rate.sleep()


def mpc_sim():
    rospy.init_node("mpc_sim_node", anonymous=True)
    mpc = MpcLogger()

    try:
        mpc.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    mpc_sim()
