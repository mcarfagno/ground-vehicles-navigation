#!/usr/bin/env python3

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from vision_msgs import Detection3D, Detection3DArray, ObjectHypothesisWithPose

import os
import csv
import math
import numpy as np
from numpy import linalg as la

import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

WORLD_FRAME_ID = "world"


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
        self.path_pub = rospy.Publisher("/mpc/path", Path, queue_size=1)
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
            points = [tuple(line) for line in csv.reader(f)]

        return points

    def read_obstacles(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../data/obstacles.csv")

        with open(filename) as f:
            obs = [tuple(line) for line in csv.reader(f)]

        return obs

    def publish_path(self):
        msg = Path()
        msg.header.stamp = rospy.time.now()

        for pt in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            q = quaternion_from_euler((0, 0, pt[2]))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_obstacles(self):
        msg = Detection3DArray()
        msg.header.stamp = rospy.time.now()

        for idx, o in enumerate(self.obstacles):
            obstacle = ObjectHypothesisWithPose()
            obstacle.id = idx
            obstacle.pose.pose.position.x = o[0]
            obstacle.pose.pose.position.y = o[1]

            d = Detection3D()
            d.results.append(obstacle)
            msg.detections.append(d)
        self.obstacles_pub.publish(msg)

        msg_viz = MarkerArray()
        msg_viz.header.stamp = rospy.time.now()
        for idx, o in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = WORLD_FRAME_ID
            marker.header.stamp = rospy.Time.now()

            marker.type = 3
            marker.action = 0
            marker.id = idx
            marker.ns = "mpc/obs"
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 2.0

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker.pose.position.x = o[0]
            marker.pose.position.y = o[1]
            msg_viz.markers.append(marker)
        self.obstacles_viz_pub.publish(msg_viz)

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

            self.publish_path()
            self.publish_obstacles()

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
