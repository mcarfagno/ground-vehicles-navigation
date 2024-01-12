#!/usr/bin/env python3

import csv
import math
import os

import numpy as np
import rospkg
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

WORLD_FRAME_ID = "world"
OBS_RADIUS = 0.5


def dist(p1, p2):
    return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)


def find_angle(v1, v2):
    cosang = np.dot(v1, v2)
    sinang = la.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)


class MpcEvaluator(object):
    def __init__(self):
        self.mpc_pos_log = []
        self.rate = rospy.Rate(10)

        rospack = rospkg.RosPack()
        mpc_gazebo_dir = rospack.get_path("mpc_gazebo")
        self.waypoints = np.genfromtxt(
            os.path.join(mpc_gazebo_dir, "./data/waypoints.csv"), delimiter=","
        )
        self.obstacles = np.genfromtxt(
            os.path.join(mpc_gazebo_dir, "./data/obstacles.csv"), delimiter=","
        )

        self.broadcaster = tf.TransformBroadcaster()
        self.odometry_sub = rospy.Subscriber(
            "/gem/base_footprint/odom", Odometry, self.odom_cb, queue_size=1
        )
        self.path_pub = rospy.Publisher("/mpc/path", Path, queue_size=1)
        self.obstacles_pub = rospy.Publisher(
            "/mpc/obstacles", Detection3DArray, queue_size=1
        )
        self.obstacles_viz_pub = rospy.Publisher(
            "/mpc/obstacles/markers", MarkerArray, queue_size=1
        )

    def odom_cb(self, msg):
        (_, _, yaw) = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        self.mpc_pos_log.append(
            (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, speed)
        )

        # Broadcast latest world -> base_link tf to simulate localization stack
        self.broadcaster.sendTransform(
            (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ),
            tf.transformations.quaternion_from_euler(0, 0, yaw),
            rospy.Time.now(),
            msg.child_frame_id,
            msg.header.frame_id,
        )

    def publish_path(self):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = WORLD_FRAME_ID

        for pt in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            q = quaternion_from_euler(0, 0, pt[2])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_obstacles(self):
        msg = Detection3DArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = WORLD_FRAME_ID

        for idx, o in enumerate(self.obstacles):
            d = Detection3D()
            # NOTE: obstacles have the form x,y,radius
            # I am misusing the bbox size for the radius
            d.bbox.center.position.x = o[0]
            d.bbox.center.position.y = o[1]
            d.bbox.size.x = o[2]
            msg.detections.append(d)

        self.obstacles_pub.publish(msg)

        msg_viz = MarkerArray()
        for idx, o in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = WORLD_FRAME_ID
            marker.header.stamp = rospy.Time.now()

            marker.type = 3
            marker.action = 0
            marker.id = idx
            marker.ns = "mpc/obs"
            marker.scale.x = o[2] * 2
            marker.scale.y = o[2] * 2
            marker.scale.z = 1.0

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.6
            marker.pose.position.x = o[0]
            marker.pose.position.y = o[1]
            marker.pose.position.z = marker.scale.z * 0.5

            msg_viz.markers.append(marker)
        self.obstacles_viz_pub.publish(msg_viz)

    def get_nearest_index(self, robot_state):
        (curr_x, curr_y, _, _) = robot_state
        dx = [curr_x - x for x in self.waypoints[:, 0]]
        dy = [curr_y - y for y in self.waypoints[:, 1]]
        return int(np.argmin(np.hypot(dx, dy)))

    def compute_cte(self, robot_state):
        (curr_x, curr_y, curr_yaw, _) = robot_state
        target_index = self.get_nearest_index(curr_x, curr_y)
        dx = curr_x - self.waypoints[target_index, 0]
        dy = curr_y - self.waypoints[target_index, 1]

        front_axle_vec_rot_90 = np.array(
            [[math.cos(curr_yaw - math.pi / 2.0)], [math.sin(curr_yaw - math.pi / 2.0)]]
        )

        vec_target_2_front = np.array([dx, dy])

        return np.dot(vec_target_2_front.T, front_axle_vec_rot_90)

    # TODO: plot MPC results at the end of trial
    def plot(self):
        return

    def run(self):
        while not rospy.is_shutdown():
            # check for goal and plot results

            if len(self.mpc_pos_log) > 0:
                nearest = self.get_nearest_index(self.mpc_pos_log[-1])
                cte = self.compute_cte(self.mpc_pos_log[-1])
                rospy.loginfo(f"nearest: {nearest} error: {cte}")
                if nearest == self.waypoints.shape[0] - 1:
                    rospy.loginfo(f"Goal Reached")
                    # TODO: plot

                    rospy.loginfo(f"Saving Image to")
                    rospy.shutdown()

            self.publish_path()
            self.publish_obstacles()

            self.rate.sleep()


def mpc_sim():
    rospy.init_node("mpc_sim_node", anonymous=True)
    mpc_eval = MpcEvaluator()

    try:
        mpc_eval.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    mpc_sim()
