#!/usr/bin/env python3

import math
import os

import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from vision_msgs.msg import Detection3D, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

WORLD_LAT = 40.09302492080515
WORLD_LON = -88.2357551253083
WORLD_FRAME_ID = "world"


def latlon_to_XY(lat0, lon0, lat1, lon1):
    """
    Convert latitude and longitude to global X, Y coordinates,
    using an equirectangular projection.

    X = meters east of lon0
    Y = meters north of lat0

    Sources: http://www.movable-type.co.uk/scripts/latlong.html
                 https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
    """
    R_earth = 6371000  # meters
    delta_lat = math.radians(lat1 - lat0)
    delta_lon = math.radians(lon1 - lon0)

    lat_avg = 0.5 * (math.radians(lat1) + math.radians(lat0))
    X = R_earth * delta_lon * math.cos(lat_avg)
    Y = R_earth * delta_lat

    return X, Y


class MpcEvaluator(object):
    def __init__(self):
        self.mpc_pos_log = []
        self.rate = rospy.Rate(10)

        rospack = rospkg.RosPack()
        mpc_gazebo_dir = rospack.get_path("mpc_gazebo")

        self.waypoints_gps = np.genfromtxt(
            os.path.join(mpc_gazebo_dir, "./data/gps-waypoints.csv"), delimiter=","
        )
        print(self.waypoints_gps)
        self.obstacles = np.genfromtxt(
            os.path.join(mpc_gazebo_dir, "./data/obstacles.csv"), delimiter=","
        )

        # convert the GPS points into World frame
        self.waypoints = np.zeros((len(self.waypoints_gps), 3))
        for i in range(len(self.waypoints_gps)):
            (world_x, world_y) = latlon_to_XY(
                WORLD_LAT, WORLD_LON, self.waypoints_gps[i, 0], self.waypoints_gps[i, 1]
            )
            self.waypoints[i, 0] = world_x
            self.waypoints[i, 1] = world_y

        self.broadcaster = tf.TransformBroadcaster()
        self.odometry_sub = rospy.Subscriber(
            "/gem/base_footprint/odom", Odometry, self.odom_cb, queue_size=1
        )

        self.path_pub = rospy.Publisher("/mpc/path", Path, queue_size=1)
        self.path_viz_pub = rospy.Publisher("/mpc/path/viz", Path, queue_size=1)
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
            (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ),
            rospy.Time.now(),
            msg.child_frame_id,
            msg.header.frame_id,
        )

    def publish_path(self):
        # Path in world frame (for rviz)
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = WORLD_FRAME_ID

        for pt in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_viz_pub.publish(msg)

        # Actual GPS waypoints (for mpc)
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = WORLD_FRAME_ID

        for pt in self.waypoints_gps:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
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
            # CYLINDER
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
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            msg_viz.markers.append(marker)

            # TEXT
            marker = Marker()
            marker.header.frame_id = WORLD_FRAME_ID
            marker.header.stamp = rospy.Time.now()

            marker.text = f"Obstacle \n ID {idx+1}"
            marker.type = 9
            marker.action = 0
            marker.id = len(self.obstacles) + idx
            marker.ns = "mpc/obs"
            marker.scale.z = 1.0

            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.pose.position.x = o[0]
            marker.pose.position.y = o[1]
            marker.pose.position.z = 1.5

            msg_viz.markers.append(marker)
        self.obstacles_viz_pub.publish(msg_viz)

    def get_nearest_index(self, state):
        (curr_x, curr_y, _, _) = state
        dx = [curr_x - x for x in self.waypoints[:, 0]]
        dy = [curr_y - y for y in self.waypoints[:, 1]]
        return int(np.argmin(np.hypot(dx, dy)))

    def compute_cte(self, state):
        (curr_x, curr_y, curr_yaw, _) = state
        target_index = self.get_nearest_index(state)
        dx = curr_x - self.waypoints[target_index, 0]
        dy = curr_y - self.waypoints[target_index, 1]

        front_axle_vec_rot_90 = np.array(
            [[math.cos(curr_yaw - math.pi / 2.0)], [math.sin(curr_yaw - math.pi / 2.0)]]
        )

        vec_target_2_front = np.array([dx, dy])

        return np.dot(vec_target_2_front.T, front_axle_vec_rot_90)

    def plot(self):
        # downsaple a bit, the log is very long and lags
        self.mpc_pos_log = self.mpc_pos_log[::10]

        plt.style.use("seaborn")
        fig = plt.figure()
        spec = fig.add_gridspec(ncols=2, nrows=4)

        # track error
        ax0 = fig.add_subplot(spec[0, :])
        # plt.title("Tracking Error")
        plt.plot(
            [self.compute_cte(x) for x in self.mpc_pos_log],
            label="crosstrack error [m]",
        )
        plt.axhline(y=1.0, color="crimson", linestyle="-", label="maximum cte")
        plt.axhline(y=-1.0, color="crimson", linestyle="-", label="minimum cte")
        plt.xticks([])
        plt.legend()

        # speed
        ax1 = fig.add_subplot(spec[1, :])
        # plt.title("Vehicle Speed")
        plt.plot([x[3] * 3.6 for x in self.mpc_pos_log], label="vehicle speed [km/h]")
        plt.axhline(y=20.0, color="crimson", linestyle="-", label="target speed")
        plt.xticks([])
        plt.legend()

        ax2 = fig.add_subplot(spec[2:, :])
        # plt.title("Vehicle Trajectory")
        plt.plot(
            [x[0] for x in self.waypoints],
            [x[1] for x in self.waypoints],
            color="crimson",
            marker=".",
            label="waypoints",
        )
        for o in self.obstacles:
            obs = plt.Circle((o[0], o[1]), o[2], color="crimson")
            ax2.add_patch(obs)

        plt.plot(
            [x[0] for x in self.mpc_pos_log],
            [x[1] for x in self.mpc_pos_log],
            label="vehicle trajectory",
        )
        plt.axis("equal")
        plt.legend()

        plt.tight_layout()
        plt.show()
        plt.savefig("./mpc_evaluation.png")
        return

    def run(self):
        while not rospy.is_shutdown():
            # check for goal and plot results

            if len(self.mpc_pos_log) > 0:
                if (
                    self.get_nearest_index(self.mpc_pos_log[-1])
                    >= self.waypoints.shape[0] - 3
                ):
                    rospy.loginfo("Goal Reached")
                    self.odometry_sub.unregister()
                    self.plot()

                    rospy.loginfo("Saving Image")
                    rospy.signal_shutdown("Done")

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
