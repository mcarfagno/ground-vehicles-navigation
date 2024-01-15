#!/usr/bin/env python3

import atexit
from os.path import expanduser
from time import gmtime, strftime

import rospy
from sensor_msgs.msg import NavSatFix

# adapted from https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2/-/blob/main/polaris_gem_simulator/waypoint_logger/scripts/waypoint_logger.py
home = expanduser("~")
file = open(strftime(home + "/wp-%Y-%m-%d-%H-%M-%S", gmtime()) + ".csv", "w")


def save_waypoint(data):
    file.write(f"{data.latitude}, {data.longitude} \n")


def shutdown():
    file.close()


def listener():
    rospy.init_node("waypoints_logger", anonymous=True)
    rospy.Subscriber("/gem/gps/fix", NavSatFix, save_waypoint)
    rospy.spin()


if __name__ == "__main__":
    atexit.register(shutdown)
    print("Saving waypoints...")
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
