import csv
import os
import time

import rospy
from gazebo_ros import gazebo_interface
from geometry_msgs.msg import Point, Pose, Quaternion
from tf.transformations import quaternion_from_euler


# adapted from https://nozomi.sk/spawn-objects-in-gazebo-of-ros-kinetic-melodic-python-2-7/
def spawn(model_name, radius, positions, orientations, static=False):
    model_template = """<sdf version="1.4">
              <model name="obstacle">
                  <static>%STATIC%</static>
            <link name="link">
              <inertial>
                <mass>1.0</mass>
                <inertia> <!-- inertias are tricky to compute -->
                  <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->
                  <ixx>0.083</ixx>
                  <ixy>0.0</ixy> 
                  <ixz>0.0</ixz>
                  <iyy>0.083</iyy>
                  <iyz>0.0</iyz> 
                  <izz>0.083</izz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <cylinder>
                    <radius>%RADIUS%</radius>
                    <length>%LENGTH%</length>
                  </cylinder>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <cylinder>
                    <radius>%RADIUS%</radius>
                    <length>%LENGTH%</length>
                  </cylinder>
                </geometry>
              </visual>
            </link>
              </model>
            </sdf>"""
    model_xml = (
        model_template.replace("%MODEL_NAME%", model_name)
        .replace("%STATIC%", str(int(static)))
        .replace("%RADIUS%", str(radius))
        .replace("%LENGTH%", "1")
    )
    initial_pose = Pose(
        Point(*positions), Quaternion(*quaternion_from_euler(*orientations))
    )
    gazebo_model_name = "%s_%d" % (model_name, round(time.time() * 1000))
    gazebo_interface.spawn_sdf_model_client(
        gazebo_model_name, model_xml, rospy.get_namespace(), initial_pose, "", "/gazebo"
    )
    rospy.loginfo("%s spawned in Gazebo as %s", model_name, gazebo_model_name)
    return gazebo_model_name


def main():
    rospy.init_node("spawner", anonymous=True)
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../data/obstacles.csv")

    with open(filename) as f:
        obstacles = [tuple(line) for line in csv.reader(f)]

    for o in obstacles:
        spawn("1", float(o[2]), [float(o[0]), float(o[1]), 0.5], [0, 0, 0], static=True)


if __name__ == "__main__":
    main()
