cmake_minimum_required(VERSION 2.8.3)

project(mpc_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  ackermann_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS scripts/mpc_evaluator.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
