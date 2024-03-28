# Ground vehicles navigation

<a href="results"><img src="./images/banner.gif" width="600"></a>

This repo contains a demo of a model predictive controller, which given a specific GPS path, is able to follow it with while able to avoid obstacles.

This Demo is built on top of [POLARIS_GEM_e2 simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2).
The MPC is implemented using the [CasADi](https://web.casadi.org/docs/) framework, using [1] as reference.

## Project Structure

The **src** directory contains the following ros packages:
* `mpc_control` -> ROS package containing the actual MPC implementation.
  * `src/mpc_node.cpp` -> Actual control node, built as a ROS wrapper of a optimization problem.

* `mpc_gazebo` -> ROS package containing all the necessary utilities to run the MPC demo with the POLARIS_GEM simulator.
  * `scripts/mpc_evaluator.py` -> A script that performs the MPC evaluation. This script starts a ROS node that publishes the *Path* and *Obstacles* for the controller and logs its position over time to evaluate the error metrics.
  * `scripts/obstacle_spawner.py` -> A helper script to spawn gazebo entities from a template. 
  * `launch/mpc_demo.launch` -> Main launchfile that brings up the simulation environment, the control node and the evaluator script that performs the trial.
  * `data/obstacles.csv` -> Obstacles used in the evaluation, with the format (x_pos, y_pos, radius) w.r.t **world**.
  * `data/gps-waypoints.csv` -> Waypoints used in the evaluation, with the format (lat,lon)

The **docker** directory contains the dockerfile with all the dependencies in order to run the demo. See [Build with Docker] for more details.

## Controller

The Model predictive Controller has been implemented in a ROS node that can be launched with:
```
roslaunch mpc_control polaris_gem_mpc.launch
```

#### Subscribed Topics

* `/gem/base_footprint/odom` (nav_msgs/Odometry) -> Odometry w.r.t **world** frame.
* `/mpc/path` (nav_msgs/Path) -> Reprenst a set of GPS coordinates, Pose X and Y coordinates encode Latitude and Longitude
* `/mpc/obstacles` (vision_msgs/Detection3DArray) -> Represent a set of obstacles, at the moment only the bbox x-size is used to encode the obstacle radius

#### Published Topics

* `/gem/ackermann_cmd` (ackermann_msgs/AckermannDrive) -> Speed and Steering command output.
* `/mpc/markers` (vision_msgs/MarkerArray) -> Rviz markers that preview the controllers predicted optimal state trajectory. 

The controller implements the following MPC problem, adapted from [*1*]:

$$
\begin{aligned}
min_{x_1,\dots,x_{N+1}, u_1, \dots, u_{N+1}} \quad & \sum_{k=1}^{N+1} ({x_k - xref_k})^T Q ({x_k- xref_k}) + \sum_{k=2}^{N} ({u_k - u_{k-1} })^T R ({u_k- u_{k-1}}) \sum_{k=1}^{N}\sum_{j=1}^{O} D \log(1 + e^{ -d(x_k, o_k)}) + S(u_k, u_{k-1}) \\
\textrm{s.t.} \quad & x_{k+1}=f(x_k,u_k) \\
\quad & u_{min} \leq u_k \leq u_{max}    \\
\quad & x_{min} \leq x_k \leq x_{max}    \\
\quad & x_1 == x_{start}    \\
\end{aligned}
$$

Where:
* $x_k$ is the state of the vehicle {x,y,heading,speed}
* $u_k$ is the control input to the vehicle {acceleration,steer}
* $Q$ tracking error weight matrix
* $R$ control rate weight matrix
* $D$ obstacle proximity weight, *where d(x_k,o_j) is the signed distance of the vehicle at time k from obstacle j*
* $S$ is the control rate slack cost, this makes *jerk* and *slew* soft-constrains.
* $f$ is the function of our model

The vehicle is modelled in the controller the *bicycle model* kinematic equations and these kinematic constrains:
| Wheel Base | Width | Min/Max Steer | Min/Max Slew | Min/Max Speed | Min/Max Acc | Min/Max Jerk |
| --- | --- | --- | --- | --- | --- |--- |
| 1.75 m    | 1.2 m    | [-0.61,0.61] rad    | [-0.5,0.5] rad/s    | [0,10] m/s    | [-3,3] m/s^2   |  [-1.5,1.5] m/s^3  |

## Instructions

For best result see the Docker section below. 

You can run this in your ros workspce with the following (notable) dependencies
* [POLARIS_GEM_e2 simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2).
* [CasADi](https://web.casadi.org/docs/) with IPOPT support.
* [ipopt](https://coin-or.github.io/Ipopt/INSTALL.html) installed.

### Build with Docker

From this repository root directory:
```bash
docker build -t mpc-demo -f docker/Dockerfile .
```

Run it:
```bash
xhost +local:
docker run -it --net=host --ipc=host --privileged \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${XAUTHORITY}:/root/.Xauthority" \
    mpc-demo:latest \
    bash -c "roslaunch mpc_gazebo mpc_demo.launch"
```

### References
* [1][MPC Berkley - genesis path follower](https://github.com/MPC-Berkeley/genesis_path_follower/tree/master)
