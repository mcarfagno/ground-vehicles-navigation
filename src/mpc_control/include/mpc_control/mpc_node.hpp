#ifndef MPC_CONTROL__MPC_NODE_HPP_
#define MPC_CONTROL__MPC_NODE_HPP_

#include "mpc_control/mpc.hpp"
#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Point.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <vector>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace mpc {

// World origin in Lat/Lon
// TODO: this should be a rosparam
static const float GPS_WORLD_ORIGIN_LAT = 40.09302492080515;
static const float GPS_WORLD_ORIGIN_LON = -88.2357551253083;

class MpcNode {
public:
  MpcNode();
  ~MpcNode(){};
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher viz_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber obstacles_sub_;

  int mpc_horizon_steps_;
  float rate_;
  float obs_safety_dist_;
  float x_weight_;
  float y_weight_;
  float yaw_weight_;
  float speed_weight_;
  float steer_rate_weight_;
  float acc_rate_weight_;
  float dist_weight_;

  MpcCmd prev_cmd_;
  std::optional<casadi::DM> prev_mpc_traj_;
  std::optional<casadi::DM> prev_mpc_cmd_;
  std::optional<KinematicMpc> mpc_;
  std::optional<nav_msgs::Odometry> latest_odom_;
  std::optional<nav_msgs::Path> path_;
  std::optional<vision_msgs::Detection3DArray> obstacles_;

  casadi::DM odometry_to_casadi(const nav_msgs::Odometry &odom) const;
  casadi::DM cmd_to_casadi(const MpcCmd &cmd) const;
  casadi::DM path_to_casadi(const nav_msgs::Path &path) const;
  casadi::DM
  obstacles_to_casadi(const vision_msgs::Detection3DArray &obs) const;
  MpcCmd casadi_to_cmd(casadi::DM &in) const;

  void publish_mpc_cmd(double speed, double steer);
  void publish_rviz_markers(const casadi::DM &predicted_state_traj);
};

} // namespace mpc

/**
 * @brief Converts latitude and longitude to global X, Y coordinates,
 *        using an equirectangular projection.
 *
 *  @returns pair(meters east of lon0, meters north of lat0)
 *
 *  Sources: http://www.movable-type.co.uk/scripts/latlong.html
 *           https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
 */
std::pair<double, double> latlon_to_XY(double lat0, double lon0, double lat1,
                                       double lon1);

#endif
