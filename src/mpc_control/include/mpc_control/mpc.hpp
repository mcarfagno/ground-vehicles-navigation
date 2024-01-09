#ifndef MPC_CONTROLLER__MPC_HPP_
#define MPC_CONTROLLER__MPC_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "casadi/casadi.hpp"

namespace mpc {

struct MpcParameters {
  std::size_t N = 10;
  double DT = 0.2;
  double wheel_base = 1.5213;
  double v_min = 0.0;
  double v_max = 20.0;
  double a_min = -3.0;
  double a_max = 2.0;
  double steer_min = -0.5;
  double steer_max = 0.5;
  double jerk_min = -1.5;
  double jerk_max = 1.5;
  double steer_rate_min = -0.5;
  double steer_rate_max = 0.5;
  double obstacle_avoidance_dist = 0.5;
  std::vector<double> state_error_weights = {1.0, 1.0, 1.0, 0.1};
  std::vector<double> control_rate_weights = {10.0, 100.0};
  double tol = 1e-3;
  double max_cpu_time = 0.5;
  int max_iter = 200;
  bool verbose = false;
};

struct MpcCmd {
  double acceleration{0.0};
  double steer{0.0};
  MpcCmd() {}
  MpcCmd(double a, double df) : acceleration(a), steer(df) {}
};

class KinematicMpc {
private:
  float dt_;
  std::size_t N_;
  const std::size_t nx_ = 4;
  const std::size_t nu_ = 2;

  // Problem placeholder obj
  casadi::Opti opti_;

  casadi::DM state_error_cost_;
  casadi::DM control_input_cost_;

  casadi::MX optimal_trajectory_;
  casadi::MX trajectory_initial_conditions_;

  casadi::MX reference_trajectory_;

  casadi::MX optimal_control_;
  casadi::MX optimal_control_prev_;
  casadi::MX slack_;

  /**
   * @brief reinterpolates a trajectory to one of the correct
   * size and starting point
   * */
  casadi::DM reinterpolate_reference(const casadi::DM &traj,
                                     const casadi::DM &x) const;

public:
  explicit KinematicMpc(const MpcParameters &p);
  ~KinematicMpc() {}

  std::optional<std::pair<casadi::DMDict, casadi::Dict>>
  solve(const casadi::DMDict &in);

  void set_initial_state() const;

  void set_prev_cmd(casadi::DMDict &in, const MpcCmd &cmd) const;

  void set_reference() const;

  MpcCmd get_control(casadi::DMDict &in) const;
};

} // namespace mpc
#endif