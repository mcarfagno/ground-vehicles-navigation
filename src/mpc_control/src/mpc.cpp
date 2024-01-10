#include "mpc_control/mpc.hpp"

#include <stdexcept>

namespace mpc {

KinematicMpc::KinematicMpc(const MpcParameters &p) {
  N_ = p.N;
  dt_ = p.DT;

  casadi::Slice all;

  // decision variables
  optimal_trajectory_ = opti_.variable(N_ + 1, nx_);
  auto x_dv = optimal_trajectory_(all, 0);
  auto y_dv = optimal_trajectory_(all, 1);
  auto psi_dv = optimal_trajectory_(all, 2);
  auto v_dv = optimal_trajectory_(all, 3);

  optimal_control_ = opti_.variable(N_, nu_);
  auto acc_dv = optimal_control_(all, 0);
  auto steer_dv = optimal_control_(all, 1);

  slack_ = opti_.variable(N_, nu_);
  auto sl_acc_dv = slack_(all, 0);
  auto sl_steer_dv = slack_(all, 1);

  // parameters
  trajectory_initial_conditions_ = opti_.parameter(nx_);
  optimal_control_prev_ = opti_.parameter(nu_);
  reference_trajectory_ = opti_.parameter(N_, nx_);
  auto x_ref = reference_trajectory_(all, 0);
  auto y_ref = reference_trajectory_(all, 1);
  auto psi_ref = reference_trajectory_(all, 2);
  auto v_ref = reference_trajectory_(all, 3);

  // numerical matrices
  state_error_cost_ = casadi::DM::diag(p.state_error_weights);
  control_input_cost_ = casadi::DM::diag(p.control_rate_weights);

  // kinematic constrains
  // NOTE: bicycle model ODE + RK4 integrator
  auto f = [this, p](const casadi::MX &x, const casadi::MX &u) {
    return horzcat(x(3) * cos(x(2)), x(3) * sin(x(2)),
                   x(3) * tan(u(1)) / p.wheel_base, u(0));
  };
  for (std::size_t k = 0; k < N_; k++) {
    casadi::MX k1 = f(optimal_trajectory_(k, all), optimal_control_(k, all));
    casadi::MX k2 =
        f(optimal_trajectory_(k, all) + dt_ / 2 * k1, optimal_control_(k, all));
    casadi::MX k3 =
        f(optimal_trajectory_(k, all) + dt_ / 2 * k2, optimal_control_(k, all));
    casadi::MX k4 =
        f(optimal_trajectory_(k, all) + dt_ * k3, optimal_control_(k, all));
    casadi::MX x_next =
        optimal_trajectory_(k, all) + dt_ / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    opti_.subject_to(optimal_trajectory_(k + 1, all) == x_next);
  }

  auto quad_form = [](const casadi::MX &x, const casadi::DM &Q) {
    return casadi::MX::mtimes({x, Q, x.T()});
  };

  // tracking cost
  auto cost = casadi::MX::zeros(1);
  for (std::size_t i = 0; i < N_; i++) {
    cost += quad_form(optimal_trajectory_(i + 1, all) -
                          reference_trajectory_(i, all),
                      state_error_cost_);
  }
  // input derivative cost
  for (std::size_t i = 0; i < N_ - 1; i++) {
    cost += quad_form(optimal_control_(i + 1, all) - optimal_control_(i, all),
                      control_input_cost_);
  }

  // slack cost
  cost += (sum1(sl_steer_dv) + sum1(sl_acc_dv));
  opti_.minimize(cost);

  // state bound constraints
  opti_.subject_to(opti_.bounded(p.v_min, v_dv, p.v_max));

  // acceleration and steer constrains
  opti_.subject_to(opti_.bounded(p.a_min, acc_dv, p.a_max));
  opti_.subject_to(opti_.bounded(p.steer_min, steer_dv, p.steer_max));

  // initial state constraint
  opti_.subject_to(x_dv(0) == trajectory_initial_conditions_(0));
  opti_.subject_to(y_dv(0) == trajectory_initial_conditions_(1));
  opti_.subject_to(psi_dv(0) == trajectory_initial_conditions_(2));
  opti_.subject_to(v_dv(0) == trajectory_initial_conditions_(3));

  // TODO: obstacle avoidance

  // input rate of change
  opti_.subject_to(opti_.bounded(p.jerk_min * dt_ - sl_acc_dv(0),
                                 acc_dv(0) - optimal_control_prev_(0),
                                 p.jerk_max * dt_ + sl_acc_dv(0)));

  opti_.subject_to(opti_.bounded(p.steer_rate_min * dt_ - sl_steer_dv(0),
                                 steer_dv(0) - optimal_control_prev_(1),
                                 p.steer_rate_max * dt_ + sl_steer_dv(0)));

  for (std::size_t i = 0; i < N_ - 1; i++) {
    opti_.subject_to(opti_.bounded(p.jerk_min * dt_ - sl_acc_dv(i + 1),
                                   acc_dv(i + 1) - acc_dv(i),
                                   p.jerk_max * dt_ + sl_acc_dv(i + 1)));
    opti_.subject_to(
        opti_.bounded(p.steer_rate_min * dt_ - sl_steer_dv(i + 1),
                      steer_dv(i + 1) - steer_dv(i),
                      p.steer_rate_max * dt_ + sl_steer_dv(i + 1)));
  }

  // slack must be positive
  opti_.subject_to(0 <= sl_steer_dv);
  opti_.subject_to(0 <= sl_acc_dv);

  // configure IPOPT solver
  const auto p_opts =
      casadi::Dict{{"expand", true}, {"print_time", p.verbose ? true : false}};
  const auto s_opts =
      casadi::Dict{{"max_cpu_time", p.max_cpu_time},
                   {"tol", p.tol},
                   {"print_level", p.verbose ? 5 : 0},
                   {"max_iter", static_cast<casadi_int>(p.max_iter)}};
  opti_.solver("ipopt", p_opts, s_opts);
}

std::optional<std::pair<casadi::DMDict, casadi::Dict>>
KinematicMpc::solve(const casadi::DMDict &in) {
  // TODO: get obstacles
  const auto &state_initial_condition = in.at(INITIAL_STATE_DICT_KEY);
  const auto &trajectory = in.at(TRAJECTORY_DICT_KEY);
  const auto &control_initial_condition = in.at(INITIAL_CONTROL_DICT_KEY);
  auto trajectory_reinterp =
      reinterpolate_reference(trajectory, state_initial_condition);

  opti_.set_value(trajectory_initial_conditions_, state_initial_condition);
  opti_.set_value(reference_trajectory_, trajectory_reinterp);
  opti_.set_value(optimal_control_prev_, control_initial_condition);

  // TODO: implement warm start: set the initial values of optimal_trajectory_
  // and optimal_control_ decision variables to reduce iterations
  try {
    auto solution = opti_.solve();
    casadi::DMDict out;
    out["X_opti"] = solution.value(optimal_trajectory_);
    out["U_opti"] = solution.value(optimal_control_);
    return std::make_pair(std::move(out), solution.stats());
  } catch (const std::exception &e) {
    // NOTE: exceptions are raised when the optimizer fails (e.g. time exceeded)
    std::cerr << e.what() << std::endl;
    return std::nullopt;
  }
}

casadi::DM KinematicMpc::reinterpolate_reference(const casadi::DM &traj,
                                                 const casadi::DM &x) const {
  using casadi::DM;
  using casadi::Slice;
  auto waypoints = DM::zeros(N_, nx_);

  // Find the index of the closest trajectory point to the vehicle.
  std::vector<double> distances(static_cast<int>(traj.size1()));
  for (casadi_int i = 0; i < traj.size1(); i++) {
    distances[i] =
        std::hypot((x(0) - traj(i, 0)).scalar(), (x(1) - traj(i, 1)).scalar());
  }
  auto min_element = std::min_element(distances.begin(), distances.end());
  std::size_t closest_idx = std::distance(distances.begin(), min_element);

  // find target states by interpolating along trajectory length.
  auto cdist = std::vector<double>(traj.size1());
  cdist[0] = 0.0;
  for (casadi_int i = 1; i < traj.size1(); i++) {
    cdist[i] =
        cdist[i - 1] + std::hypot((traj(i, 0) - traj(i - 1, 0)).scalar(),
                                  (traj(i, 1) - traj(i - 1, 1)).scalar());
  }

  auto start_dist = cdist[closest_idx];
  // NOTE: the interpolation points are equally
  // spaced given the average speed
  double v = casadi::norm_1(traj(Slice(), 3).get_elements());
  v /= traj(Slice(), 3).size1();

  auto intp_pts = DM::zeros(N_);
  for (std::size_t i = 0; i < N_; i++) {
    intp_pts(i) =
        std::clamp(start_dist + (i + 1) * v * dt_, cdist.front(), cdist.back());
  }

  auto x_intp = casadi::interpolant("x", "linear", {cdist},
                                    traj(Slice(), 0).get_elements());
  auto y_intp = casadi::interpolant("y", "linear", {cdist},
                                    traj(Slice(), 1).get_elements());
  auto t_intp = casadi::interpolant("theta", "linear", {cdist},
                                    traj(Slice(), 2).get_elements());
  auto v_intp = casadi::interpolant("speed", "linear", {cdist},
                                    traj(Slice(), 3).get_elements());

  for (std::size_t i = 0; i < N_; i++) {
    waypoints(i, 0) = x_intp(intp_pts(i));
    waypoints(i, 1) = y_intp(intp_pts(i));
    waypoints(i, 2) = t_intp(intp_pts(i));
    waypoints(i, 3) = v_intp(intp_pts(i));
  }

  // NOTE: equivalent of MATLAB unwrap, removes jumps from heading
  auto unwrap = [](float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;
  };

  waypoints(0, 2) = unwrap(x(2).scalar(), waypoints(0, 2).scalar());
  for (std::size_t i = 1; i < N_; i++) {
    waypoints(i, 2) =
        unwrap(waypoints(i - 1, 2).scalar(), waypoints(i, 2).scalar());
  }

  return waypoints;
}

void KinematicMpc::set_initial_state(casadi::DMDict &in,
                                     const nav_msgs::Odometry &odom) const {
  in[INITIAL_STATE_DICT_KEY] = casadi::DM(
      {odom.pose.pose.position.x, odom.pose.pose.position.y,
       tf::getYaw(odom.pose.pose.orientation),
       std::hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)});
}

void KinematicMpc::set_prev_cmd(casadi::DMDict &in, const MpcCmd &cmd) const {
  in[INITIAL_CONTROL_DICT_KEY] = casadi::DM({cmd.acceleration, cmd.steer});
}

void KinematicMpc::set_reference(casadi::DMDict &in,
                                 const nav_msgs::Path &path) const {
  auto tmp = casadi::DM(path.poses.size(), nx_);
  for (std::size_t i = 0; i < path.poses.size(); i++) {
    tmp(i, casadi::Slice()) = {
        path.poses[i].pose.position.x, path.poses[i].pose.position.y,
        tf::getYaw(path.poses[i].pose.orientation), MPC_REF_SPEED};
  }
  in[TRAJECTORY_DICT_KEY] = tmp;
}

void KinematicMpc::set_obstacles(
    casadi::DMDict &in, const vision_msgs::Detection3DArray &obs) const {
  auto tmp = casadi::DM(obs.detections.size(), 2);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp(i, casadi::Slice()) = {
        obs.detections[i].results.front().pose.pose.position.x,
        obs.detections[i].results.front().pose.pose.position.y};
  }
  in[OBSTACLES_DICT_KEY] = tmp;
}

MpcCmd KinematicMpc::get_control(casadi::DMDict &in) const {
  const auto &tmp = in[OPTIMIZED_CONTROL_DICT_KEY];
  return MpcCmd(tmp(0, 0).scalar(), tmp(0, 1).scalar());
}
} // namespace mpc
