#include "mpc_control/mpc.hpp"
#include <stdexcept>

namespace mpc {

KinematicMpc::KinematicMpc(const KinematicModel &m, const MpcParameters &p,
                           const casadi::DM &trajectory,
                           const casadi::DM &obstacles) {

  // save for later
  trajectory_ = trajectory;
  obstacles_ = obstacles;

  // problem size variables
  N_ = p.N;   // horizon
  dt_ = p.DT; // time step
  nx_ = m.nx; // n state vars
  nu_ = m.nu; // n control vars
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

  // problem parameters
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
  for (std::size_t k = 0; k < N_; k++) {
    casadi::MX k1 = m.f(optimal_trajectory_(k, all), optimal_control_(k, all));
    casadi::MX k2 = m.f(optimal_trajectory_(k, all) + dt_ / 2 * k1,
                        optimal_control_(k, all));
    casadi::MX k3 = m.f(optimal_trajectory_(k, all) + dt_ / 2 * k2,
                        optimal_control_(k, all));
    casadi::MX k4 =
        m.f(optimal_trajectory_(k, all) + dt_ * k3, optimal_control_(k, all));
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

  // state bound constraints
  opti_.subject_to(opti_.bounded(m.v_min, v_dv, m.v_max));

  // acceleration and steer constrains
  opti_.subject_to(opti_.bounded(m.a_min, acc_dv, m.a_max));
  opti_.subject_to(opti_.bounded(m.steer_min, steer_dv, m.steer_max));

  // initial state constraint
  opti_.subject_to(x_dv(0) == trajectory_initial_conditions_(0));
  opti_.subject_to(y_dv(0) == trajectory_initial_conditions_(1));
  opti_.subject_to(psi_dv(0) == trajectory_initial_conditions_(2));
  opti_.subject_to(v_dv(0) == trajectory_initial_conditions_(3));

  // obstacle avoidance
  for (std::size_t i = 0; i < N_; i++) {
    for (casadi_int j = 0; j < obstacles.size1(); j++) {
      auto x_rear = x_dv(i);
      auto y_rear = y_dv(i);

      // auto x_front=x_dv(i)+m.wheel_base*cos(psi_dv(i));
      // auto y_front=y_dv(i)+m.wheel_base*sin(psi_dv(i));

      // collision dist from front and rear axle
      auto d_rear =
          pow(x_rear - obstacles(j, 0), 2) + pow(y_rear - obstacles(j, 1), 2);
      // auto d_front = pow(x_front - obstacles(j, 0),2) +
      //         pow(y_front - obstacles(j, 1),
      //             2);

      // min collision dist
      // opti_.subject_to(d_front > obstacles(j, 2) +m.vehicle_width
      // +p.min_safety_dist);
      opti_.subject_to(d_rear >
                       obstacles(j, 2) + m.vehicle_width + p.min_safety_dist);

      // obstacle proximity cost
      cost += exp(p.obstacle_avoidance_weight * exp(-d_rear));
    }
  }
  // input rate of change
  opti_.subject_to(opti_.bounded(m.jerk_min * dt_ - sl_acc_dv(0),
                                 acc_dv(0) - optimal_control_prev_(0),
                                 m.jerk_max * dt_ + sl_acc_dv(0)));

  opti_.subject_to(opti_.bounded(m.steer_rate_min * dt_ - sl_steer_dv(0),
                                 steer_dv(0) - optimal_control_prev_(1),
                                 m.steer_rate_max * dt_ + sl_steer_dv(0)));

  for (std::size_t i = 0; i < N_ - 1; i++) {
    opti_.subject_to(opti_.bounded(m.jerk_min * dt_ - sl_acc_dv(i + 1),
                                   acc_dv(i + 1) - acc_dv(i),
                                   m.jerk_max * dt_ + sl_acc_dv(i + 1)));
    opti_.subject_to(
        opti_.bounded(m.steer_rate_min * dt_ - sl_steer_dv(i + 1),
                      steer_dv(i + 1) - steer_dv(i),
                      m.steer_rate_max * dt_ + sl_steer_dv(i + 1)));
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

  opti_.minimize(cost);
  opti_.solver("ipopt", p_opts, s_opts);
}

std::optional<casadi::DMDict> KinematicMpc::solve(const casadi::DMDict &in) {

  const auto &state_initial_condition = in.at(INITIAL_STATE_DICT_KEY);
  const auto &control_initial_condition = in.at(INITIAL_CONTROL_DICT_KEY);
  const auto trajectory_reinterp =
      reinterpolate_reference_trajectory(trajectory_, state_initial_condition);

  // set problem parameters
  opti_.set_value(trajectory_initial_conditions_, state_initial_condition);
  opti_.set_value(reference_trajectory_, trajectory_reinterp);
  opti_.set_value(optimal_control_prev_, control_initial_condition);

  // TODO: implement warm start: set the initial values of optimal_trajectory_
  // and optimal_control_ decision variables to reduce iterations
  try {
    auto solution = opti_.solve();
    casadi::DMDict out;
    out[OPTIMIZED_TRAJECTORY_DICT_KEY] = solution.value(optimal_trajectory_);
    out[OPTIMIZED_CONTROL_DICT_KEY] = solution.value(optimal_control_);
    return std::move(out);
  } catch (const std::exception &e) {
    // NOTE: exceptions are raised when the optimizer fails (e.g. time exceeded)
    std::cerr << e.what() << std::endl;
    return std::nullopt;
  }
}

casadi::DM
KinematicMpc::reinterpolate_reference_trajectory(const casadi::DM &traj,
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

} // namespace mpc
