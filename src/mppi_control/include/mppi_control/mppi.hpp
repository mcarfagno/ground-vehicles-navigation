#ifndef MPPI_CONTROLLER__MPPI_HPP_
#define MPPI_CONTROLLER__MPPI_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "mppi_control/utils.hpp"
namespace mppi {

/* commonly used eigen types
 * */
using Eigen::ArrayXf;
using Eigen::ArrayXXf;
using Eigen::Matrix2f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector4f;
using Eigen::VectorXf;

/* @brief (steer,accel)
 * */
typedef std::pair<double, double> MppiCmd;

class MPPI {
public:
  explicit MPPI(
      const float delta_t = 0.05, const std::size_t horizon_step_T = 30,
      const std::size_t number_of_samples_K = 1000,
      const float param_exploration = 0.0, const float param_lambda = 50.0,
      const float param_gamma = 0.0
      // Matrix2f sigma = Matrix2f(0.5, 0.0, 0.0, 0.1),
      // Vector4f stage_cost_weight = Vector4f(50.0, 50.0, 1.0,
      //                                       20.0), // weight for [x, y, yaw,
      //                                       v]
      // Vector4f terminal_cost_weight =
      //    Vector4f(50.0, 50.0, 1.0, 20.0) // weight for [x, y, yaw, v]
      )
      : dt_(delta_t), T_(horizon_step_T), K_(number_of_samples_K),
        param_exploration_(param_exploration), param_lambda_(param_lambda),
        param_gamma_(param_gamma) {

    std::cout << "dt" << dt_ << std::endl;
    std::cout << "T" << T_ << std::endl;
    std::cout << "K" << K_ << std::endl;

    sigma_ << 0.5, 0.0, 0.0, 0.1;
    stage_cost_weight_ << 50.0, 50.0, 1.0, 20.0;    // weight for [x, y, yaw, v]
    terminal_cost_weight_ << 50.0, 50.0, 1.0, 20.0; // weight for [x, y, yaw, v]

    u_prev_.setZero(T_, dim_u_);
  }

  ~MPPI() {}

  /*
   * @brief resets the nominal control sequence
   * */
  void reset() { u_prev_.setZero(T_, dim_u_); }

  // TODO: add obstacles
  std::tuple<MppiCmd, MatrixXf, std::vector<MatrixXf>>
  compute_optimal_input(const MatrixXf &trajectory, const Vector4f &x0);

private:
  // mppi parameters
  const std::size_t dim_x_ = 4; // dimension of system state vector
  const std::size_t dim_u_ = 2; // dimension of control input vector

  float dt_;      // interval (s) between two sampled points in trajectories
  std::size_t T_; // prediction horizon or time steps in each sampled trajectory
  std::size_t K_; // number of rollouts or randomly sampled trajectories
  float param_exploration_; // constant parameter of mppi
  float param_lambda_;      // temperature, lambda -> 0 selects only best
                            // trajectories with low costs
  float
      param_gamma_; // smoothness parameter, this should be a small number < 0.1
  Matrix2f sigma_;  // noise covariance matrix
  Vector4f stage_cost_weight_;    // weight for [x, y, yaw, v]
  Vector4f terminal_cost_weight_; // weight for [x, y, yaw, v]
  MatrixXf u_prev_; // nominal control sequence from prev iteration

  // vehicle parameters
  float wheel_base_ = 1.75;        // [m]
  float vehicle_width_ = 1.2;      // [m]
  float v_min_ = 0.0;              // [m/s]
  float v_max_ = 10.0;             // [m/s]
  float a_max_abs_ = 3.0;          // [m/ss]
  float jerk_max_abs_ = 1.5;       // [m/sss]
  float steer_max_abs_ = 0.61;     // [rad]
  float steer_rate_max_abs_ = 0.5; // [rad/s]

  /**
   * @brief reinterpolates a trajectory to one of the correct
   * size and starting point
   * */
  MatrixXf reinterpolate_reference_trajectory(const MatrixXf &traj,
                                              const Vector4f &x) const;

  /**
   * @brief samples disturbance vector epsilon_u_k
   * */
  ArrayXXf compute_epsilon_() const;

  /**
   * @brief vehicle kinematics
   * */
  Vector4f F_(const Vector4f &x_t, const Vector2f &u_t) const;

  /**
   * @brief input clamp function
   * */
  Vector2f g_(const Vector2f &u_t) const;

  /**
   * @brief stage cost function
   * */
  float c_(const Vector4f &x_t, const Vector4f &x_ref) const;

  /**
   * @brief terminal cost function
   * */
  float phi_(const Vector4f &x_t, const Vector4f &x_ref) const;

  /**
   * @brief computes weights for each sample
   * */
  VectorXf compute_weights_(const ArrayXf &S) const;
};

} // namespace mppi
#endif
