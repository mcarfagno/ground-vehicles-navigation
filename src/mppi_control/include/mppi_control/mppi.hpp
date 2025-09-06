#ifndef MPPI_CONTROLLER__MPPI_HPP_
#define MPPI_CONTROLLER__MPPI_HPP_

#include <eigen3/Eigen/Dense>
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
      float delta_t = 0.05, std::size_t horizon_step_T = 30,
      std::size_t number_of_samples_K = 1000, float param_exploration = 0.0,
      float param_lambda = 50.0,
      float param_alpha = 1.0 Matrix2f sigma = Matrix2f(0.5, 0.0, 0.0, 0.1),
      Vector4f stage_cost_weight = Vector4f(50.0, 50.0, 1.0,
                                            20.0), // weight for [x, y, yaw, v]
      Vector4f terminal_cost_weight =
          Vector4f(50.0, 50.0, 1.0, 20.0) // weight for [x, y, yaw, v]
      )
      : dt_(delta_t), T_(horizon_step_T), K_(number_of_samples_K),
        param_exploration_(param_exploration), param_lambda_(param_lambda),
        param_alpha_(param_alpha), {

    param_gamma_ = param_lambda_ * (1.0 - (param_alpha_));
    sigma_ = sigma;
    stage_cost_weight_ = stage_cost_weight;
    terminal_cost_weight_ << terminal_cost_weight;

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
  float dim_x_ = 4; // dimension of system state vector
  float dim_u_ = 2; // dimension of control input vector

  std::size_t T_;                 // prediction horizon
  std::size_t K_;                 // number of rollouts
  float param_exploration_;       // constant parameter of mppi
  float param_lambda_;            // constant parameter of mppi
  float param_alpha_;             // constant parameter of mppi
  float param_gamma_;             // constant parameter of mppi
  Matrix2f sigma_;                // standard deviation of noise
  Vector4f stage_cost_weight_;    // weight for [x, y, yaw, v]
  Vector4f terminal_cost_weight_; // weight for [x, y, yaw, v]
  ArrayXXf u_prev_;               // nominal control sequence (prev iteration)

  // vehicle parameters
  float dt_ = dt;
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
