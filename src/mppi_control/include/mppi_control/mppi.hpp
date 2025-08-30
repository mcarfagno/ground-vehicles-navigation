#ifndef MPPI_CONTROLLER__MPPI_HPP_
#define MPPI_CONTROLLER__MPPI_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "mppi_control/utils.hpp"
namespace mppi {

/* @brief (acceleration,steer)
 * */
typedef std::pair<double, double> MppiCmd;

class MPPI {
public:
  explicit MPPI(float delta_t = 0.05, std::size_t horizon_step_T = 30,
                std::size_t number_of_samples_K = 1000,
                float param_exploration = 0.0, float param_lambda = 50.0,
                float param_alpha = 1.0,
                // TODO: sort out thesr 3
                //  float sigma,
                //  float stage_cost_weight, // weight for [x, y, yaw, v]
                //  float terminal_cost_weight, // weight for [x, y, yaw, v]
                )
      : dt_(delta_t), T_(horizon_step_T), K_(number_of_samples_K),
        param_exploration_(param_exploration), param_lambda_(param_lambda),
        param_alpha_(param_alpha), {

    // wtf is gamma?
    param_gamma_ = param_lambda_ * (1.0 - (param_alpha_));
    sigma_ = << 0.5, 0.0, 0.0, 0.1;
    stage_cost_weight << 50.0, 50.0, 1.0, 20.0;
    terminal_cost_weight << 50.0, 50.0, 1.0, 20.0;
  }

  ~MPPI() {}

private:
  // mppi parameters
  float dim_x_ = 4; // dimension of system state vector
  float dim_u = 2;  // dimension of control input vector

  std::size_t T_;                       // prediction horizon
  std::size_t K_;                       // number of sample trajectories
  float param_exploration_;             // constant parameter of mppi
  float param_lambda_;                  // constant parameter of mppi
  float param_alpha_;                   // constant parameter of mppi
  float param_gamma_;                   // constant parameter of mppi
  Eigen::Matrix2f sigma_;               // standard deviation of noise
  Eigen::Vector4f stage_cost_weight;    // weight for [x, y, yaw, v]
  Eigen::Vector4f terminal_cost_weight; // weight for [x, y, yaw, v]

  // vehicle parameters
  float dt_ = dt;
  double wheel_base = 1.75;        // [m]
  double vehicle_width = 1.2;      // [m]
  double v_min = 0.0;              // [m/s]
  double v_max = 10.0;             // [m/s]
  double a_max_abs = 3.0;          // [m/ss]
  double jerk_max_abs = 1.5;       // [m/sss]
  double steer_max_abs = 0.61;     // [rad]
  double steer_rate_max_abs = 0.5; // [rad/s]
                                   //
  /**
   * @brief reinterpolates a trajectory to one of the correct
   * size and starting point
   * */
  Eigen::MatrixXf
  reinterpolate_reference_trajectory(const Eigen::MatrixXf &traj,
                                     const Eigen::Vector4f &x) const;
};

} // namespace mppi
#endif
