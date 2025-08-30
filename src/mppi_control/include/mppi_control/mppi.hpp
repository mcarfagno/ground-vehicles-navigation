#ifndef MPPI_CONTROLLER__MPPI_HPP_
#define MPPI_CONTROLLER__MPPI_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace mppi {

/* @brief (acceleration,steer)
 * */
typedef std::pair<double, double> MppiCmd;

class MPPI {
private:
  float dt_;

  /**
   * @brief reinterpolates a trajectory to one of the correct
   * size and starting point
   * */
  Eigen::MatrixXf reinterpolate_reference_trajectory(const Eigen::MatrixXf  &traj,
                                                 const Eigen::Vector4d  &x) const;

public:
  explicit MPPI();
  ~MPPI() {}
};

} // namespace mppi
#endif
