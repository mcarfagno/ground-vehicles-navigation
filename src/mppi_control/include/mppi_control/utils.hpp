#ifndef MPPI_CONTROLLER__UTILS_HPP_
#define MPPI_CONTROLLER__UTILS_HPP_

#include <eigen3/Eigen/Dense>
#include <random>
#include <utility>

/* @brief used to sample a multivariate_normal
 * SOURCE:
 * https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
 * usage:
 * int size = 2;
 * Eigen::MatrixXf covar(size,size);
 *  covar << 1, .5,
 *         .5, 1;
 *
 * normal_random_variable sample { covar };
 * std::cout << sample() << std::endl;
 * */
struct normal_random_variable {
  normal_random_variable(Eigen::MatrixXf const &covar)
      : normal_random_variable(Eigen::VectorXf::Zero(covar.rows()), covar) {}

  normal_random_variable(Eigen::VectorXf const &mean,
                         Eigen::MatrixXf const &covar)
      : mean(mean) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXf mean;
  Eigen::MatrixXf transform;

  Eigen::VectorXf operator()() const {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<float> dist;

    return mean + transform * Eigen::VectorXf{mean.size()}.unaryExpr(
                                  [&](float x) { return dist(gen); });
  }
};

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
                                       double lon1) {
  auto R_earth = 6371000; // meters
  auto delta_lat = (lat1 - lat0) * (M_PI / 180);

  auto delta_lon = (lon1 - lon0) * (M_PI / 180);

  auto lat_avg = 0.5 * (lat1 * (M_PI / 180) + lat0 * (M_PI / 180));
  auto X = R_earth * delta_lon * std::cos(lat_avg);
  auto Y = R_earth * delta_lat;

  return std::make_pair(X, Y);
}

#endif
