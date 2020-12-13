#pragma once
#include <Eigen/Dense>

namespace icp_cov {
class IcpCovariance {
  using CovMatrix = Eigen::MatrixXd;

 public:
  static IcpCovariance* Instance();

 public:
  void CalculateIcpCov();
  CovMatrix icp_cov_from_monte_carlo() const {
    return icp_cov_from_monte_carlo_;
  };
  CovMatrix icp_cov_from_hessian() const { return icp_cov_from_hessian_; };
  CovMatrix icp_cov_from_cost_function() const {
    return icp_cov_from_cost_function_;
  };

 private:
  static IcpCovariance* icp_covariance_;

 private:
  void IcpCovFromMonteCarlo();
  void IcpCovFromHessian();
  void IcpCovFromCostFunction();

 private:
  CovMatrix icp_cov_from_monte_carlo_;
  CovMatrix icp_cov_from_hessian_;
  CovMatrix icp_cov_from_cost_function_;

  CovMatrix vel_cov_from_monte_carlo_;
  CovMatrix vel_norm_cov_from_monte_carlo_;
  CovMatrix vel_direction_cov_from_monte_carlo_;
  CovMatrix yaw_rate_cov_from_monte_carlo_;
};
}  // namespace icp_cov