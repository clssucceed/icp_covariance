#include <Eigen/Dense>

namespace icp_cov {
class IcpCovariance {
  using IcpCovMatrix = Eigen::MatrixXd;

 public:
  static IcpCovariance* Instance();

 public:
  void CalculateIcpCov();
  IcpCovMatrix icp_cov_from_monte_carlo() const {
    return icp_cov_from_monte_carlo_;
  };
  IcpCovMatrix icp_cov_from_hessian() const { return icp_cov_from_hessian_; };
  IcpCovMatrix icp_cov_from_cost_function() const {
    return icp_cov_from_cost_function_;
  };

 private:
  static IcpCovariance* icp_covariance_;

 private:
  void IcpCovFromMonteCarlo();
  void IcpCovFromHessian();
  void IcpCovFromCostFunction();

 private:
  IcpCovMatrix icp_cov_from_monte_carlo_;
  IcpCovMatrix icp_cov_from_hessian_;
  IcpCovMatrix icp_cov_from_cost_function_;
};
}  // namespace icp_cov