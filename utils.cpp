#include "utils.h"

#include <ctime>
#include <iostream>
#include <random>

namespace icp_cov {
namespace utils {

Eigen::Affine3d RtToAffine3d(const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& t) {
  Eigen::Matrix4d T;
  T.setZero();
  T.block(0, 0, 3, 3) = R;
  T.block(0, 3, 3, 1) = t;
  return Eigen::Affine3d{T};
}

void TransformPoints(const std::vector<Eigen::Vector3d>& src_points,
                     const Eigen::Affine3d& Transform,
                     std::vector<Eigen::Vector3d>& dst_points) {
  for (const auto& point : src_points) {
    dst_points.emplace_back(Transform * point);
  }
}

Eigen::Vector3d PointNoise(const double sigma) {
  // 产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
  static std::default_random_engine e(std::time(0));
  // 产生正态分布对象
  static std::normal_distribution<double> n(0, sigma);
  // 生成point noise
  return Eigen::Vector3d(n(e), n(e), 0);
}

Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& v) {
  Eigen::Matrix3d skew_matrix;
  skew_matrix << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

Eigen::Affine3d TransformNoise(const double rotation_sigma,
                               const double translation_sigma) {
  // 产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
  static std::default_random_engine e(std::time(0));
  // 产生正态分布对象
  static std::normal_distribution<double> n_rotation(0, rotation_sigma);
  static std::normal_distribution<double> n_translation(0, rotation_sigma);
  // 生成Transform noise
  Eigen::Vector3d theta_noise(0, 0, n_rotation(e));
  Eigen::Matrix3d rotation_noise{DeltaQ(theta_noise)};
  Eigen::Vector3d translation_noise(n_translation(e), n_translation(e), 0);
  return RtToAffine3d(rotation_noise, translation_noise);
}

Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R) {
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = atan2(n(1), n(0));
  double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  double r =
      atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}
Eigen::Quaterniond DeltaQ(const Eigen::Vector3d& theta) {
  Eigen::Quaterniond dq;
  Eigen::Vector3d half_theta = theta * 0.5;
  dq.w() = 1.0;
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq.normalized();
}

}  // namespace utils
}  // namespace icp_cov
