#include "utils.h"
#include<random>
#include<ctime>

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
  return Eigen::Vector3d(n(e), n(e), n(e));
}
}  // namespace utils
}  // namespace icp_cov
