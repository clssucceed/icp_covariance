#include "utils.h"

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
}  // namespace utils
}  // namespace icp_cov
