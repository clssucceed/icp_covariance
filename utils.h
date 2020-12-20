#pragma once
#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
namespace utils {

Eigen::Affine3d RtToAffine3d(const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& t);
void TransformPoints(const std::vector<Eigen::Vector3d>& src_points,
                     const Eigen::Affine3d& Transform,
                     std::vector<Eigen::Vector3d>& dst_points);
Eigen::Vector3d PointNoise2d(const double sigma);
Eigen::Vector3d PointNoise3d(const double sigma);
Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& v);
Eigen::Affine3d TransformNoise(const double rotation_sigma,
                               const double translation_sigma);
Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R);
Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr);
Eigen::Quaterniond DeltaQ(const Eigen::Vector3d& theta);
Eigen::MatrixXd Covariance(const Eigen::MatrixXd& input);
double AngleMod(double x);
void PrintPoints(const std::vector<Eigen::Vector3d>& points,
                 const std::string& points_name);
void DebugThisSimulation();
}  // namespace utils
}  // namespace icp_cov