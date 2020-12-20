#include "utils.h"

#include <ctime>
#include <iostream>
#include <random>

#include "color.h"
#include "data_generator.h"
#include "pcl_alignment.h"
#include "visualization.h"

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
  dst_points.clear();
  dst_points.reserve(src_points.size());
  for (const auto& point : src_points) {
    dst_points.emplace_back(Transform * point);
  }
}

Eigen::Vector3d PointNoise2d(const double sigma) {
  // 产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
  static std::default_random_engine e(std::time(0));
  // 产生正态分布对象
  static std::normal_distribution<double> n(0, sigma);
  // 生成point noise
  return Eigen::Vector3d(n(e), n(e), 0);
}

Eigen::Vector3d PointNoise3d(const double sigma) {
  // 产生随机数引擎，采用time作为种子，以确保每次运行程序都会得到不同的结果
  static std::default_random_engine e(std::time(0));
  // 产生正态分布对象
  static std::normal_distribution<double> n(0, sigma);
  // 生成point noise
  return Eigen::Vector3d(n(e), n(e), n(e));
}

Eigen::Matrix3d SkewMatrix(const Eigen::Vector3d& v) {
  Eigen::Matrix3d skew_matrix;
  skew_matrix << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return skew_matrix;
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

Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr) {
  double y = ypr(0) / 180.0 * M_PI;
  double p = ypr(1) / 180.0 * M_PI;
  double r = ypr(2) / 180.0 * M_PI;

  Eigen::Matrix3d Rz;
  Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

  Eigen::Matrix3d Ry;
  Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

  Eigen::Matrix3d Rx;
  Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

  return Rz * Ry * Rx;
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
Eigen::MatrixXd Covariance(const Eigen::MatrixXd& input) {
  Eigen::MatrixXd mean = input.colwise().mean();
  //求取上述的零均值列向量矩阵
  Eigen::MatrixXd zero_mean_mat = input;
  //将列向量均值从MatrixXd 转换为行向量 RowVectorXd
  Eigen::RowVectorXd mean_mat(
      Eigen::RowVectorXd::Map(mean.data(), input.cols()));
  zero_mean_mat.rowwise() -= mean_mat;
  //计算协方差
  Eigen::MatrixXd cov =
      (zero_mean_mat.adjoint() * zero_mean_mat) / double(input.rows() - 1);

  std::cout << "mean: \n" << mean << std::endl;
  std::cout << "cov: \n" << cov << std::endl;
}

double AngleMod(double x) {
  if (x > 90) {
    return x - 180;
  } else if (x < -90) {
    return x + 180;
  } else {
    return x;
  }
}
void PrintPoints(const std::vector<Eigen::Vector3d>& points,
                 const std::string& points_name) {
  std::cout << points_name << ": size = " << points.size() << std::endl;
  for (const auto& point : points) {
    std::cout << point.transpose() << "; ";
  }
  std::cout << std::endl;
}

void DebugThisSimulation() {
  std::cout << "###############################################" << std::endl;
  auto data_generator = icp_cov::DataGenerator::Instance();
  auto pcl1_in_ref_frame = data_generator->pcl1_in_ref_frame();
  auto pcl2_in_ref_frame = data_generator->pcl2_in_ref_frame();
  auto pcl1_in_ref_frame_with_noise =
      data_generator->pcl1_in_ref_frame_with_noise();
  auto pcl2_in_ref_frame_with_noise =
      data_generator->pcl2_in_ref_frame_with_noise();
  auto visualization = icp_cov::Visualization::Instance();
  visualization->ResetCanvas();
  visualization->DrawPoints(pcl1_in_ref_frame, icp_cov::kColorRed);
  visualization->DrawPoints(pcl2_in_ref_frame, icp_cov::kColorGreen);
  visualization->DrawPoints(pcl1_in_ref_frame_with_noise, icp_cov::kColorPink);
  visualization->DrawPoints(pcl2_in_ref_frame_with_noise, icp_cov::kColorCyan);
  auto pcl_alignment = icp_cov::PclAlignment::Instance();
  std::vector<Eigen::Vector3d> pcl1_aligned_in_world_frame =
      pcl_alignment->eigen_pcl1_aligned();
  std::vector<Eigen::Vector3d> pcl1_aligned_in_ref_frame;
  icp_cov::utils::TransformPoints(pcl1_aligned_in_world_frame,
                                  data_generator->ref_pose().inverse(),
                                  pcl1_aligned_in_ref_frame);
  visualization->DrawPoints(pcl1_aligned_in_ref_frame, icp_cov::kColorWhite);
  Eigen::Affine3d icp_transform_est = pcl_alignment->icp_transform_est();
  Eigen::Vector3d ypr = icp_cov::utils::R2ypr(icp_transform_est.rotation());
  Eigen::Vector3d xyz = icp_transform_est.translation();
  const int kScale = 1000;
  const std::string image_name =
      std::to_string(static_cast<int>(ypr(0) * kScale)) + "_" +
      std::to_string(static_cast<int>(ypr(1) * kScale)) + "_" +
      std::to_string(static_cast<int>(ypr(2) * kScale)) + "_" +
      std::to_string(static_cast<int>(xyz(0) * kScale)) + "_" +
      std::to_string(static_cast<int>(xyz(1) * kScale)) + "_" +
      std::to_string(static_cast<int>(xyz(2) * kScale)) + ".png";
  visualization->Save(image_name);
  pcl_alignment->Debug();
}

void CalculateVisiblePlanesOfTargetToSensor(
    const Eigen::Affine3d& ego_pose, const Eigen::Affine3d& target_pose,
    const Eigen::Affine3d& sensor_pose_in_ego_frame,
    const Eigen::Vector3d& target_size,
    std::vector<FiniteRectangle>& visible_planes) {
  // step 1: calculate target_center_pose_in_sensor_frame
  const Eigen::Affine3d sensor_pose_in_world_frame =
      ego_pose * sensor_pose_in_ego_frame;
  const Eigen::Vector3d target_center_in_target_frame(0.5 * target_size(0), 0,
                                                      -0.5 * target_size(2));
  // target pose is pose of rear wheel center
  const Eigen::Affine3d target_center_pose = icp_cov::utils::RtToAffine3d(
      target_pose.rotation(), target_pose * target_center_in_target_frame);
  const Eigen::Affine3d target_center_pose_in_sensor_frame =
      sensor_pose_in_world_frame.inverse() * target_center_pose;
  // NOTE: all elements in step 2&3 are in sensor frame
  // step 2: calculate visible planes of target
  // step 2.1: find the anchor point (the closest point to sensor)
  const std::vector<Eigen::Vector3d>
      center_points_of_four_vertical_line_in_target_center_frame = {
          Eigen::Vector3d(-0.5 * target_size(0), -0.5 * target_size(1), 0),
          Eigen::Vector3d(0.5 * target_size(0), -0.5 * target_size(1), 0),
          Eigen::Vector3d(0.5 * target_size(0), 0.5 * target_size(1), 0),
          Eigen::Vector3d(-0.5 * target_size(0), 0.5 * target_size(1), 0)};
  std::vector<Eigen::Vector3d>
      center_points_of_four_vertical_line_in_sensor_frame;
  icp_cov::utils::TransformPoints(
      center_points_of_four_vertical_line_in_target_center_frame,
      target_center_pose_in_sensor_frame,
      center_points_of_four_vertical_line_in_sensor_frame);
  int anchor_point_index = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (int i = 0;
       i < center_points_of_four_vertical_line_in_sensor_frame.size(); ++i) {
    const double distance =
        center_points_of_four_vertical_line_in_sensor_frame.at(i).norm();
    if (distance < min_distance) {
      anchor_point_index = i;
      min_distance = distance;
    }
  }
  assert(anchor_point_index >= 0);
  // step 2.2: generate planes from neighbouring three point of anchor point
  auto generate_plane_with_two_point =
      [](const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
         const Eigen::Affine3d& target_center_pose_in_sensor_frame,
         const Eigen::Vector3d& target_size,
         std::vector<FiniteRectangle>& visible_planes) {
        const Eigen::Vector3d length_direction = point1 - point2;
        const Eigen::Vector3d normalized_length_direction =
            length_direction.normalized();
        const double length = length_direction.norm();
        assert(std::fabs(length - target_size(0)) < 1.0e-6 ||
               std::fabs(length - target_size(1)) < 1.0e-6);
        const Eigen::Vector3d width_direction =
            target_center_pose_in_sensor_frame.rotation() *
            Eigen::Vector3d(0, 0, 1);
        const Eigen::Vector3d normalized_width_direction =
            width_direction.normalized();
        const double width = target_size(2);
        const Eigen::Vector3d center_point = (point1 + point2) * 0.5;
        const Eigen::Vector3d normal =
            (icp_cov::utils::SkewMatrix(normalized_length_direction) *
             normalized_width_direction)
                .normalized();
        visible_planes.emplace_back(
            FiniteRectangle(center_point, normal, normalized_length_direction,
                            length, normalized_width_direction, width));
      };
  visible_planes.clear();
  generate_plane_with_two_point(
      center_points_of_four_vertical_line_in_sensor_frame.at(
          anchor_point_index),
      center_points_of_four_vertical_line_in_sensor_frame.at(
          (anchor_point_index - 1 + 4) % 4),
      target_center_pose_in_sensor_frame, target_size, visible_planes);
  generate_plane_with_two_point(
      center_points_of_four_vertical_line_in_sensor_frame.at(
          anchor_point_index),
      center_points_of_four_vertical_line_in_sensor_frame.at(
          (anchor_point_index + 1 + 4) % 4),
      target_center_pose_in_sensor_frame, target_size, visible_planes);
}
}  // namespace utils
}  // namespace icp_cov
