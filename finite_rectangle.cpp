#include "finite_rectangle.h"

#include "config/config.h"
#include "utils.h"

namespace icp_cov {

/**
 * Assuming two planes are both represented in sensor frame.
 * How: if four corner points of this finite rectangle is occluded by the other
 * one, this one is considered as occluded.
 */
bool FiniteRectangle::IsOccludedBy(const FiniteRectangle& other) const {
  int occlusion_number = 0;
  for (const auto& corner_point : four_corner_points_) {
    if (other.PointIsOccluded(corner_point)) occlusion_number++;
  }
  return occlusion_number == 4;
}

/**
 *
 */
bool FiniteRectangle::PointIsOccluded(const Eigen::Vector3d& point) const {
  // calculate intersection_point between plane and this point ray
  // (which starts from origin) and check whether intersection point is in
  // finite rectangle(if in, then occluded)
  Eigen::Vector3d intersection_point;
  bool is_occluded_by_plane =
      CalculateIntersectionPointWithOneRay(point, intersection_point);
  return is_occluded_by_plane;
}

void FiniteRectangle::GeneratePointsOnPlane() {
  // step 0: read config and calculate point number
  const double cell_size =
      icp_cov::Config::Instance()->kCellSizeOnTarget;  // unit: m
  const int half_point_number_in_length_direction =
      std::floor(length_ * 0.5 / cell_size);
  const int half_point_number_in_width_direction =
      std::floor(width_ * 0.5 / cell_size);
  // step 1: generate points in finite rectangle center frame
  static std::vector<Eigen::Vector3d> points_in_finite_rectangle_center_frame;
  points_in_finite_rectangle_center_frame.clear();
  for (int index_length = -half_point_number_in_length_direction;
       index_length <= half_point_number_in_length_direction; index_length++) {
    for (int index_width = -half_point_number_in_width_direction;
         index_width <= half_point_number_in_width_direction; index_width++) {
      points_in_finite_rectangle_center_frame.emplace_back(Eigen::Vector3d(
          index_length * cell_size, index_width * cell_size, 0));
    }
  }
  // step 2: transform points to sensor frame
  Eigen::Matrix3d R_finite_rectangle_center_frame_in_sensor_frame;
  R_finite_rectangle_center_frame_in_sensor_frame.col(0) =
      normalized_length_direction_;
  R_finite_rectangle_center_frame_in_sensor_frame.col(1) =
      normalized_width_direction_;
  R_finite_rectangle_center_frame_in_sensor_frame.col(2) =
      icp_cov::utils::SkewMatrix(normalized_length_direction_) *
      normalized_width_direction_;
  const Eigen::Vector3d t_finite_rectangle_center_frame_in_sensor_frame =
      center_point_;
  const Eigen::Affine3d T_finite_rectangle_center_frame_in_sensor_frame =
      icp_cov::utils::RtToAffine3d(
          R_finite_rectangle_center_frame_in_sensor_frame,
          t_finite_rectangle_center_frame_in_sensor_frame);
  icp_cov::utils::TransformPoints(
      points_in_finite_rectangle_center_frame,
      T_finite_rectangle_center_frame_in_sensor_frame, points_on_plane_);
}
}  // namespace icp_cov