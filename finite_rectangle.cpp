#include "finite_rectangle.h"

#include "config/config.h"
#include "utils.h"

namespace icp_cov {

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

FiniteRectangle::FiniteRectangle(
    const Eigen::Vector3d& center_point, const Eigen::Vector3d& normal,
    const Eigen::Vector3d& normalized_length_direction, const double length,
    const Eigen::Vector3d& normalized_width_direction, const double width)
    : center_point_(center_point),
      normal_(normal),
      normalized_length_direction_(normalized_length_direction),
      length_(length),
      normalized_width_direction_(normalized_width_direction),
      width_(width) {
  assert(normal_.norm() == 1.0);
  assert(normalized_length_direction_.norm() == 1.0);
  assert(normalized_width_direction_.norm() == 1.0);
  Eigen::Vector3d point0 = center_point_ -
                           0.5 * length_ * normalized_length_direction_ -
                           0.5 * width_ * normalized_width_direction_;
  Eigen::Vector3d point1 = center_point_ +
                           0.5 * length_ * normalized_length_direction_ -
                           0.5 * width_ * normalized_width_direction_;
  Eigen::Vector3d point2 = center_point_ +
                           0.5 * length_ * normalized_length_direction_ +
                           0.5 * width_ * normalized_width_direction_;
  Eigen::Vector3d point3 = center_point_ -
                           0.5 * length_ * normalized_length_direction_ +
                           0.5 * width_ * normalized_width_direction_;
  four_corner_points_.clear();
  four_corner_points_.emplace_back(point0);
  four_corner_points_.emplace_back(point1);
  four_corner_points_.emplace_back(point2);
  four_corner_points_.emplace_back(point3);
  GeneratePointsOnPlane();
}

bool FiniteRectangle::CalculateIntersectionPointWithOneRay(
    const Eigen::Vector3d& direction,
    Eigen::Vector3d& intersection_point) const {
  // Assume d is distance between sensor origin and generated_point,
  // then generated_point is
  // d * (cos(vangle)*cos(hangle),cos(vangle)sin(hangle), -sin(hangle)),
  // then from
  // (generated_point - center_point_of_plane)^T * normal_of_plane = 0,
  // we can get
  // d = center_point_of_plane^T * normal_of_plane /
  // (cos(vangle)*cos(hangle), cos(vangle)sin(hangle), -sin(hangle))^T *
  // normal_of_plane
  const Eigen::Vector3d normalized_intersection_point = direction.normalized();
  const double temp = normalized_intersection_point.dot(normal_);
  if (std::fabs(temp) < 1.0e-6) return false;
  double d = center_point_.dot(normal_) / temp;
  d = d > 0 ? d : -d;
  intersection_point = d * normalized_intersection_point;
  return PointIsInFiniteRectangle(intersection_point);
}

bool FiniteRectangle::CalculateIntersectionPointWithOneRay(
    const double hangle, const double vangle,
    Eigen::Vector3d& intersection_point) const {
  const Eigen::Vector3d direction(std::cos(vangle) * std::cos(hangle),
                                  std::cos(vangle) * std::sin(hangle),
                                  -sin(vangle));
  return CalculateIntersectionPointWithOneRay(direction, intersection_point);
}

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

bool FiniteRectangle::PointIsOccluded(const Eigen::Vector3d& point) const {
  // calculate intersection_point between plane and this point ray
  // (which starts from origin) and check whether intersection point is in
  // finite rectangle(if in, then occluded)
  Eigen::Vector3d intersection_point;
  bool is_occluded_by_plane =
      CalculateIntersectionPointWithOneRay(point, intersection_point);
  return is_occluded_by_plane;
}

void FiniteRectangle::Debug() const {
  std::cout << "center_point_: " << center_point_.transpose() << std::endl;
  std::cout << "normal_: " << normal_.transpose() << std::endl;
  std::cout << "normalized_length_direction_: "
            << normalized_length_direction_.transpose() << std::endl;
  std::cout << "normalized_width_direction_: "
            << normalized_width_direction_.transpose() << std::endl;
  std::cout << "length_: " << length_ << std::endl;
  std::cout << "width_: " << width_ << std::endl;
  std::cout << "corner_points: ";
  for (const auto& point : four_corner_points_) {
    std::cout << point.transpose() << ", ";
  }
  std::cout << std::endl;
}

bool FiniteRectangle::PointIsInFiniteRectangle(
    const Eigen::Vector3d& point) const {
  // if ProjectToNormal(X - center_point) == 0 &&
  // std::fabs(ProjectionToLengthDirection(X - center_point))  <  0.5 *
  // length, then X is in this finite rectangle
  bool condition1 = std::fabs(normal_.dot(point - center_point_)) < 1.0e-6;
  bool condition2 =
      std::fabs(normalized_length_direction_.dot(point - center_point_)) <
      0.5 * length_ + 1.0e-6;
  bool condition3 =
      std::fabs(normalized_width_direction_.dot(point - center_point_)) <
      0.5 * width_ + 1.0e-6;
  return condition1 && condition2 & condition3;
}
}  // namespace icp_cov