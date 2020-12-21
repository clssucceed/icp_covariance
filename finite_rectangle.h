#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace icp_cov {
class FiniteRectangle {
 public:
  FiniteRectangle(const Eigen::Vector3d& center_point,
                  const Eigen::Vector3d& normal,
                  const Eigen::Vector3d& normalized_length_direction,
                  const double length,
                  const Eigen::Vector3d& normalized_width_direction,
                  const double width)
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
  FiniteRectangle() = delete;
  bool CalculateIntersectionPointWithOneRay(
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
    const Eigen::Vector3d normalized_intersection_point =
        direction.normalized();
    const double temp = normalized_intersection_point.dot(normal_);
    if (std::fabs(temp) < 1.0e-6) return false;
    double d = center_point_.dot(normal_) / temp;
    d = d > 0 ? d : -d;
    intersection_point = d * normalized_intersection_point;
    return PointIsInFiniteRectangle(intersection_point);
  }
  bool CalculateIntersectionPointWithOneRay(
      const double hangle, const double vangle,
      Eigen::Vector3d& intersection_point) const {
    const Eigen::Vector3d direction(std::cos(vangle) * std::cos(hangle),
                                    std::cos(vangle) * std::sin(hangle),
                                    -sin(vangle));
    return CalculateIntersectionPointWithOneRay(direction, intersection_point);
  }
  bool IsOccludedBy(const FiniteRectangle& other) const;
  std::vector<Eigen::Vector3d> points_on_plane() const {
    return points_on_plane_;
  };

 public:
  void Debug() const {
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

 private:
  bool PointIsInFiniteRectangle(const Eigen::Vector3d& point) const {
    // if ProjectToNormal(X - center_point) == 0 &&
    // std::fabs(ProjectionToLengthDirection(X - center_point))  <  0.5 *
    // length, then X is in this finite rectangle
    bool condition1 = std::fabs(normal_.dot(point - center_point_)) < 1.0e-6;
    bool condition2 =
        std::fabs(normalized_length_direction_.dot(point - center_point_)) <
        0.5 * length_;
    bool condition3 =
        std::fabs(normalized_width_direction_.dot(point - center_point_)) <
        0.5 * width_;
    return condition1 && condition2 & condition3;
  }
  bool PointIsOccluded(const Eigen::Vector3d& point) const;
  void GeneratePointsOnPlane();

 private:
  // all in sensor frame
  const Eigen::Vector3d center_point_;
  const Eigen::Vector3d normal_;
  const Eigen::Vector3d normalized_length_direction_;
  const Eigen::Vector3d normalized_width_direction_;
  const double length_;
  const double width_;
  std::vector<Eigen::Vector3d> four_corner_points_;
  std::vector<Eigen::Vector3d> points_on_plane_;
};
}  // namespace icp_cov