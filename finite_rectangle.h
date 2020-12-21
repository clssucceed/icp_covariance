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
                  const double width);
  FiniteRectangle() = delete;
  bool CalculateIntersectionPointWithOneRay(
      const double hangle, const double vangle,
      Eigen::Vector3d& intersection_point) const;
  bool IsOccludedBy(const FiniteRectangle& other) const;
  std::vector<Eigen::Vector3d> points_on_plane() const {
    return points_on_plane_;
  };
  void Debug() const;

 private:
  bool PointIsInFiniteRectangle(const Eigen::Vector3d& point) const;
  bool PointIsOccluded(const Eigen::Vector3d& point) const;
  void GeneratePointsOnPlane();
  bool CalculateIntersectionPointWithOneRay(
      const Eigen::Vector3d& direction,
      Eigen::Vector3d& intersection_point) const;

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