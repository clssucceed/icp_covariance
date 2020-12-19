#pragma once
#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
class DataGenerator {
 public:
  static DataGenerator* Instance();

 public:
  void Generate();  // interface to generate new data

  const std::vector<Eigen::Vector3d>& pcl1_in_ego_frame() const {
    return pcl1_in_ego_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_world_frame() const {
    return pcl1_in_world_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ego_frame() const {
    return pcl2_in_ego_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_world_frame() const {
    return pcl2_in_world_frame_;
  }
  const Eigen::Affine3d& icp_transform() const { return icp_transform_; }
  const std::vector<Eigen::Vector3d>& pcl1_in_ego_frame_with_noise() const {
    return pcl1_in_ego_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_world_frame_with_noise() const {
    return pcl1_in_world_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ego_frame_with_noise() const {
    return pcl2_in_ego_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_world_frame_with_noise() const {
    return pcl2_in_world_frame_with_noise_;
  }
  const Eigen::Affine3d& icp_transform_init() const {
    return icp_transform_init_;
  }
  const Eigen::Affine3d& ego_pose1() const { return ego_pose1_; }
  const Eigen::Affine3d& ego_pose2() const { return ego_pose2_; }
  const Eigen::Affine3d& target_pose1() const { return target_pose1_; }
  const Eigen::Affine3d& target_pose2() const { return target_pose2_; }

  const Eigen::Affine3d& ref_pose() const { return ref_pose_; }
  const std::vector<Eigen::Vector3d>& pcl1_in_ref_frame() const {
    return pcl1_in_ref_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ref_frame() const {
    return pcl2_in_ref_frame_;
  }
  const std::vector<Eigen::Vector3d>& pcl1_in_ref_frame_with_noise() const {
    return pcl1_in_ref_frame_with_noise_;
  }
  const std::vector<Eigen::Vector3d>& pcl2_in_ref_frame_with_noise() const {
    return pcl2_in_ref_frame_with_noise_;
  }

 private:
  static DataGenerator* data_generator_;

 public:
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
    }
    FiniteRectangle() = delete;
    bool CalculateIntersectionPointWithOneRay(
        const double hangle, const double vangle,
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
      const Eigen::Vector3d normalized_intersection_point(
          std::cos(vangle) * std::cos(hangle),
          std::cos(vangle) * std::sin(hangle), -sin(vangle));
      const double d = center_point_.dot(normal_) /
                       normalized_intersection_point.dot(normal_);
      assert(std::isnan(d) == false);
      assert(d > 1.0e-6);
      intersection_point = d * normalized_intersection_point;
      return PointIsInFiniteRectangle(intersection_point);
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
      return condition1 && condition2;
    }

   private:
    const Eigen::Vector3d center_point_;
    const Eigen::Vector3d normal_;
    const Eigen::Vector3d normalized_length_direction_;
    const Eigen::Vector3d normalized_width_direction_;
    const double length_;
    const double width_;
  };

 private:
  DataGenerator() { Generate(); }

  void Generate2d();
  void Generate3d();

  void GeneratePoints(const Eigen::Affine3d& ego_pose,
                      const Eigen::Affine3d& target_pose,
                      const Eigen::Vector3d& target_size,
                      const double angle_resolution,
                      std::vector<Eigen::Vector3d>& pcl_in_ego_frame);
  void GeneratePointsOnLineSegmentInEgoFrame(
      const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
      const double angle_resolution,
      std::vector<Eigen::Vector3d>& generated_points);
  void GeneratePointsInEgoFrame3dVersion(
      const Eigen::Affine3d& ego_pose, const Eigen::Affine3d& target_pose,
      std::vector<Eigen::Vector3d>& pcl_in_ego_frame);
  bool GenerateOnePointFromHangleAndVangle(
      const double hangle, const double vangle,
      const std::vector<FiniteRectangle>& visible_planes,
      Eigen::Vector3d& generated_point);
  void AddNoiseToPoints(
      const std::vector<Eigen::Vector3d>& pcl_in_ego_frame,
      std::vector<Eigen::Vector3d>& pcl_in_ego_frame_with_noise);
  void AddNoiseToIcpTransform();
  void Reset() {
    pcl1_in_ego_frame_.clear();
    pcl2_in_ego_frame_.clear();
    pcl1_in_world_frame_.clear();
    pcl2_in_world_frame_.clear();
    pcl1_in_ego_frame_with_noise_.clear();
    pcl2_in_ego_frame_with_noise_.clear();
    pcl1_in_world_frame_with_noise_.clear();
    pcl2_in_world_frame_with_noise_.clear();
    pcl1_in_ref_frame_.clear();
    pcl2_in_ref_frame_.clear();
    pcl1_in_ref_frame_with_noise_.clear();
    pcl2_in_ref_frame_with_noise_.clear();
  }

 private:
  // ground truth
  std::vector<Eigen::Vector3d> pcl1_in_ego_frame_;
  std::vector<Eigen::Vector3d> pcl2_in_ego_frame_;
  std::vector<Eigen::Vector3d> pcl1_in_world_frame_;
  std::vector<Eigen::Vector3d> pcl2_in_world_frame_;
  Eigen::Affine3d icp_transform_;
  Eigen::Affine3d ego_pose1_;
  Eigen::Affine3d ego_pose2_;
  Eigen::Affine3d target_pose1_;
  Eigen::Affine3d target_pose2_;

  // with noise
  std::vector<Eigen::Vector3d> pcl1_in_ego_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl2_in_ego_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl1_in_world_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl2_in_world_frame_with_noise_;
  Eigen::Affine3d icp_transform_init_;

  // for visualization
  Eigen::Affine3d ref_pose_;
  std::vector<Eigen::Vector3d> pcl1_in_ref_frame_;
  std::vector<Eigen::Vector3d> pcl2_in_ref_frame_;
  std::vector<Eigen::Vector3d> pcl1_in_ref_frame_with_noise_;
  std::vector<Eigen::Vector3d> pcl2_in_ref_frame_with_noise_;
};
}  // namespace icp_cov