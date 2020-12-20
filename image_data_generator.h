#pragma once
#include <Eigen/Dense>
#include <vector>

namespace icp_cov {
class ImageDataGenerator {
 public:
  static const ImageDataGenerator* Instance();

 public:
  void Generate();

 public:
  // gt
  Eigen::Affine3d ego_pose1_in_world_frame_gt_;
  Eigen::Affine3d ego_pose2_in_world_frame_gt_;
  Eigen::Affine3d target_pose1_in_world_frame_gt_;
  Eigen::Affine3d target_pose2_in_world_frame_gt_;
  std::vector<Eigen::Vector3d> target_points_in_target_frame_gt_;
  std::vector<Eigen::Vector3d> image_points1_gt_;
  std::vector<Eigen::Vector3d> image_points2_gt_;

  // with noise
  std::vector<Eigen::Vector3d> target_points_in_target_frame_with_noise_;
  std::vector<Eigen::Vector3d> image_points1_with_noise_;
  std::vector<Eigen::Vector3d> image_points2_with_noise_;

 private:
  ImageDataGenerator() { Generate(); };

 private:
  void Reset() {
    target_points_in_target_frame_gt_.clear();
    image_points1_gt_.clear();
    image_points2_gt_.clear();
    target_points_in_target_frame_with_noise_.clear();
    image_points1_with_noise_.clear();
    image_points2_with_noise_.clear();
  }
  void GenerateImageData();
  void GenerateTargetPointsInTargetFrame();
  void ProjectTargetPointsToCameras();
  void ProjectTargetPointsToCamera(
      const Eigen::Affine3d& ego_pose_in_world_frame,
      const Eigen::Affine3d& target_pose_in_world_frame,
      const std::vector<Eigen::Vector3d>& target_points_in_target_frame,
      std::vector<Eigen::Vector3d>& image_points);
  void AddNoise();
  void AddNoiseToImagePoints(const std::vector<Eigen::Vector3d>& gt_points,
                             std::vector<Eigen::Vector3d>& noised_points);
  void AddNoiseToWorldPoints(const std::vector<Eigen::Vector3d>& gt_points,
                             std::vector<Eigen::Vector3d>& noised_points);

 private:
  static ImageDataGenerator* image_data_generator_;
};
}  // namespace icp_cov