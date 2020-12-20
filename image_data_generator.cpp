#include "image_data_generator.h"

#include "config/config.h"
#include "utils.h"

namespace icp_cov {
ImageDataGenerator* ImageDataGenerator::image_data_generator_ = nullptr;

const ImageDataGenerator* ImageDataGenerator::Instance() {
  if (image_data_generator_ == nullptr) {
    image_data_generator_ = new ImageDataGenerator();
  }
  return image_data_generator_;
}

void ImageDataGenerator::Generate() {
  // step 1: reset
  Reset();
  // step 2: get parameters from config
  // step 2.1: set ego pose
  auto config = icp_cov::Config::Instance();
  ego_pose1_in_world_frame_gt_ = config->kEgoPose1InWorldFrame;
  ego_pose2_in_world_frame_gt_ = config->kEgoPose2InWorldFrame;
  // step 2.2: set target pose
  target_pose1_in_world_frame_gt_ =
      ego_pose1_in_world_frame_gt_ * config->kTargetPose1InEgo1Frame;
  target_pose2_in_world_frame_gt_ =
      ego_pose2_in_world_frame_gt_ * config->kTargetPose2InEgo2Frame;
  // step 3: generate image data
  GenerateImageData();
  // step 4: add noise
  AddNoise();
}

void ImageDataGenerator::GenerateImageData() {
  // step 1: calculate target pose in camera frame

  // step 2: calculate world points which is visible in both frame and spatially
  // ditributed uniformly
  // step 2.1: calculate visible region on target for both camera

  // step 2.2: uniformly sample visible region to get world points

  // step 3: project world points to obtain image points in both camera
}

void ImageDataGenerator::AddNoise() {
  // step 1: add noise to image points
  AddNoiseToImagePoints(image_points1_gt_, image_points1_with_noise_);
  AddNoiseToImagePoints(image_points2_gt_, image_points2_with_noise_);
  // step 2: add noise to world points
  AddNoiseToWorldPoints(world_points_gt_, world_points_with_noise_);
}

void ImageDataGenerator::AddNoiseToImagePoints(
    const std::vector<Eigen::Vector3d>& gt_points,
    std::vector<Eigen::Vector3d>& noised_points) {
  const double image_noise_sigma = 1.0;
  noised_points.clear();
  noised_points.reserve(gt_points.size());
  for (const auto& point : gt_points) {
    noised_points.emplace_back(point +
                               icp_cov::utils::PointNoise2d(image_noise_sigma));
  }
}

void ImageDataGenerator::AddNoiseToWorldPoints(
    const std::vector<Eigen::Vector3d>& gt_points,
    std::vector<Eigen::Vector3d>& noised_points) {
  const double world_point_noise_sigma = 1.0;
  noised_points.clear();
  noised_points.reserve(gt_points.size());
  for (const auto& point : gt_points) {
    noised_points.emplace_back(
        point + icp_cov::utils::PointNoise3d(world_point_noise_sigma));
  }
}
}  // namespace icp_cov