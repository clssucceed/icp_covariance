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
  // step 1: generate target_points_in_target_frame_gt_ in frame 1
  GenerateTargetPointsInTargetFrame();
  // step 2: project target points to obtain image points in both camera
  ProjectTargetPointsToCameras();
}

void ImageDataGenerator::GenerateTargetPointsInTargetFrame() {
  // step 1: get relevant parameters
  const Eigen::Affine3d ego_pose = ego_pose1_in_world_frame_gt_;
  const Eigen::Affine3d target_pose = target_pose1_in_world_frame_gt_;
  auto config = icp_cov::Config::Instance();
  const Eigen::Affine3d camera_pose_in_ego_frame =
      config->kCameraPoseInEgoFrame;
  const Eigen::Vector3d target_size = config->kTargetSize;
  const Eigen::Affine3d target_center_pose_in_target_frame =
      icp_cov::utils::RtToAffine3d(
          Eigen::Matrix3d::Identity(),
          Eigen::Vector3d(0.5 * target_size(0), 0, -0.5 * target_size(2)));

  // step 2: get points in target center frame
  static std::vector<Eigen::Vector3d> target_points_in_target_center_frame;
  target_points_in_target_center_frame.clear();
  // step 2.1: get visible planes
  std::vector<FiniteRectangle> visible_planes;
  icp_cov::utils::CalculateVisiblePlanesOfTargetToSensor(
      ego_pose, target_pose, camera_pose_in_ego_frame, target_size,
      visible_planes);

  // step 2.2: get points on visible planes
  // TODO(clssucceed@gmail.com):
  // 0. 将新封装的生成面的函数在data_generator中测试一下
  // 1. 将该逻辑加到FinitePlane中
  // 2.
  // 需要仔细分析下只看到一个面的情况(即如何识别出返回的两个面中哪一个面是不可见的)

  // step 3: transform points to target frame
  icp_cov::utils::TransformPoints(target_points_in_target_center_frame,
                                  target_center_pose_in_target_frame,
                                  target_points_in_target_frame_gt_);
}

void ImageDataGenerator::ProjectTargetPointsToCameras() {
  ProjectTargetPointsToCamera(
      ego_pose1_in_world_frame_gt_, target_pose1_in_world_frame_gt_,
      target_points_in_target_frame_gt_, image_points1_gt_);
  ProjectTargetPointsToCamera(
      ego_pose2_in_world_frame_gt_, target_pose2_in_world_frame_gt_,
      target_points_in_target_frame_gt_, image_points2_gt_);
}

void ImageDataGenerator::ProjectTargetPointsToCamera(
    const Eigen::Affine3d& ego_pose_in_world_frame,
    const Eigen::Affine3d& target_pose_in_world_frame,
    const std::vector<Eigen::Vector3d>& target_points_in_target_frame,
    std::vector<Eigen::Vector3d>& image_points) {
  // step 0: get parameters from Config
  auto config = icp_cov::Config::Instance();
  const Eigen::Affine3d camera_pose_in_ego_frame =
      config->kCameraPoseInEgoFrame;
  const Eigen::Matrix3d camera_intrinsic_matrix =
      config->kCameraIntrinsicMatrix;
  const int image_width = config->kImageWidth;
  const int image_height = config->kImageHeight;

  // step 1: calculate target_points in camera frame
  static std::vector<Eigen::Vector3d> target_points_in_camera_frame;
  target_points_in_camera_frame.clear();
  const Eigen::Affine3d target_pose_in_camera_frame =
      camera_pose_in_ego_frame.inverse() * ego_pose_in_world_frame.inverse() *
      target_pose_in_world_frame;
  icp_cov::utils::TransformPoints(target_points_in_target_frame,
                                  target_pose_in_camera_frame,
                                  target_points_in_camera_frame);
  assert(target_points_in_camera_frame.size() ==
         target_points_in_target_frame.size());

  // step 2: project points to camera
  image_points.clear();
  image_points.reserve(target_points_in_camera_frame.size());
  for (const auto& point : target_points_in_camera_frame) {
    Eigen::Vector3d image_point = camera_intrinsic_matrix * point;
    assert(std::fabs(image_point(2)) > 1.0e-6);
    image_point = image_point / image_point(2);
    assert(image_point(0) >= 0);
    assert(image_point(0) < image_width);
    assert(image_point(1) >= 0);
    assert(image_point(1) < image_height);
    assert(std::fabs(image_point(2) - 1.0) > 1.0e-6);
    image_points.emplace_back(image_point);
  }
}

void ImageDataGenerator::AddNoise() {
  // step 1: add noise to image points
  AddNoiseToImagePoints(image_points1_gt_, image_points1_with_noise_);
  AddNoiseToImagePoints(image_points2_gt_, image_points2_with_noise_);
  // step 2: add noise to world points
  AddNoiseToWorldPoints(target_points_in_target_frame_gt_,
                        target_points_in_target_frame_with_noise_);
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